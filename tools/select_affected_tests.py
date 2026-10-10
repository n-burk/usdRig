#!/usr/bin/env python3
"""Map a source path to the CTest targets that link it.

Reads the root CMakeLists.txt (no configure, no compiler). Prints the
libraries, executables, and ctest names a change would rebuild. The
ctest line uses the labels from cmake/TestLabels.cmake: usd-free when
every direct executable is free of OpenUSD, otherwise the project
libraries the change sits in. bin/test_changed.sh --run executes that
line. This script is not wired into CI.

Limitations are printed with --explain. Schedule/cone variants that
rigexec_add_graph_test() adds are synthesized. Tests registered from
example-fixture lists are named but not expanded (those lists live in
tests/exampleFixtures.cmake).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CMAKE = os.path.join(ROOT, "CMakeLists.txt")

# Imported OpenUSD/OpenExec targets. A test that reaches one of these
# cannot be built or run without the USD prefix.
PXR_LIBS = {
    "arch", "tf", "gf", "vt", "sdf", "ts", "usd", "usdGeom", "usdSkel",
    "plug", "work", "exec", "execUsd", "ef", "vdf", "esf", "hd",
    "usdImaging", "usdUtils", "hio", "sdr", "js", "garch", "glf", "hf",
    "hdSt", "hgi", "trace",
}

# Project libraries whose PUBLIC interface does not pull pxr, from the
# target_link_libraries edges in CMakeLists.txt. Recomputed after parse
# when the file says otherwise.
USD_FREE_SEEDS = {"rigExecGraph", "rigExecBinary", "rigExecRuntime", "rigExecLzma", "rigExecFlatBuffers"}


def strip_comments(text: str) -> str:
    out = []
    i = 0
    n = len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == '"' and text[i - 1] != "\\":
                in_str = False
            i += 1
            continue
        if c == '"':
            in_str = True
            out.append(c)
            i += 1
            continue
        if c == "#":
            while i < n and text[i] != "\n":
                i += 1
            continue
        out.append(c)
        i += 1
    return "".join(out)


def split_args(body: str) -> list[str]:
    args = []
    cur = []
    depth = 0
    in_str = False
    for c in body:
        if in_str:
            cur.append(c)
            if c == '"':
                in_str = False
            continue
        if c == '"':
            in_str = True
            cur.append(c)
            continue
        if c in "([{":
            depth += 1
            cur.append(c)
            continue
        if c in ")]}":
            depth = max(0, depth - 1)
            cur.append(c)
            continue
        if c.isspace() and depth == 0:
            token = "".join(cur).strip()
            if token:
                args.append(token)
            cur = []
            continue
        cur.append(c)
    token = "".join(cur).strip()
    if token:
        args.append(token)
    return args


def iter_commands(text: str):
    """Yield (name, args) for top-level-looking commands, paren-balanced."""
    i = 0
    n = len(text)
    ident = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
    while True:
        m = ident.search(text, i)
        if not m:
            return
        j = m.end()
        while j < n and text[j].isspace():
            j += 1
        if j >= n or text[j] != "(":
            i = m.end()
            continue
        depth = 0
        k = j
        in_str = False
        while k < n:
            c = text[k]
            if in_str:
                if c == '"':
                    in_str = False
            elif c == '"':
                in_str = True
            elif c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    yield m.group(0), split_args(text[j + 1 : k])
                    i = k + 1
                    break
            k += 1
        else:
            return


def expand_foreach(text: str) -> str:
    """Expand foreach(var literals) and foreach(var IN ITEMS literals)."""
    pattern = re.compile(
        r"foreach\s*\(\s*([A-Za-z_][A-Za-z0-9_]*)\s+(IN\s+ITEMS\s+)?(.*?)\)\s*(.*?)\s*endforeach\s*\(\s*\)",
        re.DOTALL,
    )

    def repl(match: re.Match) -> str:
        var = match.group(1)
        items_kw = match.group(2)
        raw_items = match.group(3)
        body = match.group(4)
        if "IN LISTS" in raw_items or (items_kw is None and "IN " in raw_items.split()[:1]):
            return match.group(0)
        if "IN LISTS" in match.group(0)[: match.group(0).find(")")]:
            return match.group(0)
        items = split_args(raw_items)
        if any(tok in {"IN", "LISTS", "RANGE"} for tok in items):
            return match.group(0)
        chunks = []
        for item in items:
            chunks.append(body.replace("${" + var + "}", item))
        return "\n".join(chunks)

    prev = None
    cur = text
    # Innermost first: keep going while a foreach remains that we can expand.
    for _ in range(8):
        nxt = pattern.sub(repl, cur)
        if nxt == cur:
            break
        cur = nxt
    return cur


def synthesize_graph_tests(text: str) -> str:
    """Append the tests rigexec_add_graph_test() registers."""
    extra = []
    for m in re.finditer(
        r"rigexec_add_graph_test\s*\(\s*([A-Za-z0-9_]+)([^)]*)\)", text
    ):
        name = m.group(1)
        flags = m.group(2)
        cone = "CONE_VERIFY" in flags
        separate = "SEPARATE_PARALLEL" in flags
        if separate:
            extra.append(f'add_test(NAME {name}Graph_parallel COMMAND {name})')
        extra.append(f'add_test(NAME {name}Graph_serial COMMAND {name})')
        if cone:
            for schedule in ("serial", "parallel"):
                extra.append(
                    f'add_test(NAME {name}Cones_{schedule} COMMAND {name})'
                )
                extra.append(
                    f'add_test(NAME {name}Cones_grain0_{schedule} COMMAND {name})'
                )
    return text + "\n" + "\n".join(extra) + "\n"


def parse(cmake_text: str) -> dict:
    text = synthesize_graph_tests(expand_foreach(strip_comments(cmake_text)))
    libraries = {}  # name -> {sources, links, kind}
    executables = {}  # name -> {sources, links}
    tests = []  # {name, command, executable}

    current_kind = None
    for name, args in iter_commands(text):
        if name == "add_library" and args:
            lib = args[0]
            kind = "STATIC"
            rest = args[1:]
            if rest and rest[0] in {"STATIC", "SHARED", "INTERFACE", "OBJECT", "MODULE"}:
                kind = rest[0]
                rest = rest[1:]
            sources = [a for a in rest if not a.startswith("$")]
            libraries[lib] = {"kind": kind, "sources": sources, "links": []}
        elif name == "add_executable" and args:
            exe = args[0]
            sources = [a for a in args[1:] if not a.startswith("$")]
            executables[exe] = {"sources": sources, "links": []}
        elif name == "target_link_libraries" and len(args) >= 2:
            target = args[0]
            libs = [a for a in args[1:] if a not in {"PUBLIC", "PRIVATE", "INTERFACE"}]
            if target in libraries:
                libraries[target]["links"].extend(libs)
            elif target in executables:
                executables[target]["links"].extend(libs)
        elif name == "add_test":
            if "NAME" in args and "COMMAND" in args:
                tname = args[args.index("NAME") + 1]
                command = args[args.index("COMMAND") + 1 :]
            elif args:
                tname = args[0]
                command = args[1:]
            else:
                continue
            exe = command[0] if command else ""
            exe = exe.strip('"')
            # $<TARGET_FILE:foo> and ${name} already expanded when literal.
            gen = re.search(r"TARGET_FILE:([A-Za-z0-9_]+)", exe)
            if gen:
                exe = gen.group(1)
            if exe in {"${CMAKE_COMMAND}", "cmake"} or exe.endswith(".cmake"):
                exe = ""
            tests.append({"name": tname, "executable": exe, "command": command})

    deduped = []
    seen_tests = set()
    for test in tests:
        if test["name"] in seen_tests:
            continue
        seen_tests.add(test["name"])
        deduped.append(test)
    return {"libraries": libraries, "executables": executables, "tests": deduped}


def closure(start: set[str], edges: dict[str, list[str]]) -> set[str]:
    seen = set(start)
    stack = list(start)
    while stack:
        cur = stack.pop()
        for nxt in edges.get(cur, []):
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    return seen


def build_index(model: dict) -> dict:
    lib_edges = {n: info["links"] for n, info in model["libraries"].items()}
    exe_edges = {}
    for name, info in model["executables"].items():
        exe_edges[name] = info["links"]
    # Library public/private are not distinguished: both end up on the link line.
    source_to_lib = defaultdict(list)
    for lib, info in model["libraries"].items():
        for src in info["sources"]:
            source_to_lib[os.path.normpath(src)].append(lib)

    exe_closure = {}
    for exe, info in model["executables"].items():
        reached = closure(set(info["links"]), {**lib_edges, **exe_edges})
        exe_closure[exe] = sorted(reached)

    tests_by_exe = defaultdict(list)
    for test in model["tests"]:
        if test["executable"]:
            tests_by_exe[test["executable"]].append(test["name"])

    return {
        "source_to_lib": source_to_lib,
        "exe_closure": exe_closure,
        "tests_by_exe": tests_by_exe,
        "lib_edges": lib_edges,
    }


def usd_free_exes(model: dict, index: dict) -> list[str]:
    free = []
    for exe, reached in index["exe_closure"].items():
        pxr = [lib for lib in reached if lib in PXR_LIBS or lib.startswith("usd")]
        project = [lib for lib in reached if lib in model["libraries"]]
        # rigExecMath's link line carries gf/vt/tf/arch. Catch that via closure.
        if pxr:
            continue
        if any(lib not in USD_FREE_SEEDS and lib in model["libraries"] for lib in project):
            # A project lib outside the USD-free seeds. Still free if THAT
            # lib's own closure has no pxr. closure() already walked links,
            # so absence of pxr above is the real test.
            pass
        if not pxr:
            free.append(exe)
    return sorted(free)


def includes_header(path: str, header_rel: str) -> bool:
    """True when a C++ file includes header_rel or its basename."""
    base = os.path.basename(header_rel)
    stem_pat = re.compile(
        r'#\s*include\s*[<"]([^>"]*%s)[>"]' % re.escape(base)
    )
    try:
        with open(path, "r", errors="replace") as fh:
            text = fh.read()
    except OSError:
        return False
    for m in stem_pat.finditer(text):
        inc = m.group(1).replace("\\", "/")
        if inc.endswith(header_rel) or inc.endswith(base):
            # Basename collisions exist (opGraph.h). Prefer a path suffix.
            if header_rel.endswith(inc) or inc.endswith("/" + base) or inc == base:
                if "/" in header_rel and "/" not in inc and base in {
                    "opGraph.h", "runtime.h", "format.h", "values.h"
                }:
                    # Ambiguous short include: accept only an exact basename
                    # match when the including file lives next to it.
                    inc_dir = os.path.dirname(path)
                    if os.path.exists(os.path.join(inc_dir, base)):
                        return os.path.normpath(os.path.join(inc_dir, base)).endswith(
                            header_rel
                        ) or header_rel.endswith(base)
                return True
    return False


def files_including(header_rel: str) -> list[str]:
    hits = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [
            d
            for d in dirnames
            if d not in {".git", "build", "third_party", "thirdparty", "plugin"}
        ]
        for fn in filenames:
            if not fn.endswith((".h", ".cpp", ".c")):
                continue
            full = os.path.join(dirpath, fn)
            rel = os.path.normpath(os.path.relpath(full, ROOT))
            if rel == os.path.normpath(header_rel):
                continue
            if includes_header(full, os.path.normpath(header_rel)):
                hits.append(rel)
    return hits


def select(paths: list[str], model: dict, index: dict, follow_headers: bool) -> dict:
    libs = set()
    reasons = defaultdict(list)
    header_fans = {}
    for raw in paths:
        rel = os.path.normpath(raw)
        if rel.startswith("libs" + os.sep) or rel.startswith("tests" + os.sep):
            owners = index["source_to_lib"].get(rel, [])
            if owners:
                for lib in owners:
                    libs.add(lib)
                    reasons[lib].append(rel + " (compiled into this target)")
            elif follow_headers and rel.endswith((".h", ".hpp")):
                includers = files_including(rel)
                header_fans[rel] = includers
                for inc in includers:
                    for lib in index["source_to_lib"].get(inc, []):
                        libs.add(lib)
                        reasons[lib].append(inc + " includes " + rel)
                    # Test translation units are not libraries.
                    for exe, info in model["executables"].items():
                        if inc in info["sources"] or any(
                            inc.endswith(s) for s in info["sources"]
                        ):
                            libs.add("exe:" + exe)
                            reasons["exe:" + exe].append(inc + " includes " + rel)
            elif rel.endswith(".cpp"):
                reasons["unmapped"].append(rel)
        elif rel.endswith("CMakeLists.txt") or rel.startswith("cmake" + os.sep):
            libs.add("*")
            reasons["*"].append(rel)
        elif "schema.usda" in rel:
            libs.add("rigExec")
            reasons["rigExec"].append(rel + " (schema; consumers are the USD evaluator)")

    pathset = {os.path.normpath(p) for p in paths}
    exes = []
    direct_exes = []
    if "*" in libs:
        exes = sorted(model["executables"])
        direct_exes = exes
    else:
        wanted = set()
        direct = set()
        for exe, info in model["executables"].items():
            reached = set(index["exe_closure"][exe])
            linked = set(info["links"])
            if reached & libs or ("exe:" + exe) in libs:
                wanted.add(exe)
            if linked & libs or ("exe:" + exe) in libs:
                direct.add(exe)
            for src in info["sources"]:
                if os.path.normpath(src) in pathset:
                    wanted.add(exe)
                    direct.add(exe)
        exes = sorted(wanted)
        direct_exes = sorted(direct)

    def names_for(exe_list: list[str]) -> list[str]:
        seen = set()
        ordered = []
        for exe in exe_list:
            for name in index["tests_by_exe"].get(exe, []):
                if name not in seen and "${" not in name:
                    seen.add(name)
                    ordered.append(name)
        return ordered

    ordered = names_for(exes)
    direct_tests = names_for(direct_exes)
    return {
        "libraries": sorted(l for l in libs if not l.startswith("exe:")),
        "executables": exes,
        "direct_executables": direct_exes,
        "tests": ordered,
        "direct_tests": direct_tests,
        "reasons": {k: v for k, v in reasons.items()},
        "header_includers": header_fans,
    }


def ctest_regex(names: list[str]) -> str:
    if not names:
        return ""
    return "^(" + "|".join(re.escape(n) for n in names) + ")$"


def snapshot(model: dict, index: dict) -> dict:
    by_lib = defaultdict(list)
    for exe, reached in index["exe_closure"].items():
        for lib in reached:
            if lib in model["libraries"]:
                by_lib[lib].append(exe)
    tests_for_lib = {}
    for lib, exes in by_lib.items():
        names = []
        for exe in sorted(set(exes)):
            names.extend(index["tests_by_exe"].get(exe, []))
        tests_for_lib[lib] = sorted({n for n in names if "${" not in n})
    free = usd_free_exes(model, index)
    return {
        "libraries": {
            name: {
                "kind": info["kind"],
                "sources": info["sources"],
                "links": info["links"],
            }
            for name, info in sorted(model["libraries"].items())
        },
        "executables": {
            name: {"sources": info["sources"], "links": info["links"],
                   "closure": index["exe_closure"][name]}
            for name, info in sorted(model["executables"].items())
        },
        "tests": model["tests"],
        "tests_for_library": tests_for_lib,
        "usd_free_executables": free,
        "notes": [
            "Link edges are not split into PUBLIC and PRIVATE.",
            "rigexec_add_graph_test cone and graph variants are synthesized.",
            "Example-fixture loops (IN LISTS) are not expanded.",
            "A change inside libs/rigExec selects every test that links the rigExec shared library.",
        ],
    }


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", help="repo-relative paths (as from git diff --name-only)")
    parser.add_argument("--json", help="write the full source-to-test snapshot")
    parser.add_argument("--diff", metavar="REV", help="select paths from git diff REV")
    parser.add_argument("--no-headers", action="store_true",
                        help="do not scan includers of changed headers")
    parser.add_argument("--explain", action="store_true")
    args = parser.parse_args(argv)

    with open(CMAKE, "r", encoding="utf-8") as fh:
        model = parse(fh.read())
    index = build_index(model)

    if args.json:
        payload = snapshot(model, index)
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)
            fh.write("\n")
        print(f"wrote {args.json} ({len(model['tests'])} tests, {len(model['executables'])} executables)")

    paths = list(args.paths)
    if args.diff:
        import subprocess
        out = subprocess.check_output(
            ["git", "diff", "--name-only", args.diff], cwd=ROOT, text=True
        )
        paths.extend(line.strip() for line in out.splitlines() if line.strip())

    if not paths and not args.json:
        parser.error("pass paths, --diff REV, or --json FILE")

    if paths:
        result = select(paths, model, index, follow_headers=not args.no_headers)
        print("paths:")
        for p in paths:
            print(f"  {p}")
        print("libraries:")
        for lib in result["libraries"] or ["(none)"]:
            print(f"  {lib}")
        print(
            f"direct executables ({len(result['direct_executables'])}) "
            "(link the changed library themselves):"
        )
        for exe in result["direct_executables"]:
            print(f"  {exe}")
        print(
            f"direct tests ({len(result['direct_tests'])}):"
        )
        for name in result["direct_tests"]:
            print(f"  {name}")
        print(
            f"transitive executables ({len(result['executables'])}) "
            "(link closure, safe rebuild):"
        )
        for exe in result["executables"]:
            print(f"  {exe}")
        print(f"transitive tests ({len(result['tests'])}):")
        for name in result["tests"]:
            print(f"  {name}")
        regex = ctest_regex(result["direct_tests"])
        free = set(usd_free_exes(model, index))
        direct_free = bool(result["direct_executables"]) and all(
            exe in free for exe in result["direct_executables"]
        )
        if direct_free:
            label = "^usd-free$"
        else:
            libs = [lib for lib in result["libraries"] if lib.startswith("rigExec")]
            label = "^(" + "|".join(re.escape(lib) for lib in libs) + ")$" if libs else ""
        if regex and label:
            print("ctest (direct):")
            print(
                f"  ctest --test-dir build -L '{label}' -R '{regex}' "
                "--output-on-failure"
            )
        elif regex:
            print("ctest (direct):")
            print(f"  ctest --test-dir build -R '{regex}' --output-on-failure")
        if result["direct_executables"]:
            print("build (direct):")
            print(
                "  cmake --build build --target "
                + " ".join(result["direct_executables"])
            )
        if args.explain:
            print("why:")
            for key, lines in result["reasons"].items():
                for line in lines:
                    print(f"  {key}: {line}")
            if result["header_includers"]:
                print("header includers:")
                for header, incs in result["header_includers"].items():
                    print(f"  {header}: {len(incs)} files")
        print(
            "note: the direct set is the inner loop. bin/test_changed.sh "
            "--run builds those targets and runs the ctest line. The "
            "transitive set is what to run before pushing. A change inside "
            "the rigExec shared library still selects every test that links it."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
