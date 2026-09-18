#!/usr/bin/env python
"""Generate Houdini-style node pages from schema.usda + operator notes.

Usage: python docs/build_pages.py

Reads attribute documentation (names, types, defaults, allowed tokens,
doc strings) from libs/rigExecSchema/schema.usda, combines it with the
hand-written prose in docs/operator_notes.py, and writes one page per
operator to docs/nodes/<operator>.md plus the gallery at docs/index.md.
Re-running regenerates every page; prose lives only in operator_notes.py.

Concept pages are hand-written Markdown in docs/concepts/ and are not
generated: this module only parses their front matter (see parse_concept)
so the index can list them and build_html.py can render them.
"""
import os
import re
import sys
import textwrap

RIG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RIG, "docs"))
from operator_notes import OPERATORS, CATEGORIES  # noqa: E402

SCHEMA = os.path.join(RIG, "libs", "rigExecSchema", "schema.usda")
NODES_DIR = os.path.join(RIG, "docs", "nodes")
GIFS_DIR = os.path.join(RIG, "docs", "gifs")
CONCEPTS_DIR = os.path.join(RIG, "docs", "concepts")


# ---- concept pages ---------------------------------------------------
# Hand-written prose, one Markdown file per page in docs/concepts/. The
# sources are the Markdown output -- nothing is generated for them here --
# and build_html.py renders the same files into docs/site/concepts/.
#
# A page opens with an optional front matter block:
#
#     ---
#     title: How operators fire
#     summary: One line.
#     order: 20
#     ---
#
# and continues with ordinary Markdown whose own headings start at "##".
# Links and images are written ONCE, relative to docs/: a node page is
# "nodes/<key>.md", an image is "gifs/<name>.gif". build_html.py rewrites
# them to "../nodes/<key>.html" and "../gifs/<name>.gif" for the site.

_FRONT_RE = re.compile(r"\A---[ \t]*\r?\n(.*?)\r?\n---[ \t]*\r?\n?", re.S)
_H1_RE = re.compile(r"^#\s+(.+?)\s*$", re.M)


def _unquote_scalar(text):
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "\"'":
        return text[1:-1]
    return text


def parse_concept(path):
    """One concept source as {slug, title, summary, order, body, path}."""
    slug = os.path.splitext(os.path.basename(path))[0]
    with open(path, encoding="utf-8") as stream:
        text = stream.read()
    meta = {}
    match = _FRONT_RE.match(text)
    if match:
        body = text[match.end():]
        for line in match.group(1).splitlines():
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            name, _, value = line.partition(":")
            meta[name.strip().lower()] = _unquote_scalar(value)
    else:
        body = text
    title = meta.get("title", "")
    if not title:
        # No front matter: the first "# " heading names the page, and is
        # dropped from the body so the rendered title is not printed twice.
        heading = _H1_RE.search(body)
        if heading:
            title = heading.group(1).strip()
            body = body[:heading.start()] + body[heading.end():]
        else:
            title = slug.replace("-", " ").capitalize()
    try:
        order = int(meta.get("order", "1000"))
    except ValueError:
        order = 1000
    return {"slug": slug, "title": title, "summary": meta.get("summary", ""),
            "order": order, "body": body.strip(), "path": path}


def load_concepts():
    """Every docs/concepts/*.md, ordered by front-matter "order" then title."""
    if not os.path.isdir(CONCEPTS_DIR):
        return []
    pages = [parse_concept(os.path.join(CONCEPTS_DIR, name))
             for name in sorted(os.listdir(CONCEPTS_DIR))
             if name.endswith(".md")]
    pages.sort(key=lambda page: (page["order"], page["title"].lower()))
    return pages


def concept_headings(body):
    """The page's "## " headings as [(slug, title)], for "On this page"."""
    out, fenced = [], False
    for line in body.splitlines():
        stripped = line.strip()
        if stripped.startswith("```") or stripped.startswith("~~~"):
            fenced = not fenced
            continue
        if fenced or not stripped.startswith("## "):
            continue
        title = stripped[3:].strip().rstrip("#").strip()
        out.append((concept_slug(title), title))
    return out


def concept_slug(text):
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")


def example_stage(key, note):
    """The stage a page points at, without its ".usda" suffix.

    Usually the page's own; "example_key" borrows another page's stage, and
    an interface page ("no_gif") may have no stage at all.
    """
    if note.get("no_gif"):
        return note.get("example_key")
    return note.get("example_key", key)


def check_examples():
    """Fail on a dangling example_key; warn on a GIF not rendered yet."""
    for key, note in OPERATORS.items():
        borrowed = note.get("example_key")
        if borrowed is not None and borrowed not in OPERATORS:
            raise SystemExit("%s: example_key \"%s\" is not an operator"
                             % (key, borrowed))
        if note.get("no_gif"):
            continue
        if not os.path.isfile(os.path.join(GIFS_DIR, key + ".gif")):
            # Not fatal: docs/render_media.py may simply not have run yet.
            sys.stderr.write("warning: %s: docs/gifs/%s.gif is missing\n"
                             % (key, key))

_CLASS_RE = re.compile(r'^class\s+(?:"([^"]+)"|(\S+))\s*(?:"[^"]*")?\s*\($')
_INHERITS_RE = re.compile(r"inherits\s*=\s*</([^>]+)>")
# Attribute declaration: an optional "custom" and/or "uniform" prefix, a type
# that may carry an array "[]" suffix, the namespaced name, and an optional
# default. Any trailing "( ... )" metadata is split off before matching.
_ATTR_RE = re.compile(
    r"^(?:custom\s+)?(?:(uniform)\s+)?(rel|[A-Za-z][\w]*(?:\[\])?)\s+([\w:.]+)"
    r"(?:\s*=\s*(.*\S))?\s*$")
_TOKENS_RE = re.compile(r"allowedTokens\s*=\s*\[(.*?)\]", re.S)


def _mask_strings(line):
    """Blank the inside of every quoted span, keeping the line's length."""
    out = list(line)
    inside = False
    for index, char in enumerate(line):
        if char == '"':
            inside = not inside
        elif inside:
            out[index] = " "
    return "".join(out)


def _mask_meta_line(line, in_doc):
    """Drop quoted text from one metadata line, tracking open \"\"\" blocks.

    Returns (code, still_in_doc). Parens inside a doc string -- including a
    multi-line one -- must not be counted, or the metadata block appears to
    close early and its closing ")" leaks back into the attribute loop.
    """
    out = []
    index = 0
    while index < len(line):
        if in_doc:
            end = line.find('"""', index)
            if end < 0:
                return "".join(out), True
            in_doc = False
            index = end + 3
            continue
        if line.startswith('"""', index):
            in_doc = True
            index += 3
            continue
        if line[index] == '"':
            end = line.find('"', index + 1)
            index = len(line) if end < 0 else end + 1
            continue
        out.append(line[index])
        index += 1
    return "".join(out), in_doc


def _split_attr_meta(stripped):
    """Split an attribute line into (declaration, metadata, metadata_is_open).

    Handles both ``float x = 0 (`` -- metadata continuing on the following
    lines -- and ``token x = "d" (allowedTokens = ["d"])`` complete on one
    line, without mistaking a parenthesised default such as
    ``matrix4d m = ( (1, 0, 0, 0), ... )`` for metadata.
    """
    masked = _mask_strings(stripped)
    depth, open_at, last_span = 0, -1, None
    for index, char in enumerate(masked):
        if char == "(":
            if depth == 0:
                open_at = index
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                last_span = (open_at, index)
            elif depth < 0:
                return stripped, "", False
    if depth > 0 and open_at >= 0:
        return stripped[:open_at].rstrip(), stripped[open_at:], True
    if depth == 0 and last_span and last_span[1] == len(stripped) - 1:
        start = last_span[0]
        equals = masked.find("=")
        # A default value's parens open right after "="; metadata's do not.
        default_paren = (0 <= equals < start
                         and not stripped[equals + 1:start].strip())
        if not default_paren:
            return stripped[:start].rstrip(), stripped[start:], False
    return stripped, "", False


def _unquote(text):
    text = text.strip()
    if text.startswith('"""') and text.endswith('"""'):
        text = text[3:-3]
    elif text.startswith('"') and text.endswith('"'):
        text = text[1:-1]
    else:
        return text
    # Schema continuation lines carry source indentation (8 spaces for
    # attribute docs, 4 for class docs); it would otherwise litter pages.
    text = textwrap.dedent(text).strip()
    text = text.replace("\n        ", "\n").replace("\n    ", "\n")
    return text


def parse_schema(path):
    """Parse schema.usda classes into {name: {doc, inherits, attrs}}."""
    with open(path) as stream:
        lines = stream.readlines()
    classes = {}
    i, total = 0, len(lines)
    while i < total:
        match = _CLASS_RE.match(lines[i].strip())
        if not match:
            i += 1
            continue
        name = match.group(1) or match.group(2)
        entry = {"doc": "", "inherits": [], "attrs": []}
        i += 1
        # Class header: customData/inherits/doc until the closing ")".
        depth = 1
        header = []
        while i < total and depth > 0:
            depth += lines[i].count("(") - lines[i].count(")")
            header.append(lines[i])
            i += 1
        header_text = "".join(header)
        entry["inherits"] = _INHERITS_RE.findall(header_text)
        doc_match = re.search(r'doc\s*=\s*("""[\s\S]*?"""|"[^"\n]*")', header_text)
        if doc_match:
            entry["doc"] = _unquote(doc_match.group(1))
        # Body: "{" ... attributes ... "}".
        while i < total and lines[i].strip() != "{":
            i += 1
        i += 1
        while i < total:
            stripped = lines[i].strip()
            if stripped == "}":
                i += 1
                break
            if not stripped or stripped.startswith("#"):
                i += 1
                continue
            head, meta_text, meta_open = _split_attr_meta(stripped)
            attr_match = _ATTR_RE.match(head)
            if not attr_match:
                # Loud on purpose: a silently skipped declaration drops a
                # parameter from every page that documents this class.
                sys.stderr.write(
                    "build_pages: unparsed schema line %s:%d: %s\n"
                    % (os.path.basename(path), i + 1, stripped))
                i += 1
                continue
            uniform, kind, attr_name, default = attr_match.groups()
            attr = {"name": attr_name,
                    "type": ("uniform " if uniform else "") + kind,
                    "default": (default or "").strip(),
                    "tokens": [], "doc": ""}
            i += 1
            if meta_open:
                # Attribute metadata until the closing ")".
                code, in_doc = _mask_meta_line(meta_text, False)
                meta_depth = code.count("(") - code.count(")")
                meta_lines = [meta_text + "\n"]
                while i < total and meta_depth > 0:
                    # Doc strings may contain parens; strip them first.
                    code, in_doc = _mask_meta_line(lines[i], in_doc)
                    meta_depth += code.count("(") - code.count(")")
                    meta_lines.append(lines[i])
                    i += 1
                meta_text = "".join(meta_lines)
            if meta_text:
                tokens_match = _TOKENS_RE.search(meta_text)
                if tokens_match:
                    attr["tokens"] = re.findall(r'"([^"]+)"', tokens_match.group(1))
                doc_match = re.search(
                    r'doc\s*=\s*("""[\s\S]*?"""|"[^"\n]*")', meta_text)
                if doc_match:
                    attr["doc"] = _unquote(doc_match.group(1))
            entry["attrs"].append(attr)
        classes[name] = entry
    return classes


def _attr_markdown(attr):
    chunks = ["#### `%s`" % attr["name"]]
    if attr["type"] == "rel":
        chunks.append("")
        chunks.append("*Relationship.*")
    else:
        chunks.append("")
        if attr["default"]:
            chunks.append("*Type:* `%s`. *Default:* `%s`."
                          % (attr["type"], attr["default"]))
        else:
            chunks.append("*Type:* `%s`." % attr["type"])
    if attr["tokens"]:
        chunks.append("")
        chunks.append("Valid values: %s."
                      % ", ".join("`%s`" % token for token in attr["tokens"]))
    if attr["doc"]:
        chunks.append("")
        chunks.append(attr["doc"].strip())
    chunks.append("")
    return "\n".join(chunks)


def _params_section(title, attrs):
    chunks = ["### %s" % title, ""]
    for attr in attrs:
        chunks.append(_attr_markdown(attr))
    return "\n".join(chunks)


def render_page(key, note, classes):
    schema = classes[note["schema"]]
    stage = example_stage(key, note)
    lines = []
    lines.append("# ![%s](../../icons/%s.png) %s" % (note["title"], key, note["title"]))
    lines.append("")
    lines.append("*%s*" % note["summary"])
    lines.append("")
    lines.append("| | |")
    lines.append("|---|---|")
    lines.append("| **Node type** | `%s` |" % note["schema"])
    if stage:
        lines.append("| **Example** | [%s.usda](../examples/%s.usda) |"
                     % (stage, stage))
    lines.append("")
    lines.append("On this page:")
    lines.append("")
    lines.append("- [Overview](#overview)")
    lines.append("- [How it works](#how-it-works)")
    lines.append("- [Wiring](#wiring)")
    lines.append("- [Parameters](#parameters)")
    lines.append("- [Example](#example)")
    lines.append("- [Tips](#tips)")
    lines.append("- [See also](#see-also)")
    lines.append("")
    lines.append("## Overview")
    lines.append("")
    if not note.get("no_gif"):
        lines.append("![%s effect](../gifs/%s.gif)" % (note["title"], key))
        lines.append("")
    lines.append(note["description"].strip())
    lines.append("")
    if schema["doc"]:
        lines.append(schema["doc"].strip())
        lines.append("")
    lines.append("## How it works")
    lines.append("")
    lines.append(note["how_it_works"].strip())
    lines.append("")
    lines.append("## Wiring")
    lines.append("")
    lines.append("| Relationship | Points to | Required |")
    lines.append("|---|---|---|")
    for rel, target, required in note["wiring"]:
        lines.append("| %s | %s | %s |" % (rel, target, required))
    lines.append("")
    lines.append("## Parameters")
    lines.append("")
    for group_title, group_schema in note.get("param_groups", []):
        lines.append(_params_section(group_title, classes[group_schema]["attrs"]))
    lines.append(_params_section("Node parameters", schema["attrs"]))
    lines.append("## Example")
    lines.append("")
    lines.append(note["example"].strip())
    lines.append("")
    if stage:
        lines.append("Open it live with:")
        lines.append("")
        lines.append("```bat")
        lines.append("bin\\launch_usdview.bat docs\\examples\\%s.usda" % stage)
        lines.append("```")
        lines.append("")
    if not note.get("no_gif"):
        lines.append("Re-render the GIF above with:")
        lines.append("")
        lines.append("```bat")
        lines.append("python docs/render_media.py --page %s" % key)
        lines.append("```")
        lines.append("")
    lines.append("## Tips")
    lines.append("")
    for tip in note["tips"]:
        lines.append("- %s" % tip)
    lines.append("")
    lines.append("## See also")
    lines.append("")
    for target, label in note["see_also"]:
        lines.append("- [%s](%s.md)" % (label, target))
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("[UsdRig](../index.md)")
    lines.append("")
    return "\n".join(lines)


def render_index(concepts=()):
    lines = []
    lines.append("# UsdRig")
    lines.append("")
    lines.append("One page per operator: what it does, how to wire it, every")
    lines.append("parameter, and a minimal animated example. Each example stage")
    lines.append("lives in [examples](examples/) and plays in `usdview` via")
    lines.append("`bin\\launch_usdview.bat`; the GIF on each page is rendered")
    lines.append("live from that stage by `docs/render_media.py`, an offscreen")
    lines.append("Storm viewport with the rig guides on.")
    lines.append("")
    if concepts:
        lines.append("## Concepts")
        lines.append("")
        lines.append("| | Page | About |")
        lines.append("|---|---|---|")
        for page in concepts:
            lines.append("| ![%s](../icons/concept.png) | [%s](concepts/%s.md)"
                         " | %s |" % (page["title"], page["title"],
                                      page["slug"], page["summary"]))
        lines.append("")
    for category, keys in CATEGORIES:
        lines.append("## %s" % category)
        lines.append("")
        lines.append("| | Node | Does |")
        lines.append("|---|---|---|")
        for key in keys:
            note = OPERATORS[key]
            lines.append("| ![%s](../icons/%s.png) | [%s](nodes/%s.md) | %s |"
                         % (note["title"], key, note["title"], key, note["summary"]))
        lines.append("")
    lines.append("Implementation and design notes live in [specs](specs/).")
    lines.append("")
    return "\n".join(lines)


def main():
    classes = parse_schema(SCHEMA)
    # Every class a page reaches for, node schema and param groups alike:
    # an unchecked param group would surface as a bare KeyError deep in
    # render_page instead of naming the renamed class here.
    wanted = set()
    for note in OPERATORS.values():
        wanted.add(note["schema"])
        wanted.update(group_schema
                      for _title, group_schema in note.get("param_groups", []))
    missing = sorted(name for name in wanted if name not in classes)
    if missing:
        raise SystemExit("schema classes not found: %s" % ", ".join(missing))
    check_examples()
    os.makedirs(NODES_DIR, exist_ok=True)
    for key, note in OPERATORS.items():
        page = render_page(key, note, classes)
        with open(os.path.join(NODES_DIR, key + ".md"), "w") as stream:
            stream.write(page)
        print("wrote nodes/%s.md" % key)
    concepts = load_concepts()
    for page in concepts:
        print("concept concepts/%s.md (order %d)"
              % (page["slug"], page["order"]))
    with open(os.path.join(RIG, "docs", "index.md"), "w") as stream:
        stream.write(render_index(concepts))
    print("wrote index.md")


if __name__ == "__main__":
    main()
