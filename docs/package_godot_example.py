"""Package the sibling Godot tutorial for docs; omit caches and stale binaries."""
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile
import hashlib
import json
import argparse

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT.parent / "godot_rigExec"
OUT = ROOT / "docs/examples/godot_rolling_ball.zip"


def _tree(folder):
    """Every file under a checked-in plugin folder, without Python caches."""
    return [path for path in (PLUGIN / folder).rglob("*")
            if path.is_file() and "__pycache__" not in path.parts
            and path.suffix != ".pyc"]


def main():
    global PLUGIN
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--addon-root", type=Path, default=PLUGIN,
                        help="Godot addon checkout (default: sibling godot_rigExec)")
    PLUGIN = parser.parse_args().addon_root.resolve()
    files = [PLUGIN / "SConstruct"]
    files += list((PLUGIN / "addons/rigexec").glob("*.gd"))
    files += list((PLUGIN / "addons/rigexec").glob("*.cfg"))
    files += list((PLUGIN / "addons/rigexec").glob("*.gdextension"))
    files += [PLUGIN / "addons/rigexec" / name for name in (
        "README.md", "THIRD_PARTY_NOTICES.md", "LICENSE-OpenUSD.txt",
        "LICENSE-flatbuffers.txt", "LICENSE-godot-cpp.md",
        "LICENSE-LZMA-SDK.txt", "NOTICE-LZMA-SDK.md")]
    files += list((PLUGIN / "addons/rigexec/src").glob("*.cpp"))
    files += list((PLUGIN / "addons/rigexec/src").glob("*.h"))
    # Windows only: bin/ can also hold other platforms' untested libraries.
    files += list((PLUGIN / "addons/rigexec/bin").glob("*.windows.*.dll"))
    files += [PLUGIN / "tools" / name for name in (
        "export_ball_assets.py", "gen_presentation.py", "ball_subdivide.cpp",
        "build_ball_subdivide.bat", "retarget_ball_star_uvs.py")]
    # The exporter's generated presentation builder and the vendored
    # FlatBuffers Python runtime it runs on, with that runtime's LICENSE.
    files += _tree("tools/generated") + _tree("tools/thirdparty")
    files += [PLUGIN / "demo" / name for name in (
        "project.godot", "rolling_ball.tscn", "rolling_ball.gd",
        "rolling_motion.gd", "rolling_game.tscn", "rolling_game.gd",
        "rolling_ball.rigexec", "verify_rolling.gd", "record_rolling.gd", "record_ball_material.gd",
        "setup_rolling.py", "play_rolling.bat", "README.md")]
    # (source, path under godot_rigExec/). The repository LICENSE also
    # ships as the add-on's own, as the add-on package does.
    members = [(path, path.relative_to(PLUGIN).as_posix()) for path in sorted(set(files))]
    members += [(PLUGIN / "LICENSE", "LICENSE"),
                (PLUGIN / "LICENSE", "addons/rigexec/LICENSE")]
    licenses = (
        ("LICENSE-usdRig", ROOT / "LICENSE"),
        ("NOTICE-usdRig", ROOT / "NOTICE"),
        ("LICENSE-OpenUSD", ROOT / "plugin/usdNoodles/LICENSE.txt"),
        ("LICENSE-flatbuffers", ROOT / "thirdparty/flatbuffers/LICENSE"),
        ("THIRD_PARTY_NOTICES-usdRig.md", ROOT / "THIRD_PARTY_NOTICES.md"),
        ("LICENSE-LZMA-SDK", ROOT / "third_party/lzma/DOC/lzma-sdk.txt"),
        ("NOTICE-LZMA-SDK.md", ROOT / "third_party/lzma/NOTICE.md"),
        ("LICENSE-godot-cpp", PLUGIN / "thirdparty/godot-cpp/LICENSE.md"),
    )
    missing = [str(path) for path, _ in members if not path.is_file()]
    missing += [str(path) for _, path in licenses if not path.is_file()]
    if missing:
        raise SystemExit("missing: " + ", ".join(missing))
    manifest = {}
    with ZipFile(OUT, "w", ZIP_DEFLATED, compresslevel=9) as archive:
        for path, relative in members:
            archive.write(path, "godot_rigExec/" + relative)
            manifest[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
            if relative.startswith("addons/") and "/src/" not in relative:
                archive.write(path, "godot_rigExec/demo/" + relative)
        for name, path in licenses:
            archive.write(path, "godot_rigExec/" + name)
        archive.writestr("godot_rigExec/package_manifest.json",
                         json.dumps(manifest, indent=2) + "\n")
        archive.writestr("godot_rigExec/START_HERE.txt", """Godot rolling-ball tutorial
Windows x86-64 binaries, Godot 4.7+. Open demo/project.godot and press F5.
The prebuilt game does not need USD.

For rebuilds, extract godot_rigExec beside usdRig and usd-install.
Setup exports the presentation (numpy must be installed in the Python with
the USD bindings), then bakes it into demo/rolling_ball.rigexec.
Close the Godot editor before running python demo/setup_rolling.py.
For native builds, first fetch godot-cpp into thirdparty/godot-cpp:
git clone https://github.com/godotengine/godot-cpp.git thirdparty/godot-cpp
git -C thirdparty/godot-cpp checkout 507ed9d840c01a3c5b2a39af8bb4000bfac30bf5
git -C thirdparty/godot-cpp submodule update --init --recursive
python demo/setup_rolling.py --build

No Godot cache, editor state, credentials, or platform binaries other than
the tested Windows x86-64 pair are included. Source is included; other
platforms need their own native builds and exporter toolchain setup.

Licenses: LICENSE (godot_rigExec, MIT); the LICENSE-*, NOTICE-usdRig and
THIRD_PARTY_NOTICES-usdRig.md files here; addons/rigexec/THIRD_PARTY_NOTICES.md
and its license texts; tools/thirdparty/flatbuffers/LICENSE for the vendored
FlatBuffers Python runtime.
""")
    print(f"Wrote {OUT} ({OUT.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
