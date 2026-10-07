"""Package the sibling Godot tutorial for docs; omit caches and stale binaries."""
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT.parent / "godot_rigExec"
OUT = ROOT / "docs/examples/godot_rolling_ball.zip"


def main():
    files = [PLUGIN / "SConstruct"]
    files += list((PLUGIN / "addons/rigexec").glob("*.gd"))
    files += list((PLUGIN / "addons/rigexec").glob("*.cfg"))
    files += list((PLUGIN / "addons/rigexec").glob("*.gdextension"))
    files += list((PLUGIN / "addons/rigexec/src").glob("*.cpp"))
    files += list((PLUGIN / "addons/rigexec/src").glob("*.h"))
    files += list((PLUGIN / "addons/rigexec/bin").glob("*.windows.*.dll"))
    files += [PLUGIN / "tools" / name for name in (
        "export_ball_assets.py", "ball_subdivide.cpp", "build_ball_subdivide.bat",
        "retarget_ball_star_uvs.py")]
    files += [PLUGIN / "demo" / name for name in (
        "project.godot", "rolling_ball.tscn", "rolling_ball.gd",
        "rolling_motion.gd", "rolling_game.tscn", "rolling_game.gd",
        "rolling_ball.rigexec", "verify_rolling.gd", "record_rolling.gd", "record_ball_material.gd",
        "setup_rolling.py", "play_rolling.bat", "README.md")]
    manifest = {}
    with ZipFile(OUT, "w", ZIP_DEFLATED, compresslevel=9) as archive:
        for path in sorted(set(files)):
            relative = path.relative_to(PLUGIN).as_posix()
            archive.write(path, "godot_rigExec/" + relative)
            manifest[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
            if relative.startswith("addons/") and "/src/" not in relative:
                archive.write(path, "godot_rigExec/demo/" + relative)
        for name, path in (
            ("LICENSE-usdRig", ROOT / "LICENSE"),
            ("NOTICE-usdRig", ROOT / "NOTICE"),
            ("LICENSE-OpenUSD", ROOT / "plugin/usdNoodles/LICENSE.txt"),
            ("THIRD_PARTY_NOTICES-usdRig.md", ROOT / "THIRD_PARTY_NOTICES.md"),
            ("LICENSE-godot-cpp", PLUGIN / "thirdparty/godot-cpp/LICENSE.md"),
        ):
            if path.exists():
                archive.write(path, "godot_rigExec/" + name)
        archive.writestr("godot_rigExec/package_manifest.json",
                         json.dumps(manifest, indent=2) + "\n")
        archive.writestr("godot_rigExec/START_HERE.txt", """Godot rolling-ball tutorial
Windows x86-64 binaries, Godot 4.7+. Open demo/project.godot and press F5.
The prebuilt game does not need USD.

For rebuilds, extract godot_rigExec beside usdRig and usd-install.
Close the Godot editor before running python demo/setup_rolling.py.
For native builds, first fetch godot-cpp into thirdparty/godot-cpp:
git clone https://github.com/godotengine/godot-cpp.git thirdparty/godot-cpp
git -C thirdparty/godot-cpp checkout 507ed9d840c01a3c5b2a39af8bb4000bfac30bf5
git -C thirdparty/godot-cpp submodule update --init --recursive
python demo/setup_rolling.py --build

No Godot cache, editor state, credentials, or platform binaries other than
the tested Windows x86-64 pair are included. Source is included; other
platforms need their own native builds and exporter toolchain setup.
""")
    print(f"Wrote {OUT} ({OUT.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
