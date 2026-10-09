"""Package the sibling Godot tutorial for docs; omit caches and stale binaries."""
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT.parent / "godot_rigExec"
OUT = ROOT / "docs/examples/godot_rolling_ball.zip"

START_HERE = """Godot rolling-ball tutorial
Windows x86-64 binaries, Godot 4.7+. Open demo/project.godot and press F5.
The prebuilt game does not need USD. Walkthrough:
docs/concepts/tutorial-godot-baked-rig.md in the usdRig checkout.

The shipped rolling_ball.rigexec still embeds the older textured presentation.
The checkout's tutorial_rolling_ball.usda is a displayColor stripe with no
UVs or preview-surface material, so tools/export_ball_assets.py fails its
material asserts against that stage. Playing the prebuilt file does not
rebuild it.

For a native addon build, extract godot_rigExec beside a directory named
usdRig and beside usd-install. Close the Godot editor first. Fetch godot-cpp:
git clone https://github.com/godotengine/godot-cpp.git thirdparty/godot-cpp
git -C thirdparty/godot-cpp checkout 507ed9d840c01a3c5b2a39af8bb4000bfac30bf5
git -C thirdparty/godot-cpp submodule update --init --recursive
python demo/setup_rolling.py --build

No Godot cache, editor state, credentials, or platform binaries other than
the tested Windows x86-64 pair are included. Source is included; other
platforms need their own native builds and exporter toolchain setup.
"""

DEMO_README = """# Roll / Collect — one self-contained rigExec asset

Open project.godot in Godot 4.7+ and press F5. WASD or the arrows move,
Space hops, Shift brakes, and R resets. Collect six rings and reach the exit.

rolling_ball.tscn owns a collision body and RigExecPlayer. The player loads
rolling_ball.rigexec. That file embeds the execution program and a
presentation section (deformed-point bindings, subdivision stencils, UVs,
material, PNG, and public controller names). No Godot skeleton or loose
mesh, material, or texture files are required to play.

rolling_ball.gd drives Move.tx, Move.ty, Move.tz, Roll.rx, Roll.ry, and
Roll.rz through set_control(). evaluate() runs the rig and updates the
render mesh. reset_controls() restores the baseline. get_controls() lists
the public names and does not expose USD paths.

The free-variant wrapper in the usdRig checkout declares
rigExec:exposedAvars and rigExec:publicName on Move and Roll. The legacy
skeleton adapter remains available for older program-only files; this
example does not use it.

The file in this archive was baked while the ball still had a texture.
Its embedded material uses diffuse scale 0.75, emission scale 0.5,
roughness 0.5, metallic 0, specular 0.35, repeat on S, and clamp on T.
The stage now checked in at docs/examples/tutorial_rolling_ball.usda is a
yellow sphere with a black equatorial displayColor stripe. It has no UVs
and no UsdPreviewSurface, so export_ball_assets.py rejects it. Playing
this project uses the baked file already here.

setup_rolling.py looks for a sibling directory named usdRig and for
usd-install. Close the Godot editor before running it. The bake command
it runs does not pass --poseable:

    rigExecBake docs/examples/tutorial_rolling_ball_free.usda --frames 1001 -o demo/rolling_ball.rigexec

Walkthrough: docs/concepts/tutorial-godot-baked-rig.md in the usdRig checkout.
Format: docs/specs/rigexec-presentation.md
"""


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
        "setup_rolling.py", "play_rolling.bat")]
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
        archive.writestr("godot_rigExec/START_HERE.txt", START_HERE)
        archive.writestr("godot_rigExec/demo/README.md", DEMO_README)
    print(f"Wrote {OUT} ({OUT.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
