"""Package the editable rolling-ball USD with every referenced icon."""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import hashlib
import json
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
files = {DOCS / "examples/tutorial_rolling_ball.usda",
         DOCS / "examples/tutorial_rolling_ball_free.usda"}
for stage in list(files):
    for relative in re.findall(r"@([^@]+)@", stage.read_text()):
        dependency = (stage.parent / relative).resolve()
        assert dependency.is_relative_to(ROOT) and dependency.is_file(), dependency
        files.add(dependency)
manifest = {}
archive_path = DOCS / "examples/usdview_rolling_ball.zip"
with ZipFile(archive_path, "w", ZIP_DEFLATED, compresslevel=9) as archive:
    archive.write(ROOT / "LICENSE", "LICENSE-usdRig")
    archive.write(ROOT / "NOTICE", "NOTICE-usdRig")
    archive.write(ROOT / "plugin/usdNoodles/LICENSE.txt", "LICENSE-OpenUSD")
    archive.write(ROOT / "THIRD_PARTY_NOTICES.md", "THIRD_PARTY_NOTICES-usdRig.md")
    for path in sorted(files):
        name = path.relative_to(ROOT).as_posix()
        archive.write(path, name)
        manifest[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    archive.writestr("manifest.json", json.dumps(manifest, indent=2)+"\n")
    archive.writestr("README.txt", """USD rolling ball: yellow sphere with a black equatorial stripe

Extract the entire archive. Keep docs/ and icons/ beside one another.
Open docs/examples/tutorial_rolling_ball.usda in a RigExec-enabled usdview.
Scrub 1001-1049 for travel-driven rolling.
Open docs/examples/tutorial_rolling_ball_free.usda for the Godot bake source.

The ball carries no texture. Its stripe is per-point displayColor on this
stage and on the free variant that references it. The Godot archive paints
its presentation PNG from that same stripe.
The stage's rig, controllers and colors remain editable.

Live rig evaluation requires the RigExec schema and imaging plugins.
""")
for path in files:
    relative = path.relative_to(ROOT)
    if str(relative).startswith("docs"):
        target = DOCS / "site" / path.relative_to(DOCS)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
shutil.copy2(archive_path, DOCS / "site/examples" / archive_path.name)
print(f"Packaged {len(files)} files: {archive_path}")
