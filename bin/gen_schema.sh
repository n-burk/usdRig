#!/bin/bash
# bin/gen_schema.sh -- generate the codeless RigExec schema plugin from
# schema.usda. The POSIX twin of gen_schema.bat.
#
# Writes into plugin/rigExecSchema/resources, which is CHECKED IN: run it only
# after editing libs/rigExecSchema/schema.usda, and review the diff.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$USD/bin/usdGenSchema"

cd "$RIG/libs/rigExecSchema"
"$PY" "$USD/bin/usdGenSchema" schema.usda ../../plugin/rigExecSchema/resources

# Substitute the build-system placeholders for a codeless resource plugin. The
# generated plugInfo names a LibraryPath that a codeless schema has no library
# for, and Plug refuses to load the plugin while it is present.
"$PY" - <<'PY'
import io
p = '../../plugin/rigExecSchema/resources/plugInfo.json'
s = io.open(p, encoding='utf-8').read()
s = s.replace('"LibraryPath": "@PLUG_INFO_LIBRARY_PATH@", ', '')
s = s.replace('"@PLUG_INFO_RESOURCE_PATH@"', '"."')
s = s.replace('"@PLUG_INFO_ROOT@"', '"."')
io.open(p, 'w', encoding='utf-8').write(s)
PY
echo "regenerated plugin/rigExecSchema/resources -- review the diff before committing"
