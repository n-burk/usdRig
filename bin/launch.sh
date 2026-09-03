#!/bin/bash
# bin/launch.sh — launch usdview with Muse Assistant on a blank stage (or given stage)
# Usage: bin/launch.sh [stage.usda] [--renderer Gl|Embree]
#   no args → blank stage /tmp/blank.usda (World Xform)
# Requires: usd-install at $USD, rig build at $RIG/build, venv at $VENV
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
. "$SCRIPT_DIR/_env.sh"

if [ ! -x "$PY" ]; then
  echo "ERROR: venv python not found at $PY (set VENV=...)" >&2
  exit 1
fi
if [ ! -f "$USDVIEW" ]; then
  echo "ERROR: usdview not found at $USD/bin/usdview" >&2
  exit 1
fi

# Build if needed (no-op if up to date)
if [ -f "$RIG/build/CMakeCache.txt" ]; then
  cmake --build "$RIG/build" >/dev/null 2>&1 || true
fi

# Help
if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  echo "Usage: bin/launch.sh [stage.usda] [--renderer Storm|GL|Embree]"
  echo "  no args      → examples/ArmShotAnim.usda (rig with Storm-visible controls; blank has no rig)"
  echo "  --blank      → blank stage /tmp/blank.usda (World Xform, no rig, no controls to draw)"
  echo "  <file.usda>  → that stage"
  echo "  --help/-h    → this help"
  exit 0
fi

# Blank stage helper
BLANK="/tmp/blank.usda"
_create_blank() {
  cat > "$BLANK" <<'USD'
#usda 1.0
def Xform "World" {
}
USD
}
if [ "$#" -eq 0 ]; then
  # Default to the rigged shot so Storm controls are visible immediately (matches launch_usdview.bat)
  if [ -f "$RIG/examples/ArmShotAnim.usda" ]; then
    STAGE="$RIG/examples/ArmShotAnim.usda"
  else
    _create_blank
    STAGE="$BLANK"
  fi
  EXTRA=()
elif [ "${1:-}" = "--blank" ]; then
  _create_blank
  STAGE="$BLANK"
  shift
  EXTRA=("$@")
elif [ "${1#--}" != "$1" ]; then
  # No stage given, just renderer/options – use default stage so controls are visible
  if [ -f "$RIG/examples/ArmShotAnim.usda" ]; then
    STAGE="$RIG/examples/ArmShotAnim.usda"
  else
    _create_blank
    STAGE="$BLANK"
  fi
  EXTRA=("$@")
else
  STAGE="$1"
  shift
  EXTRA=("$@")
fi

export PATH="$RIG/build:$USD/lib:$PATH"
export DYLD_LIBRARY_PATH="$USD/lib:$RIG/build${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export PYTHONPATH="$RIG/plugin/rigExecUsdview:$RIG/plugin/museAssistant:$USD/lib/python3.11/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export PXR_PLUGINPATH_NAME="$RIG/build/usd/rigExecSchema/resources:$RIG/build/usd/rigExecImaging/resources:$RIG/plugin/rigExecUsdview:$RIG/plugin/museAssistant${PXR_PLUGINPATH_NAME:+:$PXR_PLUGINPATH_NAME}"

echo "USD: $USD"
echo "RIG: $RIG"
echo "VENV: $VENV ($($PY --version))"
echo "Stage: $STAGE"
echo "Muse Assistant: $RIG/plugin/museAssistant (tool loop + drawover + /goal)"
echo "Launch: $PY $USDVIEW ${STAGE} ${EXTRA[*]:-}"
echo ""

# Report the selected back end before opening the window. Local providers need
# no user credential. LM Studio and Ollama still use the Anthropic SDK; Apple
# has its own HTTP adapter.
_MUSE_PROVIDER="${MUSE_PROVIDER:-}"
_MUSE_SETTINGS_FILE="$HOME/.config/muse/credentials.json"
_muse_saved_setting() {
  "$PY" -c 'import json,sys; p,k=sys.argv[1:3];
try: v=json.load(open(p)).get(k, "")
except (OSError, ValueError): v=""
print(v if isinstance(v, str) else "")' "$1" "$2" 2>/dev/null
}
if [ -z "$_MUSE_PROVIDER" ] && [ -r "$_MUSE_SETTINGS_FILE" ]; then
  _MUSE_PROVIDER="$(_muse_saved_setting "$_MUSE_SETTINGS_FILE" MUSE_PROVIDER)"
fi
_MUSE_PROVIDER="$(printf '%s' "$_MUSE_PROVIDER" | tr '[:upper:]' '[:lower:]')"
_MUSE_KEY="${MUSE_API_KEY:-${ANTHROPIC_API_KEY:-}}"
_MUSE_KEY_NAME="MUSE_API_KEY"
[ -z "${MUSE_API_KEY:-}" ] && [ -n "${ANTHROPIC_API_KEY:-}" ] && _MUSE_KEY_NAME="ANTHROPIC_API_KEY"
_MUSE_BASE="${MUSE_BASE_URL:-${ANTHROPIC_BASE_URL:-}}"

if [ "$_MUSE_PROVIDER" = "apple" ]; then
  _MUSE_APPLE_URL="${MUSE_APPLE_URL:-}"
  if [ -z "$_MUSE_APPLE_URL" ] && [ -r "$_MUSE_SETTINGS_FILE" ]; then
    _MUSE_APPLE_URL="$(_muse_saved_setting "$_MUSE_SETTINGS_FILE" MUSE_APPLE_URL)"
  fi
  _MUSE_APPLE_URL="${_MUSE_APPLE_URL:-http://127.0.0.1:1976}"
  _MUSE_APPLE_URL="${_MUSE_APPLE_URL%/}"
  echo "Muse provider: Apple Foundation Models"
  echo "Muse endpoint: $_MUSE_APPLE_URL/v1/chat/completions"
  echo "Muse model: system (on-device; PCC is never selected)"
  echo "Muse key/SDK: not needed"
  if MUSE_APPLE_PROBE_URL="$_MUSE_APPLE_URL" "$PY" -c \
      'import json, os, sys, urllib.request; o=urllib.request.build_opener(urllib.request.ProxyHandler({})); p=json.load(o.open(os.environ["MUSE_APPLE_PROBE_URL"]+"/health", timeout=4)); sys.exit(0 if any(m.get("name")=="system" and m.get("available") is True for m in p.get("models", [])) else 1)' \
      >/dev/null 2>&1; then
    echo "Muse server: ready"
  else
    echo "Muse server: NOT READY — start it in another Terminal:"
    echo "             fm serve --host 127.0.0.1 --port 1976"
  fi
elif [ "$_MUSE_PROVIDER" = "ollama" ]; then
  echo "Muse provider: Ollama"
  echo "Muse endpoint: ${MUSE_OLLAMA_URL:-http://127.0.0.1:11434}/v1/messages"
  echo "Muse model: ${MUSE_MODEL:-qwen3.5:9b}"
  echo "Muse key: not needed"
elif [ "$_MUSE_PROVIDER" = "lmstudio" ]; then
  _MUSE_LMSTUDIO_URL="${MUSE_LMSTUDIO_URL:-}"
  _MUSE_LMSTUDIO_URL_SOURCE="MUSE_LMSTUDIO_URL"
  if [ -z "$_MUSE_LMSTUDIO_URL" ] && [ -r "$_MUSE_SETTINGS_FILE" ]; then
    _MUSE_LMSTUDIO_URL="$(_muse_saved_setting "$_MUSE_SETTINGS_FILE" MUSE_LMSTUDIO_URL)"
    _MUSE_LMSTUDIO_URL_SOURCE="saved settings"
  fi
  if [ -z "$_MUSE_LMSTUDIO_URL" ]; then
    _MUSE_LMSTUDIO_URL="http://hivemind.local:1234"
    _MUSE_LMSTUDIO_URL_SOURCE="default"
  fi
  _MUSE_LMSTUDIO_URL="${_MUSE_LMSTUDIO_URL%/}"

  _MUSE_LMSTUDIO_MODEL="${MUSE_MODEL:-}"
  _MUSE_LMSTUDIO_MODEL_SOURCE="MUSE_MODEL"
  if [ -z "$_MUSE_LMSTUDIO_MODEL" ] && [ -r "$_MUSE_SETTINGS_FILE" ]; then
    _MUSE_LMSTUDIO_MODEL="$(_muse_saved_setting "$_MUSE_SETTINGS_FILE" MUSE_MODEL)"
    _MUSE_LMSTUDIO_MODEL_SOURCE="saved settings"
  fi

  echo "Muse provider: LM Studio on Hivemind"
  echo "Muse endpoint: $_MUSE_LMSTUDIO_URL/v1/messages ($_MUSE_LMSTUDIO_URL_SOURCE)"
  echo "Muse key: not needed — a local placeholder is used; hosted keys are not sent"
  if ! "$PY" -c "import anthropic" 2>/dev/null; then
    echo "Muse SDK: MISSING — run:  $PY -m pip install anthropic"
  else
    echo "Muse SDK: anthropic $("$PY" -c 'import anthropic;print(anthropic.__version__)')"
  fi

  # Probe the native inventory directly. Disabling proxy handling matters for
  # this LAN address: a configured HTTP proxy can make a healthy server look
  # unavailable. The second field mirrors Muse's server-selected model policy.
  if _MUSE_LMSTUDIO_PROBE="$(MUSE_LMSTUDIO_PROBE_URL="$_MUSE_LMSTUDIO_URL" "$PY" -c \
      'import json, os, urllib.request
root = os.environ["MUSE_LMSTUDIO_PROBE_URL"]
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
with opener.open(root + "/api/v1/models", timeout=4) as response:
    payload = json.load(response)
entries = payload.get("models", []) if isinstance(payload, dict) else []
models = []
for entry in entries:
    if not isinstance(entry, dict) or str(entry.get("type", "")).lower() in ("embedding", "embeddings"):
        continue
    name = entry.get("key") or entry.get("id")
    if not name:
        continue
    capabilities = entry.get("capabilities") or {}
    if isinstance(capabilities, dict):
        tools = bool(capabilities.get("trained_for_tool_use") or capabilities.get("tool_use") or capabilities.get("tools"))
        vision = bool(capabilities.get("vision"))
    else:
        names = set(capabilities)
        tools = bool(names.intersection(("tool_use", "tools")))
        vision = "vision" in names
    loaded = bool(entry.get("loaded_instances")) or str(entry.get("state", "")).lower() == "loaded"
    models.append((str(name), loaded, tools, vision))
preferred = max(models, key=lambda item: (4 if item[1] else 0, 2 if item[2] else 0, 1 if item[3] else 0))[0] if models else ""
print("%d\t%s" % (len(models), preferred))' 2>/dev/null)"; then
    IFS=$'\t' read -r _MUSE_LMSTUDIO_MODEL_COUNT _MUSE_LMSTUDIO_SERVER_MODEL <<< "$_MUSE_LMSTUDIO_PROBE"
    if [ -n "$_MUSE_LMSTUDIO_MODEL" ]; then
      echo "Muse model: $_MUSE_LMSTUDIO_MODEL ($_MUSE_LMSTUDIO_MODEL_SOURCE)"
    elif [ -n "$_MUSE_LMSTUDIO_SERVER_MODEL" ]; then
      echo "Muse model: $_MUSE_LMSTUDIO_SERVER_MODEL (selected from server)"
    else
      echo "Muse model: none — LM Studio listed no LLMs"
    fi
    echo "Muse server: ready — $_MUSE_LMSTUDIO_MODEL_COUNT LLM(s) listed"
  else
    if [ -n "$_MUSE_LMSTUDIO_MODEL" ]; then
      echo "Muse model: $_MUSE_LMSTUDIO_MODEL ($_MUSE_LMSTUDIO_MODEL_SOURCE)"
    else
      echo "Muse model: unavailable until the server lists one"
    fi
    echo "Muse server: NOT READY — on Hivemind, start LM Studio on port 1234"
    echo "             and enable Serve on Local Network in its server settings."
  fi
else
  if [ -z "$_MUSE_KEY" ]; then
    echo "Muse key: NOT SET — the assistant cannot talk to a hosted model."
    echo "          export MUSE_API_KEY=...   then relaunch."
  else
    echo "Muse key: $_MUSE_KEY_NAME (set, ${#_MUSE_KEY} chars)"
    if [ -n "$_MUSE_BASE" ]; then
      echo "Muse endpoint: $_MUSE_BASE (MUSE_BASE_URL)"
      echo "Muse model: ${MUSE_MODEL:-<endpoint default>}"
    elif [ -z "${_MUSE_KEY##LLM_*}" ]; then
      # A Meta Muse key. api.anthropic.com answers this shape with 401, so the
      # plugin routes it to Meta's Messages API automatically.
      echo "Muse endpoint: https://api.meta.ai (Meta Muse key detected)"
      echo "Muse model: ${MUSE_MODEL:-muse-spark-1.3-contributor}"
    else
      echo "Muse endpoint: https://api.anthropic.com"
      echo "Muse model: ${MUSE_MODEL:-claude-opus-5}"
      case "$_MUSE_KEY" in
        sk-ant-*) ;;
        *) echo "          WARNING: this does not look like an Anthropic key"
           echo "                   (sk-ant-…). Set MUSE_BASE_URL to the endpoint"
           echo "                   that issued it, or expect a 401." ;;
      esac
    fi
  fi
  if ! "$PY" -c "import anthropic" 2>/dev/null; then
    echo "Muse SDK: MISSING — run:  $PY -m pip install anthropic"
  else
    echo "Muse SDK: anthropic $("$PY" -c 'import anthropic;print(anthropic.__version__)')"
  fi
fi
echo ""
echo "In usdview: Muse → Open Muse Assistant"
echo "  Ask it to change the stage — it inspects, runs Python here, and"
echo "  captures the viewport to check its own work."
echo "  Muse → Save Screenshot with Camera…  writes a PNG you can attach later."
echo "Logs: /tmp/museAssistant_crash.log  (if the panel fails, paste it)"
echo ""

if [ ${#EXTRA[@]} -eq 0 ]; then
  exec "$PY" "$USDVIEW" "$STAGE"
else
  exec "$PY" "$USDVIEW" "$STAGE" "${EXTRA[@]}"
fi
