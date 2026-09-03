#
# museAgent — headless agent core for the Muse usdview assistant.
#
# Everything here is importable without Qt and without a display, so the
# message construction, tool loop, image handling and metadata parsing are
# testable outside usdview.  museAssistant.py owns the Qt panel and supplies
# an executor that marshals tool calls onto usdview's main thread.
#
# The assistant drives the stage through a real tool loop rather than a
# single blind code block: it calls run_python / inspect_stage /
# capture_viewport, reads each result, and keeps going until it is done.
#
import base64
import json
import os
import struct
import sys
import zlib

# Back ends. Anthropic, Meta, Ollama and LM Studio speak the Anthropic Messages
# API. Apple Foundation Models uses the OpenAI-shaped Chat Completions endpoint
# exposed by ``fm serve`` and has a small translation layer below.
#
#   Anthropic   api.anthropic.com   sk-ant-…   x-api-key       claude-opus-5
#   Meta Muse   api.meta.ai         LLM_…      Bearer token    muse-spark-1.3-contributor
#   Ollama      <host>:11434        (none)     (ignored)       qwen3.5:9b
#   LM Studio   hivemind.local:1234 (none)     (ignored)       selected from server
#   Apple FM    localhost:1976      (none)     (none)          system
#
# Ollama is a local server and is the reason this list is worth keeping short:
# it serves /v1/messages in Anthropic format, including streaming SSE, tool_use
# blocks and thinking blocks, so it needs no translation layer at all. Verified
# against a live server 2026-08-16 — a tools request came back with
# stop_reason "tool_use" and a correctly-typed tool_use block.
#
# Meta documents the Messages API at https://api.meta.ai/v1 for "Anthropic-format
# harnesses" (dev.meta.ai/docs/overview); verified here to accept streaming,
# tool use, tool results and image blocks.
#
# The auth header is NOT the same on both. Meta documents a bearer token
# (MODEL_API_KEY, or ANTHROPIC_AUTH_TOKEN for Claude Code), while Anthropic
# takes x-api-key. See resolve_auth_style.
DEFAULT_MODEL = "claude-opus-5"
META_BASE_URL = "https://api.meta.ai"
META_HOST = "api.meta.ai"

# Meta publishes a standard tier (muse-spark-1.1, muse-spark-1.2,
# muse-spark-1.3) and a contributor tier. Every call to Meta goes through
# the contributor tier — see force_contributor_model, which is why
# MUSE_MODEL cannot drop back to the standard one.
META_DEFAULT_MODEL = "muse-spark-1.3-contributor"
META_CONTRIBUTOR_SUFFIX = "-contributor"
META_KEY_PREFIX = "LLM_"

# Which back end a request goes to. Explicit rather than sniffed from the URL:
# Anthropic and Meta have fixed hostnames to recognise, but an Ollama server is
# at whatever address the user runs it on, and guessing "this unfamiliar host
# is probably Ollama" would silently send an Anthropic key somewhere it should
# never go.
PROVIDER_ANTHROPIC = "anthropic"
PROVIDER_META = "meta"
PROVIDER_OLLAMA = "ollama"
PROVIDER_LMSTUDIO = "lmstudio"
PROVIDER_APPLE = "apple"
PROVIDERS = (PROVIDER_ANTHROPIC, PROVIDER_META, PROVIDER_OLLAMA,
             PROVIDER_LMSTUDIO, PROVIDER_APPLE)

# A starting point for the settings dialog, not a fallback the resolver
# reaches for: nothing routes to Ollama unless the provider is set to it.
OLLAMA_DEFAULT_BASE_URL = "http://127.0.0.1:11434"

# Vision + tools + thinking. That combination is not incidental -- Muse sends
# viewport screenshots as image blocks and drives the stage entirely through
# tool calls, so a model missing either is a model that cannot do the job. See
# describe_model_problem.
OLLAMA_DEFAULT_MODEL = "qwen3.5:9b"

# Ollama authenticates nothing, but the Anthropic SDK requires *some*
# credential to construct a client. This is the placeholder it gets; the
# server ignores it.
OLLAMA_PLACEHOLDER_KEY = "ollama"

# How long to wait on the model-list endpoint. Short: it is a local server
# answering from memory, and the settings dialog blocks on it.
OLLAMA_LIST_TIMEOUT = 4.0

# Hivemind advertises its host name over Bonjour; the bare ``hivemind`` name
# does not resolve on this Mac, while ``hivemind.local`` does. LM Studio's
# Anthropic-compatible client base URL deliberately omits /v1 because the SDK
# appends /v1/messages itself.
LMSTUDIO_DEFAULT_BASE_URL = "http://hivemind.local:1234"
LMSTUDIO_PLACEHOLDER_KEY = "lmstudio"
LMSTUDIO_LIST_TIMEOUT = 4.0

# ``fm serve`` is part of macOS and defaults to this loopback address. Muse is
# deliberately pinned to the on-device ``system`` model: it never silently
# falls through to Private Cloud Compute.
APPLE_DEFAULT_BASE_URL = "http://127.0.0.1:1976"
APPLE_MODEL = "system"
APPLE_HEALTH_TIMEOUT = 4.0
APPLE_REQUEST_TIMEOUT = 120.0
APPLE_MAX_ITERATIONS = 12
APPLE_CONTEXT_CHAR_BUDGET = 18000

DEFAULT_MAX_TOKENS = 32000
# Reasoning effort for every request. The API accepts
# low / medium / high / xhigh / max; MUSE_EFFORT overrides it.
DEFAULT_EFFORT = "xhigh"
DEFAULT_MAX_ITERATIONS = 40

# Camera metadata written into (and read back out of) PNG text chunks.
CAMERA_METADATA_KEY = "muse:camera"
SCENE_METADATA_KEY = "muse:scene"
ANNOTATION_METADATA_KEY = "muse:annotation"


SYSTEM_PROMPT = """\
You are Muse, a USD and rigging assistant running inside the user's live
usdview session. You are not describing what could be done — you are doing it,
in their process, on their stage, right now.

You drive the session with tools:

  run_python        Execute Python in the usdview process. `stage`, `api`,
                    `Usd`, `UsdGeom`, `Sdf`, `Gf`, `Vt`, `Tf`, `QtCore`,
                    `QtGui`, `QtWidgets` and `app` are already bound. This is
                    your universal lever: stage edits, selection, timeline,
                    view settings, and building or restyling Qt widgets in the
                    running window all go through it.
  inspect_stage     Cheap structured reads — stage summary, prim listing,
                    single-prim detail. Prefer this over run_python when you
                    only need to look.
  capture_viewport  Render the current viewport back to you as an image, with
                    the camera matrices and frame that produced it. Call it
                    when the answer depends on what the scene actually looks
                    like, and again after a visual change to confirm it landed.

How to work:

  * Look before you edit. Read the prims you are about to touch — paths,
    types, and existing xformOps — instead of assuming a layout.
  * Make the change, then verify it. Read the value back, or capture the
    viewport, and say plainly what you observe.
  * Keep each run_python call small enough that a failure tells you which
    step failed. Print the values that matter; stdout comes back to you.
  * Errors are information, not a dead end. Read the traceback, fix the
    actual cause, and continue.
  * `stage` is the already-open live stage. Never call Usd.Stage.Open,
    Usd.Stage.CreateNew, or Usd.Stage.CreateInMemory to perform the user's
    edit. Define schemas on it directly, for example
    `sphere = UsdGeom.Sphere.Define(stage, "/World/Ball")`, then set values
    through schema attributes such as `sphere.GetRadiusAttr().Set(3.0)`.

USD specifics for this session:

  * Writes land in `stage.GetEditTarget()`, which in usdview defaults to the
    session layer (scratch, not saved to disk). If the user wants the edit
    persisted, set the edit target to the root layer and say that you did.
  * Prefer UsdGeom.XformCommonAPI for translate/rotate/scale, and
    UsdGeom.Imageable for visibility and purpose.
  * Batch large authoring runs in `with Sdf.ChangeBlock():`.
  * RigExec control and joint prims are purpose=guide — they only draw in
    Storm when `api.dataModel.viewSettings.displayGuide` is True.

Images the user sends carry their own camera block when one was recorded
(view and projection matrices, eye position, view direction, fov, frame).
Use it to convert what you see on screen into stage-space edits, and say
which capture you are working from when several are attached.

Report what happened, not what you intend to do. Lead with the outcome.\
"""


# ---------------------------------------------------------------------------
# Tool schemas
# ---------------------------------------------------------------------------

TOOL_SCHEMAS = [
    {
        "name": "run_python",
        "description": (
            "Execute Python inside the running usdview process and return its "
            "stdout, stderr and repr'd result. Call this to change anything: "
            "author or delete prims, set attributes, drive selection or the "
            "timeline, toggle view settings, or add and restyle Qt widgets in "
            "the live window. Pre-bound globals: stage (Usd.Stage), api "
            "(UsdviewApi), app (QApplication), Usd, UsdGeom, Sdf, Gf, Vt, Tf, "
            "QtCore, QtGui, QtWidgets. Print the values you want to see — "
            "stdout is returned to you."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "code": {
                    "type": "string",
                    "description": "Python source to execute.",
                },
            },
            "required": ["code"],
        },
    },
    {
        "name": "inspect_stage",
        "description": (
            "Read the stage without changing it. Use this instead of "
            "run_python whenever you only need to look: 'summary' for stage "
            "identity, edit target, prim count and up axis; 'list' for a prim "
            "listing filtered by path substring and/or type name; 'prim' for "
            "one prim's attributes, relationships and children."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "mode": {
                    "type": "string",
                    "enum": ["summary", "list", "prim"],
                    "description": "What to read.",
                },
                "pattern": {
                    "type": "string",
                    "description": "list mode: only paths containing this substring.",
                },
                "type_name": {
                    "type": "string",
                    "description": "list mode: only prims of this exact type name.",
                },
                "limit": {
                    "type": "integer",
                    "description": "list mode: maximum prims to return (default 200).",
                },
                "prim_path": {
                    "type": "string",
                    "description": "prim mode: the prim path to describe.",
                },
            },
            "required": ["mode"],
        },
    },
    {
        "name": "capture_viewport",
        "description": (
            "Render what the user is currently looking at and return it to you "
            "as an image, together with the camera matrices, eye position, "
            "field of view and frame that produced it. Call it when the answer "
            "depends on the scene's appearance, and again after a visual edit "
            "to confirm the result rather than assuming it."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "note": {
                    "type": "string",
                    "description": "Why you are capturing, shown to the user.",
                },
            },
        },
    },
]


# FoundationModels 2.0.68 accepts OpenAI ``tools`` but its system model does
# not currently produce a usable first-leg ``message.tool_calls``. Forced tool
# choice returns HTTP 500. JSON-schema guided output is reliable when kept
# small, flat and all-required, so Apple uses a proven two-phase protocol: pick
# one tool, then fill only that tool's arguments. This avoids the malformed
# cross-filled fields produced by one oversized union schema.
APPLE_DECISION_PROPERTIES = {
    "action": {
        "type": "string",
        "description": "Exactly tool or respond.",
    },
    "tool": {
        "type": "string",
        "description": (
            "Exactly run_python, inspect_stage, or capture_viewport for a tool "
            "action; empty for respond."),
    },
    "message": {
        "type": "string",
        "description": (
            "Concise final user answer for respond, or a short progress summary "
            "for tool."),
    },
}

APPLE_DECISION_SCHEMA = {
    "title": "MuseDecision",
    "type": "object",
    "properties": APPLE_DECISION_PROPERTIES,
    "required": list(APPLE_DECISION_PROPERTIES),
    "additionalProperties": False,
    "x-order": list(APPLE_DECISION_PROPERTIES),
}

APPLE_TOOL_ARGUMENT_SCHEMAS = {
    "run_python": {
        "title": "MuseRunPythonArguments",
        "type": "object",
        "properties": {
            "code": {"type": "string"},
        },
        "required": ["code"],
        "additionalProperties": False,
        "x-order": ["code"],
    },
    "inspect_stage": {
        "title": "MuseInspectStageArguments",
        "type": "object",
        "properties": {
            "mode": {"type": "string"},
            "pattern": {"type": "string"},
            "type_name": {"type": "string"},
            "limit": {"type": "integer"},
            "prim_path": {"type": "string"},
        },
        "required": ["mode", "pattern", "type_name", "limit", "prim_path"],
        "additionalProperties": False,
        "x-order": ["mode", "pattern", "type_name", "limit", "prim_path"],
    },
    "capture_viewport": {
        "title": "MuseCaptureViewportArguments",
        "type": "object",
        "properties": {
            "note": {"type": "string"},
        },
        "required": ["note"],
        "additionalProperties": False,
        "x-order": ["note"],
    },
}

APPLE_DECISION_INSTRUCTIONS = """\
APPLE ON-DEVICE ACTION PROTOCOL:

First choose exactly one next action. Set `action` to tool or respond. For a
tool action, set `tool` to exactly run_python, inspect_stage, or
capture_viewport. For respond, set `tool` to "" and put the concise final answer
in `message`. Use respond only after observed tool output proves that a
requested stage or viewport change succeeded. When the latest successful tool
result already prints or shows the exact verification the user requested,
respond immediately; do not call inspect_stage merely to repeat that proof.

If a tool reports an error, choose another tool action to repair the cause. Do
not merely describe a tool action in `message`. Tool arguments are requested
separately after this decision.\
"""


# ---------------------------------------------------------------------------
# Message normalization
#
# The Anthropic Messages API rejects a `system` role inside messages, rejects
# two turns of the same role in a row, and requires the first turn to be
# `user`.  A chat panel naturally produces all three (goal notices logged as
# system, the live prompt appended to history before the call).  Normalizing
# here means no caller can construct a request the API will refuse.
# ---------------------------------------------------------------------------

def normalize_messages(raw_messages):
    """
    Return (system_extras, messages) valid for the Messages API.

    * `system` entries are lifted out and returned as context lines rather
      than silently overwriting the real system prompt.
    * consecutive same-role turns are merged into one turn.
    * leading assistant turns are dropped; empty turns are dropped.
    """
    system_extras = []
    cleaned = []
    for entry in raw_messages or []:
        role = entry.get("role")
        content = entry.get("content")
        if content is None or content == "" or content == []:
            continue
        if role == "system":
            if isinstance(content, str):
                system_extras.append(content)
            continue
        if role not in ("user", "assistant"):
            continue
        cleaned.append({"role": role, "content": content})

    # The conversation must open on a user turn.
    while cleaned and cleaned[0]["role"] != "user":
        cleaned.pop(0)

    merged = []
    for entry in cleaned:
        if merged and merged[-1]["role"] == entry["role"]:
            merged[-1] = {
                "role": entry["role"],
                "content": _join_content(merged[-1]["content"], entry["content"]),
            }
        else:
            merged.append(dict(entry))
    return system_extras, merged


def _join_content(first, second):
    """Merge two turns' content, promoting to block lists when needed."""
    if isinstance(first, str) and isinstance(second, str):
        return first + "\n\n" + second
    first_blocks = [{"type": "text", "text": first}] if isinstance(first, str) else list(first)
    second_blocks = [{"type": "text", "text": second}] if isinstance(second, str) else list(second)
    return first_blocks + second_blocks


# ---------------------------------------------------------------------------
# PNG text metadata — how camera info rides along with a screenshot
# ---------------------------------------------------------------------------

_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _png_chunks(data):
    """Yield (type, payload) for each chunk in a PNG byte string."""
    if not data[:8] == _PNG_SIGNATURE:
        return
    offset = 8
    while offset + 8 <= len(data):
        (length,) = struct.unpack(">I", data[offset:offset + 4])
        ctype = data[offset + 4:offset + 8]
        payload = data[offset + 8:offset + 8 + length]
        yield ctype, payload
        offset += 12 + length  # length + type + payload + crc
        if ctype == b"IEND":
            break


def png_read_text(data):
    """Return {keyword: text} for every tEXt/zTXt/iTXt chunk in a PNG."""
    out = {}
    try:
        for ctype, payload in _png_chunks(data):
            if ctype == b"tEXt":
                key, _, value = payload.partition(b"\x00")
                out[key.decode("latin-1")] = value.decode("latin-1")
            elif ctype == b"zTXt":
                key, _, rest = payload.partition(b"\x00")
                if rest[:1] == b"\x00":
                    try:
                        out[key.decode("latin-1")] = zlib.decompress(rest[1:]).decode("utf-8", "replace")
                    except Exception:
                        pass
            elif ctype == b"iTXt":
                # keyword \0 flag(1) method(1) language \0 translated \0 text
                key, _, rest = payload.partition(b"\x00")
                if len(rest) < 2:
                    continue
                compressed = rest[0:1] == b"\x01"
                rest = rest[2:]
                _lang, _, rest = rest.partition(b"\x00")
                _translated, _, text = rest.partition(b"\x00")
                try:
                    if compressed:
                        text = zlib.decompress(text)
                    out[key.decode("latin-1")] = text.decode("utf-8", "replace")
                except Exception:
                    pass
    except Exception:
        pass
    return out


def _chunk_keyword(ctype, payload):
    if ctype in (b"tEXt", b"zTXt", b"iTXt"):
        return payload.partition(b"\x00")[0]
    return None


def png_write_text(data, mapping):
    """
    Return *data* with each key/value in *mapping* stored as an iTXt chunk.

    Any existing text chunk carrying one of these keywords is dropped first, so
    re-tagging an image replaces its metadata instead of layering a second copy
    behind the first.  Values are stored uncompressed so any PNG reader can
    recover them.  Returns *data* unchanged on any problem.
    """
    try:
        if data[:8] != _PNG_SIGNATURE:
            return data
        replaced = set(key.encode("latin-1") for key in mapping)

        kept = b""
        offset = 8
        ihdr = b""
        while offset + 8 <= len(data):
            (length,) = struct.unpack(">I", data[offset:offset + 4])
            ctype = data[offset + 4:offset + 8]
            payload = data[offset + 8:offset + 8 + length]
            raw = data[offset:offset + 12 + length]
            offset += 12 + length
            if ctype == b"IHDR":
                ihdr = raw
            elif _chunk_keyword(ctype, payload) in replaced:
                pass  # superseded by the value we are about to write
            else:
                kept += raw
            if ctype == b"IEND":
                break
        if not ihdr:
            return data

        chunks = b""
        for key, value in mapping.items():
            if value is None:
                continue
            payload = (key.encode("latin-1") + b"\x00"      # keyword
                       + b"\x00"                             # compression flag
                       + b"\x00"                             # compression method
                       + b"\x00"                             # language tag
                       + b"\x00"                             # translated keyword
                       + str(value).encode("utf-8"))
            chunks += (struct.pack(">I", len(payload)) + b"iTXt" + payload
                       + struct.pack(">I", zlib.crc32(b"iTXt" + payload) & 0xFFFFFFFF))
        return _PNG_SIGNATURE + ihdr + chunks + kept
    except Exception:
        return data


# ---------------------------------------------------------------------------
# Attachments — images the user sends, with whatever camera context they carry
# ---------------------------------------------------------------------------

_MEDIA_TYPES = {
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".gif": "image/gif",
    ".webp": "image/webp",
}


class Attachment(object):
    """
    One image heading to the model, plus whatever camera/scene context we could
    recover for it.  Sources, in order of preference:

      1. PNG text chunks written by Muse's own capture path
      2. a sidecar <image>.json / <image>.camera.json next to the file
      3. nothing — the image still goes, flagged as having no camera
    """

    def __init__(self, b64, media_type="image/png", label="", camera=None,
                 scene="", annotation="", source=""):
        self.b64 = b64
        self.media_type = media_type
        self.label = label
        self.camera = camera or {}
        self.scene = scene or ""
        self.annotation = annotation or ""
        self.source = source

    @classmethod
    def from_file(cls, path):
        with open(path, "rb") as handle:
            raw = handle.read()
        ext = os.path.splitext(path)[1].lower()
        media_type = _MEDIA_TYPES.get(ext, "image/png")

        camera, scene, annotation = {}, "", ""
        if media_type == "image/png":
            text = png_read_text(raw)
            if CAMERA_METADATA_KEY in text:
                try:
                    camera = json.loads(text[CAMERA_METADATA_KEY])
                except Exception:
                    camera = {"parse_error": "camera metadata present but unreadable"}
            scene = text.get(SCENE_METADATA_KEY, "")
            annotation = text.get(ANNOTATION_METADATA_KEY, "")

        if not camera:
            camera = _read_sidecar_camera(path)

        return cls(
            b64=base64.b64encode(raw).decode("ascii"),
            media_type=media_type,
            label=os.path.basename(path),
            camera=camera,
            scene=scene,
            annotation=annotation,
            source=path,
        )

    def has_camera(self):
        return bool(self.camera) and "parse_error" not in self.camera

    def to_context_block(self, index):
        lines = ["--- Image %d: %s ---" % (index + 1, self.label or "(attachment)")]
        if self.annotation:
            lines.append("Annotation: %s" % self.annotation)
        if self.has_camera():
            lines.append("Camera metadata:\n%s" % json.dumps(self.camera, indent=2, default=str))
        else:
            lines.append("Camera metadata: none recorded for this image.")
        if self.scene:
            scene = self.scene if len(self.scene) <= 3000 else self.scene[:3000] + "\n… (truncated)"
            lines.append("Scene at capture:\n%s" % scene)
        return "\n".join(lines)

    def to_image_block(self):
        return {
            "type": "image",
            "source": {
                "type": "base64",
                "media_type": self.media_type,
                "data": self.b64,
            },
        }


def _read_sidecar_camera(image_path):
    stem = os.path.splitext(image_path)[0]
    for candidate in (stem + ".camera.json", stem + ".json", image_path + ".json"):
        if os.path.isfile(candidate):
            try:
                with open(candidate, "r") as handle:
                    data = json.load(handle)
                if isinstance(data, dict):
                    return data.get("camera", data)
            except Exception:
                continue
    return {}


def build_user_content(prompt, attachments):
    """
    Assemble one user turn: the attachments' context text and images first,
    then the user's request.  Returns a plain string when there is nothing
    attached, so simple turns stay cheap.
    """
    if not attachments:
        return prompt
    blocks = []
    header = ["%d image(s) attached." % len(attachments)]
    for index, item in enumerate(attachments):
        header.append(item.to_context_block(index))
    blocks.append({"type": "text", "text": "\n\n".join(header)})
    for item in attachments:
        blocks.append(item.to_image_block())
    blocks.append({"type": "text", "text": prompt})
    return blocks


# ---------------------------------------------------------------------------
# The agent loop
# ---------------------------------------------------------------------------

class AgentError(RuntimeError):
    pass


def resolve_api_key():
    """
    Read the key from the environment at call time, so setting it from the
    panel takes effect on the next send without a restart.

    Surrounding quotes are stripped: `export MUSE_API_KEY="sk-..."` is fine in
    a shell, but a key pasted into a launcher script or a plist often keeps
    them, and the API rejects the quoted form with a bare 401.
    """
    for name in ("MUSE_API_KEY", "ANTHROPIC_API_KEY"):
        value = os.environ.get(name, "").strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1].strip()
        if value:
            return value, name
    return None, None


def _env_base_url():
    """The endpoint named by the environment, or (None, None).

    Split out of resolve_base_url so resolve_provider can consult it without
    the two calling each other in a circle.
    """
    for name in ("MUSE_BASE_URL", "ANTHROPIC_BASE_URL"):
        value = os.environ.get(name, "").strip().rstrip("/")
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1].strip().rstrip("/")
        if value:
            return value, name
    return None, None


def resolve_provider():
    """Which back end this session talks to.

    MUSE_PROVIDER states it outright; that is the only way to select a local
    server, because its address alone does not identify its protocol. Without
    it the answer is inferred exactly the way it always was, so a session that
    never heard of providers behaves identically.
    """
    explicit = os.environ.get("MUSE_PROVIDER", "").strip().lower()
    if explicit in PROVIDERS:
        return explicit
    if explicit:
        _warn("MUSE_PROVIDER=%r is not one of %s; inferring instead."
              % (explicit, ", ".join(PROVIDERS)))

    base_url, _name = _env_base_url()
    if base_url and META_HOST in base_url:
        return PROVIDER_META
    api_key, _key_name = resolve_api_key()
    if api_key and api_key.startswith(META_KEY_PREFIX):
        return PROVIDER_META
    return PROVIDER_ANTHROPIC


def resolve_ollama_base_url():
    """The Ollama address: MUSE_OLLAMA_URL, else the default."""
    value = os.environ.get("MUSE_OLLAMA_URL", "").strip().rstrip("/")
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1].strip().rstrip("/")
    return value or OLLAMA_DEFAULT_BASE_URL


def resolve_lmstudio_base_url():
    """The LM Studio address: MUSE_LMSTUDIO_URL, else Hivemind."""
    value = os.environ.get("MUSE_LMSTUDIO_URL", "").strip().rstrip("/")
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1].strip().rstrip("/")
    return value or LMSTUDIO_DEFAULT_BASE_URL


def resolve_apple_base_url():
    """The loopback ``fm serve`` address, never a hosted gateway."""
    value = os.environ.get("MUSE_APPLE_URL", "").strip().rstrip("/")
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1].strip().rstrip("/")
    return value or APPLE_DEFAULT_BASE_URL


def resolve_base_url():
    """
    Which Messages API endpoint to talk to.

    An explicit MUSE_BASE_URL always wins.  Otherwise the provider decides: a
    Meta Muse key (`LLM_…`) selects Meta's endpoint, because
    api.anthropic.com answers that key shape with 401 and routing it to
    Anthropic could only ever fail; MUSE_PROVIDER=ollama selects the local
    server.
    """
    provider = resolve_provider()
    # Apple and LM Studio have dedicated URL settings and are intentionally
    # isolated from MUSE_BASE_URL. That variable names a generic gateway and
    # must never redirect either session (or its viewport images) elsewhere.
    if provider == PROVIDER_APPLE:
        return resolve_apple_base_url(), "Apple Foundation Models"
    if provider == PROVIDER_LMSTUDIO:
        return resolve_lmstudio_base_url(), "LM Studio on Hivemind"

    value, name = _env_base_url()
    if value:
        return value, name

    if provider == PROVIDER_OLLAMA:
        return resolve_ollama_base_url(), "Ollama"
    if provider == PROVIDER_META:
        return META_BASE_URL, "Meta Muse key"
    return None, None


def resolve_auth_style(base_url=None):
    """Which header should carry the credential: "bearer" or "api_key".

    The scheme belongs to the endpoint, not to the key. Meta documents a bearer
    token for api.meta.ai, while api.anthropic.com takes x-api-key — and a
    Messages-API gateway configured through MUSE_BASE_URL is a third service
    whose scheme we do not know, so it keeps the x-api-key it has always been
    sent.

    The SDK picks the header from which argument it receives: api_key= sends
    X-Api-Key, auth_token= sends Authorization: Bearer.
    """
    if base_url and META_HOST in base_url:
        return "bearer"
    return "api_key"


def _warn(message):
    """Report a correction. Plain stderr because this module has no pxr
    dependency — that is what keeps it importable and testable headlessly."""
    sys.stderr.write("museAgent: %s\n" % message)


def force_contributor_model(model, base_url):
    """Every Meta call goes through the contributor tier.

    Applied to both ways a model can be chosen — MUSE_MODEL and an explicit
    model= argument — so there is one place that decides, rather than two that
    have to agree.

    A standard-tier id is replaced outright rather than having "-contributor"
    appended to it: only muse-spark-1.3-contributor is published, so
    synthesising "muse-spark-1.1-contributor" would produce a name that does
    not exist and fail at the API instead of here.

    Anthropic and MUSE_BASE_URL gateways are left alone; the tier is a Meta
    concept and those services have never heard of it.
    """
    if not base_url or META_HOST not in base_url:
        return model
    if model and model.endswith(META_CONTRIBUTOR_SUFFIX):
        return model
    if model and model != META_DEFAULT_MODEL:
        _warn("%r is not a contributor model; using %s instead."
              % (model, META_DEFAULT_MODEL))
    return META_DEFAULT_MODEL


def resolve_model(base_url=None, provider=None):
    """The model id, defaulted to match whichever back end we are pointed at."""
    selected_provider = provider or resolve_provider()
    if selected_provider == PROVIDER_APPLE:
        explicit = os.environ.get("MUSE_MODEL", "").strip()
        if explicit and explicit != APPLE_MODEL:
            _warn("Apple sessions are on-device only; ignoring MUSE_MODEL=%r "
                  "and using %s." % (explicit, APPLE_MODEL))
        return APPLE_MODEL
    explicit = os.environ.get("MUSE_MODEL", "").strip()
    if explicit:
        return force_contributor_model(explicit, base_url)
    if selected_provider == PROVIDER_LMSTUDIO:
        return preferred_lmstudio_model(fetch_lmstudio_models(base_url))
    if base_url and META_HOST in base_url:
        return META_DEFAULT_MODEL
    if selected_provider == PROVIDER_OLLAMA:
        return OLLAMA_DEFAULT_MODEL
    return DEFAULT_MODEL


def describe_key_problem(api_key, base_url, provider=None):
    """
    Return a problem description for *api_key*, or None when it is worth
    sending.  Split out so the panel can warn before the user hits Send.
    """
    # Ollama authenticates nothing. Demanding a key here would make a local
    # server the one back end you cannot use without signing up for a hosted
    # one, which is the opposite of the point.
    if (provider or resolve_provider()) in (
            PROVIDER_OLLAMA, PROVIDER_LMSTUDIO, PROVIDER_APPLE):
        return None
    if not api_key:
        return ("No API key found. Set MUSE_API_KEY (or ANTHROPIC_API_KEY) in the "
                "environment that launches usdview, or use 'Set API key…' in the "
                "panel to set one for this process.")
    if base_url:
        return None
    if api_key.startswith("sk-ant-"):
        return None
    return ("This key is not an Anthropic key (api.anthropic.com accepts "
            "sk-ant-… and answers anything else with 401 invalid x-api-key), "
            "and no endpoint is configured for it. Set MUSE_BASE_URL to the "
            "Messages API endpoint that issued this key — for a Meta Muse key "
            "that is %s." % META_BASE_URL)


def _apple_http_json(path, payload=None, base_url=None, timeout=None):
    """Read one JSON response from ``fm serve`` with actionable errors."""
    import urllib.error
    import urllib.parse
    import urllib.request

    root = (base_url or resolve_apple_base_url()).rstrip("/")
    parsed = urllib.parse.urlsplit(root)
    if (parsed.scheme != "http"
            or parsed.hostname not in ("127.0.0.1", "localhost", "::1")):
        raise AgentError(
            "MUSE_APPLE_URL must be a local HTTP loopback address; got %s. "
            "Muse will not send stage context or viewport images to a remote "
            "host through the Apple provider." % root)
    url = root + path
    data = None
    headers = {"accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["content-type"] = "application/json"
    request = urllib.request.Request(url, data=data, headers=headers)
    # A URL spelling 127.0.0.1 is not sufficient to keep data local: urllib's
    # default opener honours HTTP_PROXY, and its redirect handler can follow a
    # loopback 3xx to another host. Disable both for stage context and viewport
    # images sent through the explicitly on-device provider.
    class _NoAppleRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, req, fp, code, msg, response_headers, newurl):
            return None

    opener = urllib.request.build_opener(
        urllib.request.ProxyHandler({}), _NoAppleRedirect())
    budget = timeout if timeout is not None else APPLE_REQUEST_TIMEOUT
    try:
        with opener.open(request, timeout=budget) as response:
            raw = response.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as error:
        try:
            raw = error.read().decode("utf-8", "replace")
        except Exception:
            raw = ""
        detail = raw
        try:
            body = json.loads(raw)
            nested = body.get("error") if isinstance(body, dict) else None
            if isinstance(nested, dict):
                detail = nested.get("message") or raw
            elif nested:
                detail = str(nested)
        except Exception:
            pass
        raise AgentError("Apple FM at %s returned HTTP %s: %s"
                         % (root, error.code, detail or error.reason))
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        reason = getattr(error, "reason", None) or str(error)
        raise AgentError("Apple FM at %s is not answering: %s. Start `fm serve` "
                         "and keep it running." % (root, reason))
    try:
        decoded = json.loads(raw)
    except ValueError:
        raise AgentError("Apple FM at %s returned invalid JSON: %s"
                         % (root, raw[:500]))
    if not isinstance(decoded, dict):
        raise AgentError("Apple FM at %s returned an unexpected JSON value."
                         % root)
    return decoded


def fetch_apple_health(base_url=None, timeout=None):
    """Return the live ``fm serve`` health payload, or raise AgentError."""
    return _apple_http_json(
        "/health", base_url=base_url,
        timeout=APPLE_HEALTH_TIMEOUT if timeout is None else timeout)


def describe_apple_health_problem(health):
    """Why the on-device system model is unusable, or None when ready."""
    if not isinstance(health, dict):
        return "Apple FM returned no health information."
    models = health.get("models") or []
    system = next((entry for entry in models
                   if isinstance(entry, dict) and entry.get("name") == APPLE_MODEL),
                  None)
    if system is None:
        return "Apple FM did not list the on-device `system` model."
    if system.get("available") is not True:
        return (system.get("reason")
                or "Apple FM reports that the on-device `system` model is unavailable.")
    return None


# LM Studio model lists, keyed by server address. Its native endpoint exposes
# the capability data the OpenAI-compatible /v1/models response may omit.
_LMSTUDIO_MODEL_CACHE = {}


def clear_lmstudio_model_cache():
    """Forget cached LM Studio model lists (the dialog's Refresh calls this)."""
    _LMSTUDIO_MODEL_CACHE.clear()


def fetch_lmstudio_models(base_url=None, timeout=None, use_cache=True):
    """Return LM Studio LLMs with native-tool, vision and load-state metadata.

    LM Studio gives every LLM a default prompted tool format; the
    ``trained_for_tool_use`` capability means the stronger native tool path,
    not that other LLMs cannot call tools at all. The native v1 endpoint is
    preferred for that distinction, with /v1/models as a compatibility
    fallback for an older server.
    """
    import urllib.parse
    import urllib.request

    root = (base_url or resolve_lmstudio_base_url()).rstrip("/")
    if use_cache and root in _LMSTUDIO_MODEL_CACHE:
        return _LMSTUDIO_MODEL_CACHE[root]
    parsed = urllib.parse.urlsplit(root)
    if parsed.scheme not in ("http", "https") or not parsed.hostname:
        _LMSTUDIO_MODEL_CACHE[root] = []
        return []

    budget = timeout or LMSTUDIO_LIST_TIMEOUT
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    payload = None
    for path in ("/api/v1/models", "/v1/models"):
        try:
            with opener.open(root + path, timeout=budget) as response:
                candidate = json.loads(response.read().decode("utf-8"))
            if isinstance(candidate, dict):
                payload = candidate
                break
        except Exception:
            continue
    if payload is None:
        _LMSTUDIO_MODEL_CACHE[root] = []
        return []

    native_entries = payload.get("models")
    entries = native_entries if isinstance(native_entries, list) \
        else payload.get("data") or []
    native_shape = isinstance(native_entries, list)
    models = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        model_type = str(entry.get("type") or "").lower()
        if model_type in ("embedding", "embeddings"):
            continue
        name = entry.get("key") if native_shape else entry.get("id")
        name = name or entry.get("id") or entry.get("key")
        if not name:
            continue
        capabilities = entry.get("capabilities") or {}
        if isinstance(capabilities, dict):
            native_tools = bool(
                capabilities.get("trained_for_tool_use")
                or capabilities.get("tool_use")
                or capabilities.get("tools"))
            vision = bool(capabilities.get("vision"))
        else:
            capability_names = set(capabilities or [])
            native_tools = bool(
                capability_names.intersection(("tool_use", "tools")))
            vision = "vision" in capability_names
        loaded_instances = entry.get("loaded_instances") or []
        state = str(entry.get("state") or "").lower()
        models.append({
            "name": str(name),
            "display_name": str(entry.get("display_name") or name),
            # LM Studio supplies a default tool format even when this is not
            # a model-native capability. Keep the distinction visible in UI.
            "tools": True,
            "native_tools": native_tools,
            "vision": vision or model_type == "vlm",
            "loaded": bool(loaded_instances) or state == "loaded",
        })
    _LMSTUDIO_MODEL_CACHE[root] = models
    return models


def preferred_lmstudio_model(models):
    """Choose a useful default without hard-coding Hivemind's model names."""
    if not models:
        return ""
    # Avoid an unnecessary load first, then prefer native tool syntax and
    # vision. max() is stable for equal scores, preserving server order.
    return max(models, key=lambda entry: (
        4 if entry.get("loaded") else 0,
        2 if entry.get("native_tools") else 0,
        1 if entry.get("vision") else 0))["name"]


# Model lists, keyed by server address.
#
# fetch_ollama_models costs a round trip per model, and run_agent checks the
# chosen model on EVERY send -- so without this, every message the user types
# waits on ~11 HTTP requests before the first token. An installed model set
# does not change mid-session in any way that matters here, and the settings
# dialog's Refresh button is the explicit way to re-read it.
_OLLAMA_MODEL_CACHE = {}


def clear_ollama_model_cache():
    """Forget the cached model lists (the dialog's Refresh calls this)."""
    _OLLAMA_MODEL_CACHE.clear()


def fetch_ollama_models(base_url=None, timeout=None, use_cache=True):
    """The models an Ollama server has, with their capabilities.

    Returns a list of dicts: name, tools, vision, thinking. An unreachable
    server returns [] rather than raising -- the settings dialog has to stay
    usable when the machine is off, and "no models listed" says that plainly
    enough.

    The list comes from /api/tags but the CAPABILITIES come from /api/show,
    one call per model, because the two disagree. Measured on a live server:
    /api/tags reported "completion" only for four of ten models -- every
    user-namespaced one (smtek/..., kwangsuklee/..., zfujicute/...) -- while
    /api/show reported tools for all four, and a real tools request to one of
    them came back with stop_reason "tool_use" and a correct block. Trusting
    /api/tags therefore refuses working models, which for a local server is
    the worst possible direction to be wrong in.

    Ten models cost about 1.2s, so this stays a click, not a wait.
    """
    import urllib.request

    root = (base_url or resolve_ollama_base_url()).rstrip("/")
    if use_cache and root in _OLLAMA_MODEL_CACHE:
        return _OLLAMA_MODEL_CACHE[root]
    budget = timeout or OLLAMA_LIST_TIMEOUT
    try:
        with urllib.request.urlopen(root + "/api/tags",
                                    timeout=budget) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except Exception:
        # Cached too. A server that is off stays off for the session as far as
        # this is concerned, and re-probing it on every send would put the
        # connection timeout in front of every message.
        _OLLAMA_MODEL_CACHE[root] = []
        return []

    models = []
    for entry in (payload or {}).get("models") or []:
        name = entry.get("name")
        if not name:
            continue
        # /api/tags' capabilities are the fallback, used only when the
        # authoritative call fails.
        caps = set(entry.get("capabilities") or [])
        try:
            request = urllib.request.Request(
                root + "/api/show",
                data=json.dumps({"model": name}).encode("utf-8"),
                headers={"content-type": "application/json"})
            with urllib.request.urlopen(request, timeout=budget) as response:
                shown = json.loads(response.read().decode("utf-8"))
            caps = set(shown.get("capabilities") or caps)
        except Exception:
            pass
        models.append({
            "name": name,
            "tools": "tools" in caps,
            "vision": "vision" in caps,
            "thinking": "thinking" in caps,
        })
    _OLLAMA_MODEL_CACHE[root] = models
    return models


def describe_model_problem(model, provider=None, models=None):
    """Why *model* cannot do Muse's job, or None.

    This is the check worth having. Muse does not answer questions about a
    stage -- it edits one, entirely through tool calls, and a model with no
    tool support simply never emits any. The panel then streams a perfectly
    fluent reply while the stage sits untouched, which is the exact failure
    the canned-reply stub used to produce and which this plugin has a standing
    rule against.

    Ollama is judged from /api/show, never from its model name or /api/tags
    summary. LM Studio is judged from its native /api/v1/models inventory;
    models without native tool training still have LM Studio's prompted tool
    format and remain selectable.

    Returns None for the hosted back ends: their model ids are fixed and their
    capabilities are not ours to enumerate.
    """
    selected_provider = provider or resolve_provider()
    if selected_provider not in (PROVIDER_OLLAMA, PROVIDER_LMSTUDIO):
        return None
    if models is None:
        models = (fetch_lmstudio_models()
                  if selected_provider == PROVIDER_LMSTUDIO
                  else fetch_ollama_models())
    if not models:
        return None  # server unreachable; not the model's fault
    if not model:
        return ("No LM Studio model is selected. Choose one in Muse settings."
                if selected_provider == PROVIDER_LMSTUDIO else None)

    known = {entry["name"]: entry for entry in models}
    entry = known.get(model)
    if entry is None:
        if selected_provider == PROVIDER_LMSTUDIO:
            return ("%s is not available on this LM Studio server. "
                    "Available: %s"
                    % (model, ", ".join(sorted(known)) or "none"))
        return ("%s is not installed on this Ollama server. Installed: %s"
                % (model, ", ".join(sorted(known)) or "none"))
    if not entry["tools"]:
        capable = sorted(n for n, e in known.items() if e["tools"])
        return ("%s has no tool support, and Muse works entirely through tool "
                "calls -- it would reply fluently and change nothing on your "
                "stage. Choose a model that supports tools%s."
                % (model, (": " + ", ".join(capable)) if capable else
                   " (this server has none installed)"))
    return None


def describe_auth_failure(exc, api_key, key_name, base_url, base_name):
    """Explain a 401 from the endpoint, or return None for any other error.

    describe_key_problem() runs before the request and can only judge the key's
    shape. This runs after, when the endpoint itself has refused the
    credential, and the raw SDK text for that is a repr of the error JSON —
    accurate but it names neither the endpoint that answered nor the key that
    was sent, and both are chosen implicitly for a Meta Muse key.
    """
    status = getattr(exc, "status_code", None)
    if status != 401 and "401" not in str(exc):
        return None
    if status not in (401, None):
        return None

    where = base_url or "https://api.anthropic.com"
    origin = "%s (%s)" % (where, base_name) if base_name else where
    header = ("Authorization: Bearer"
              if resolve_auth_style(base_url) == "bearer" else "x-api-key")
    identity = ("%s, %d chars, ends %s, sent as %s"
                % (key_name or "the key", len(api_key or ""),
                   (api_key or "")[-4:], header)
                if api_key else "no key")

    return (
        "%s rejected the key (HTTP 401).\n"
        "\n"
        "Sent: %s\n"
        "\n"
        "A 401 here is the endpoint refusing the credential itself, not a "
        "formatting or routing problem — an identical request carrying no key "
        "at all gets the same answer. The key is expired, revoked, or was "
        "issued for a different service, and no change on this side will make "
        "it work. Replace it, then use 'Set API key…' or relaunch with "
        "MUSE_API_KEY set.\n"
        "\n"
        "If instead the key is good and the endpoint is wrong, point "
        "MUSE_BASE_URL at the Messages API that issued it."
        % (origin, identity))


def build_system_prompt(stage_context="", goal=None, extras=None):
    parts = [SYSTEM_PROMPT]
    if goal:
        parts.append(
            "LONG-HORIZON GOAL (set via /goal, keep every turn aligned to it "
            "until it is cleared): %s" % goal)
    if stage_context:
        parts.append("Current stage:\n%s" % stage_context)
    for extra in extras or []:
        if extra:
            parts.append(extra)
    return "\n\n".join(parts)


def _block_value(block, name, default=None):
    """Read a Messages block represented by either an SDK object or a dict."""
    if isinstance(block, dict):
        return block.get(name, default)
    return getattr(block, name, default)


def _apple_image_part(block):
    """Translate one Anthropic-style image block to an OpenAI data URL part."""
    source = _block_value(block, "source", {}) or {}
    source_type = _block_value(source, "type", "")
    if source_type != "base64":
        return None
    encoded = _block_value(source, "data", "") or ""
    media_type = _block_value(source, "media_type", "image/png") or "image/png"
    if not encoded:
        return None
    return {
        "type": "image_url",
        "image_url": {"url": "data:%s;base64,%s" % (media_type, encoded)},
    }


def _apple_regular_content(content):
    """Translate ordinary text/image content to Chat Completions content."""
    if isinstance(content, str):
        return content
    parts = []
    for block in content or []:
        kind = _block_value(block, "type")
        if kind == "text":
            text = _block_value(block, "text", "") or ""
            if text:
                parts.append({"type": "text", "text": text})
        elif kind == "image":
            image = _apple_image_part(block)
            if image:
                parts.append(image)
    return parts or ""


def _apple_tool_result_text(result):
    """Text accepted in a role=tool message; images travel in a user turn."""
    content = _block_value(result, "content", "")
    if isinstance(content, str):
        text = content
    else:
        text = "\n".join(
            _block_value(block, "text", "") or ""
            for block in content or []
            if _block_value(block, "type") == "text").strip()
    if not text:
        text = "(no output)"
    if _block_value(result, "is_error", False):
        text = "ERROR:\n" + text
    return text


def _apple_chat_messages(conversation, system_prompt):
    """Translate Muse's canonical Messages history to Chat Completions."""
    translated = [{"role": "system", "content": system_prompt}]
    for entry in conversation:
        role = entry.get("role")
        content = entry.get("content")

        if role == "assistant":
            if isinstance(content, str):
                translated.append({"role": "assistant", "content": content})
                continue
            text_parts = []
            tool_calls = []
            for block in content or []:
                kind = _block_value(block, "type")
                if kind == "text":
                    text = _block_value(block, "text", "") or ""
                    if text:
                        text_parts.append(text)
                elif kind == "tool_use":
                    tool_calls.append({
                        "id": _block_value(block, "id", ""),
                        "type": "function",
                        "function": {
                            "name": _block_value(block, "name", ""),
                            "arguments": json.dumps(
                                _block_value(block, "input", {}) or {},
                                separators=(",", ":")),
                        },
                    })
            message = {
                "role": "assistant",
                "content": "\n\n".join(text_parts) if text_parts else None,
            }
            if tool_calls:
                message["tool_calls"] = tool_calls
            translated.append(message)
            continue

        if role != "user":
            continue
        blocks = content if isinstance(content, list) else []
        results = [block for block in blocks
                   if _block_value(block, "type") == "tool_result"]
        if not results:
            translated.append({
                "role": "user",
                "content": _apple_regular_content(content),
            })
            continue

        image_parts = []
        for result in results:
            translated.append({
                "role": "tool",
                "tool_call_id": _block_value(result, "tool_use_id", ""),
                "content": _apple_tool_result_text(result),
            })
            result_content = _block_value(result, "content", [])
            if isinstance(result_content, list):
                for block in result_content:
                    if _block_value(block, "type") == "image":
                        image = _apple_image_part(block)
                        if image:
                            image_parts.append(image)
        if image_parts:
            translated.append({
                "role": "user",
                "content": ([{"type": "text", "text":
                              "Viewport image returned by capture_viewport."}]
                            + image_parts),
            })
        # normalize_messages can merge the trailing user tool_result from an
        # exhausted turn with the user's next "continue" text. Preserve those
        # ordinary blocks after satisfying every role=tool obligation.
        regular_blocks = [block for block in blocks
                          if _block_value(block, "type") != "tool_result"]
        if regular_blocks:
            translated.append({
                "role": "user",
                "content": _apple_regular_content(regular_blocks),
            })
    return translated


def _split_apple_mixed_user_turns(conversation):
    """Separate a merged tool-result + follow-up user turn at its boundary."""
    split = []
    for entry in conversation:
        content = entry.get("content")
        if entry.get("role") != "user" or not isinstance(content, list):
            split.append(entry)
            continue
        results = [block for block in content
                   if _block_value(block, "type") == "tool_result"]
        regular = [block for block in content
                   if _block_value(block, "type") != "tool_result"]
        if not results or not regular:
            split.append(entry)
            continue
        split.append({"role": "user", "content": results})
        split.append({"role": "user", "content": regular})
    return split


def _apple_is_user_request(entry):
    if entry.get("role") != "user":
        return False
    content = entry.get("content")
    if isinstance(content, str):
        return bool(content.strip())
    return any(_block_value(block, "type") != "tool_result"
               for block in content or [])


def _apple_entry_size(entries):
    return len(json.dumps(entries, default=str, separators=(",", ":")))


def _apple_tool_pair(entry, next_entry):
    if entry.get("role") != "assistant" or next_entry.get("role") != "user":
        return False
    assistant_content = entry.get("content")
    result_content = next_entry.get("content")
    if not isinstance(assistant_content, list) or not isinstance(result_content, list):
        return False
    call_ids = {_block_value(block, "id") for block in assistant_content
                if _block_value(block, "type") == "tool_use"}
    result_ids = {_block_value(block, "tool_use_id") for block in result_content
                  if _block_value(block, "type") == "tool_result"}
    return bool(call_ids) and call_ids == result_ids


def _compact_apple_exchange(group, budget):
    """Drop oldest complete tool pairs when one active task exceeds budget."""
    if _apple_entry_size(group) <= budget or len(group) <= 1:
        return group
    head = [group[0]]
    segments = []
    index = 1
    while index < len(group):
        if index + 1 < len(group) and _apple_tool_pair(
                group[index], group[index + 1]):
            segments.append(group[index:index + 2])
            index += 2
        else:
            segments.append([group[index]])
            index += 1
    kept = []
    size = _apple_entry_size(head)
    for segment in reversed(segments):
        segment_size = _apple_entry_size(segment)
        # Always retain the newest segment, even if its current viewport image
        # alone is larger than the heuristic character budget.
        if kept and size + segment_size > budget:
            break
        kept[0:0] = segment
        size += segment_size
    return head + kept


def _compact_apple_conversation(conversation):
    """Keep recent complete user exchanges within the local model's context.

    The active exchange is never split, so synthetic tool_call/tool-result
    pairs stay valid. Older exchanges (especially ones carrying base64 viewport
    images) fall away as whole units. The returned history is only the request
    view; run_agent still returns the caller's full canonical conversation.
    """
    try:
        budget = max(1000, int(os.environ.get(
            "MUSE_APPLE_CONTEXT_CHARS", APPLE_CONTEXT_CHAR_BUDGET)))
    except ValueError:
        budget = APPLE_CONTEXT_CHAR_BUDGET
    conversation = _split_apple_mixed_user_turns(conversation)
    starts = [index for index, entry in enumerate(conversation)
              if _apple_is_user_request(entry)]
    if not starts:
        return list(conversation)
    groups = []
    for position, start in enumerate(starts):
        end = starts[position + 1] if position + 1 < len(starts) else len(conversation)
        groups.append(conversation[start:end])
    groups[-1] = _compact_apple_exchange(groups[-1], budget)
    kept = []
    size = 0
    for group in reversed(groups):
        group_size = _apple_entry_size(group)
        if kept and size + group_size > budget:
            break
        kept[0:0] = group
        size += group_size
    return kept


def _apple_completion_text(response):
    """Extract one assistant content string from a Chat Completion."""
    choices = response.get("choices") or []
    if not choices:
        raise AgentError("Apple FM returned no completion choices.")
    choice = choices[0] or {}
    if choice.get("finish_reason") == "content_filter":
        raise AgentError("Apple FM stopped the response at its safety filter.")
    message = choice.get("message") or {}
    refusal = message.get("refusal")
    if refusal:
        raise AgentError("Apple FM declined this request: %s" % refusal)
    content = message.get("content")
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "\n".join(
            part.get("text", "") for part in content
            if isinstance(part, dict) and part.get("type") in ("text", "output_text"))
    raise AgentError("Apple FM returned no assistant content.")


def _parse_apple_object(text, label="structured output"):
    """Decode guided JSON, tolerating an unnecessary Markdown fence."""
    candidate = (text or "").strip()
    if candidate.startswith("```"):
        lines = candidate.splitlines()
        if lines and lines[0].startswith("```"):
            lines.pop(0)
        if lines and lines[-1].strip().startswith("```"):
            lines.pop()
        candidate = "\n".join(lines).strip()
    try:
        action = json.loads(candidate)
    except ValueError:
        start = candidate.find("{")
        end = candidate.rfind("}")
        try:
            action = json.loads(candidate[start:end + 1]) if start >= 0 and end > start else None
        except ValueError:
            action = None
    if not isinstance(action, dict):
        raise AgentError("Apple FM returned invalid %s: %s"
                         % (label, candidate[:800]))
    return action


def _normalize_apple_decision(value):
    """Normalize common small-model aliases without widening the tool set."""
    action = str(value.get("action") or "").strip().lower().replace("-", "_")
    tool = str(value.get("tool") or "").strip().lower().replace("-", "_")
    message = str(value.get("message") or "").strip()
    aliases = {
        "python": "run_python",
        "runpython": "run_python",
        "inspect": "inspect_stage",
        "stage": "inspect_stage",
        "capture": "capture_viewport",
        "viewport": "capture_viewport",
    }
    tool = aliases.get(tool, tool)
    if action in APPLE_TOOL_ARGUMENT_SCHEMAS:
        tool = action
        action = "tool"
    if action in ("answer", "complete", "final", "done"):
        action = "respond"
    if action == "respond":
        if not message:
            raise AgentError("Apple FM chose respond but returned no message.")
        return {"action": "respond", "tool": "", "message": message}
    if action != "tool" or tool not in APPLE_TOOL_ARGUMENT_SCHEMAS:
        raise AgentError(
            "Apple FM returned an invalid decision (action=%r, tool=%r)."
            % (action, tool))
    return {"action": "tool", "tool": tool, "message": message}


def _normalize_apple_tool_arguments(name, value):
    """Keep only selected-tool fields and repair harmless empty defaults."""
    if name == "run_python":
        code = value.get("code") or ""
        if not isinstance(code, str) or not code.strip():
            raise AgentError("Apple FM returned empty run_python code.")
        try:
            compile(code, "<muse-apple>", "exec")
        except SyntaxError as error:
            raise AgentError("Apple FM returned invalid Python: %s" % error)
        lowered = code.lower()
        forbidden = next((token for token in (
            "getactivestage", "usd.stage.open", "usd.stage.createnew",
            "usd.stage.createinmemory", "import usd\n", "import usdgeom\n",
            "from usd import", "from usdgeom import")
            if token in lowered), None)
        rebinds_stage = any(
            line.lstrip().startswith("stage =") for line in code.splitlines())
        if forbidden or rebinds_stage:
            raise AgentError(
                "Apple FM tried to obtain or replace the live stage (%s); it "
                "must use the pre-bound `stage` directly."
                % (forbidden or "stage assignment"))
        return {"code": code}
    if name == "inspect_stage":
        mode = str(value.get("mode") or "summary").strip().lower()
        if mode not in ("summary", "list", "prim"):
            mode = "summary"
        limit = value.get("limit")
        if not isinstance(limit, int) or limit <= 0:
            limit = None
        pattern = value.get("pattern") or None
        type_name = value.get("type_name") or None
        prim_path = value.get("prim_path") or None
        if mode == "summary":
            pattern = type_name = prim_path = None
            limit = None
        elif mode == "list":
            prim_path = None
        else:  # prim
            pattern = type_name = None
            limit = None
        return {
            "mode": mode,
            "pattern": pattern,
            "type_name": type_name,
            "limit": limit,
            "prim_path": prim_path,
        }
    if name == "capture_viewport":
        return {"note": value.get("note") or ""}
    raise AgentError("Apple FM selected an unknown Muse tool %r." % name)


def _apple_guided_request(conversation, system_prompt, schema_name, schema,
                          base_url, extra_prompt=""):
    messages = _apple_chat_messages(conversation, system_prompt)
    if extra_prompt:
        messages.append({"role": "user", "content": extra_prompt})
    request = {
        "model": APPLE_MODEL,
        "stream": False,
        "messages": messages,
        "response_format": {
            "type": "json_schema",
            "json_schema": {"name": schema_name, "schema": schema},
        },
    }
    response = _apple_http_json(
        "/v1/chat/completions", payload=request, base_url=base_url,
        timeout=_apple_timeout())
    return _parse_apple_object(
        _apple_completion_text(response), label=schema_name)


def _apple_timeout():
    try:
        return max(1.0, float(os.environ.get(
            "MUSE_APPLE_TIMEOUT", APPLE_REQUEST_TIMEOUT)))
    except (TypeError, ValueError):
        return APPLE_REQUEST_TIMEOUT


def _run_apple_agent(messages, executor, system_prompt, on_event,
                     model=None, max_iterations=DEFAULT_MAX_ITERATIONS,
                     should_stop=None):
    """Drive Muse through the on-device ``fm serve`` action protocol."""
    base_url = resolve_apple_base_url()
    health = fetch_apple_health(base_url)
    health_problem = describe_apple_health_problem(health)
    if health_problem:
        raise AgentError("Apple FM at %s is not ready: %s"
                         % (base_url, health_problem))

    if model and model != APPLE_MODEL:
        _warn("Apple sessions are on-device only; ignoring model=%r and using %s."
              % (model, APPLE_MODEL))
    model = APPLE_MODEL

    system_extras, conversation = normalize_messages(messages)
    if system_extras:
        system_prompt = system_prompt + "\n\nSession notes:\n" + "\n".join(system_extras)
    system_prompt = system_prompt + "\n\n" + APPLE_DECISION_INSTRUCTIONS
    if not conversation:
        raise AgentError("Nothing to send — the conversation has no user turn.")

    try:
        apple_limit = int(os.environ.get(
            "MUSE_APPLE_MAX_ITERATIONS", APPLE_MAX_ITERATIONS))
    except ValueError:
        apple_limit = APPLE_MAX_ITERATIONS
    iteration_limit = min(max_iterations, max(1, apple_limit))

    for iteration in range(iteration_limit):
        if should_stop and should_stop():
            on_event("error", {"message": "Stopped by user."})
            break
        on_event("iteration", {"index": iteration})
        request_conversation = _compact_apple_conversation(conversation)
        raw_decision = _apple_guided_request(
            request_conversation, system_prompt, "muse_decision",
            APPLE_DECISION_SCHEMA, base_url)
        try:
            decision = _normalize_apple_decision(raw_decision)
        except AgentError as first_error:
            raw_decision = _apple_guided_request(
                request_conversation, system_prompt, "muse_decision",
                APPLE_DECISION_SCHEMA, base_url,
                extra_prompt=(
                    "The previous decision was invalid: %s Return a clean "
                    "replacement with action exactly tool or respond, and an "
                    "exact Muse tool name when action is tool." % first_error))
            decision = _normalize_apple_decision(raw_decision)
        # urllib cannot cancel an in-flight request safely from another thread;
        # honour Escape immediately after it returns and, critically, before a
        # generated stage mutation is dispatched.
        if should_stop and should_stop():
            on_event("error", {"message": "Stopped by user."})
            break

        if decision["action"] == "respond":
            text = decision["message"]
            on_event("text", {"text": text})
            conversation = conversation + [{
                "role": "assistant",
                "content": [{"type": "text", "text": text}],
            }]
            break

        name = decision["tool"]
        argument_schema = APPLE_TOOL_ARGUMENT_SCHEMAS[name]
        argument_prompt = (
            "The next action is already locked to the %s tool. Fill only its "
            "argument schema. Do not select another tool and do not answer the "
            "user. %s"
            % (name, (
                "For run_python, all pxr names are already bound: use `stage`, "
                "`Usd`, `UsdGeom`, `Sdf`, `Gf`, and `Vt` directly. Do not import "
                "anything, assign to `stage`, call GetActiveStage, or open/create "
                "a stage. Define schemas exactly as "
                "`sphere = UsdGeom.Sphere.Define(stage, \"/World/Ball\")`; set "
                "and read attributes exactly as "
                "`sphere.GetRadiusAttr().Set(3.0)` and "
                "`sphere.GetRadiusAttr().Get()`. Substitute the requested schema, "
                "path, and values, and print exact verification values. The code "
                "must run verbatim in that pre-bound namespace."
                if name == "run_python" else
                "For inspect_stage, use mode summary, list, or prim; use empty "
                "strings and limit 0 for unused filters."
                if name == "inspect_stage" else
                "Describe briefly why the viewport is being captured.")))
        raw_arguments = _apple_guided_request(
            request_conversation, system_prompt, "muse_%s_arguments" % name,
            argument_schema, base_url, extra_prompt=argument_prompt)
        try:
            arguments = _normalize_apple_tool_arguments(name, raw_arguments)
        except AgentError as first_error:
            raw_arguments = _apple_guided_request(
                request_conversation, system_prompt, "muse_%s_arguments" % name,
                argument_schema, base_url,
                extra_prompt=(
                    "%s The previous arguments were invalid: %s Return a "
                    "correct replacement for this same tool."
                    % (argument_prompt, first_error)))
            arguments = _normalize_apple_tool_arguments(name, raw_arguments)
        if should_stop and should_stop():
            on_event("error", {"message": "Stopped by user."})
            break
        call_id = "call_muse_apple_%d" % iteration
        on_event("tool_use", {"name": name, "input": arguments})
        content, is_error = _dispatch_tool(
            executor, name, arguments, on_event)
        conversation = conversation + [
            {
                "role": "assistant",
                "content": [{
                    "type": "tool_use",
                    "id": call_id,
                    "name": name,
                    "input": arguments,
                }],
            },
            {
                "role": "user",
                "content": [{
                    "type": "tool_result",
                    "tool_use_id": call_id,
                    "content": content,
                    "is_error": is_error,
                }],
            },
        ]
    else:
        on_event("error", {
            "message": "Stopped after %d Apple FM tool rounds — the task did "
                       "not finish. Send another message to continue."
                       % iteration_limit})

    return conversation


def run_agent(messages, executor, system_prompt, on_event,
              model=None, max_tokens=None, effort=None,
              max_iterations=DEFAULT_MAX_ITERATIONS, should_stop=None):
    """
    Drive the tool loop until the model stops asking for tools.

    messages     — conversation so far, ending with the current user turn.
    executor     — object exposing run_python / inspect_stage / capture_viewport.
    on_event     — callback(kind, payload); kinds: thinking, text, tool_use,
                   tool_result, iteration, error.
    should_stop  — optional callable; when it returns True the loop unwinds.

    Returns the full message list including everything the assistant produced,
    so the caller can persist it as conversation history.
    """
    provider = resolve_provider()
    if provider == PROVIDER_APPLE:
        return _run_apple_agent(
            messages=messages, executor=executor, system_prompt=system_prompt,
            on_event=on_event, model=model, max_iterations=max_iterations,
            should_stop=should_stop)

    try:
        import anthropic
    except ImportError:
        raise AgentError(
            "The `anthropic` package is not installed in this Python. "
            "Install it into the interpreter running usdview: "
            "python -m pip install anthropic")

    api_key, _key_name = resolve_api_key()
    base_url, _base_name = resolve_base_url()
    problem = describe_key_problem(api_key, base_url, provider)
    if problem:
        raise AgentError(problem)
    if provider == PROVIDER_OLLAMA:
        # Explicit local providers must never receive a hosted credential that
        # happens to remain in the environment. The server ignores this; the
        # Anthropic SDK insists on a non-empty value.
        api_key = OLLAMA_PLACEHOLDER_KEY
        _key_name = "Ollama placeholder"
    elif provider == PROVIDER_LMSTUDIO:
        api_key = LMSTUDIO_PLACEHOLDER_KEY
        _key_name = "LM Studio placeholder"

    # auth_token= and api_key= are how the SDK is told which header to send;
    # sending Meta a key in x-api-key answers 401 no matter how good the key is.
    if resolve_auth_style(base_url) == "bearer":
        client_kwargs = {"auth_token": api_key}
    else:
        client_kwargs = {"api_key": api_key}
    if base_url:
        client_kwargs["base_url"] = base_url
    client = anthropic.Anthropic(**client_kwargs)
    # force_contributor_model again, not just inside resolve_model: an
    # explicit model= argument never passes through that function.
    model = force_contributor_model(
        model or resolve_model(base_url, provider), base_url)
    if provider == PROVIDER_LMSTUDIO and not model:
        raise AgentError(
            "LM Studio at %s did not list an LLM. Start the server on "
            "Hivemind with port 1234 exposed to the local network, then "
            "select a model in Muse settings." % base_url)
    # A model that cannot call tools cannot do anything Muse asks of it, and
    # the symptom -- fluent answers, untouched stage -- looks like the
    # assistant working. Fail here instead, while the reason is still legible.
    model_problem = describe_model_problem(model, provider)
    if model_problem:
        raise AgentError(model_problem)
    max_tokens = max_tokens or int(os.environ.get("MUSE_MAX_TOKENS", DEFAULT_MAX_TOKENS))
    effort = effort or os.environ.get("MUSE_EFFORT", DEFAULT_EFFORT)

    system_extras, conversation = normalize_messages(messages)
    if system_extras:
        system_prompt = system_prompt + "\n\nSession notes:\n" + "\n".join(system_extras)

    if not conversation:
        raise AgentError("Nothing to send — the conversation has no user turn.")

    _degraded = False
    for iteration in range(max_iterations):
        if should_stop and should_stop():
            on_event("error", {"message": "Stopped by user."})
            break
        on_event("iteration", {"index": iteration})

        request = {
            "model": model,
            "max_tokens": max_tokens,
            "system": system_prompt,
            "messages": conversation,
            "tools": TOOL_SCHEMAS,
        }
        if not _degraded:
            request["thinking"] = {"type": "adaptive", "display": "summarized"}
            request["output_config"] = {"effort": effort}

        try:
            with client.messages.stream(**request) as stream:
                response = stream.get_final_message()
        except Exception as exc:
            # Adaptive thinking and effort are model-gated. Rather than fail the
            # session on an older model or SDK, drop them once and carry on —
            # the tool loop is what matters and it works without them.
            if not _degraded and _is_parameter_rejection(exc):
                _degraded = True
                on_event("error", {
                    "message": "This model rejected adaptive thinking / effort "
                               "(%s). Continuing without them." % exc})
                continue
            authFailure = describe_auth_failure(
                exc, api_key, _key_name, base_url, _base_name)
            if authFailure:
                if provider == PROVIDER_LMSTUDIO:
                    raise AgentError(
                        "LM Studio at %s requires authentication, but the "
                        "Hivemind entry is configured as a keyless LAN "
                        "service. Disable Require Authentication in LM Studio "
                        "or configure a separate authenticated gateway."
                        % base_url)
                raise AgentError(authFailure)
            raise AgentError("%s: %s" % (type(exc).__name__, exc))

        if getattr(response, "stop_reason", None) == "refusal":
            details = getattr(response, "stop_details", None)
            on_event("error", {
                "message": "The model declined this request%s." % (
                    " (%s)" % details.category if details and getattr(details, "category", None) else "")})
            break

        for block in response.content:
            kind = getattr(block, "type", None)
            if kind == "thinking":
                text = getattr(block, "thinking", "") or ""
                if text:
                    on_event("thinking", {"text": text})
            elif kind == "text":
                on_event("text", {"text": block.text})

        conversation = conversation + [{"role": "assistant", "content": response.content}]

        tool_uses = [b for b in response.content if getattr(b, "type", None) == "tool_use"]
        if not tool_uses:
            break

        results = []
        for call in tool_uses:
            on_event("tool_use", {"name": call.name, "input": call.input})
            content, is_error = _dispatch_tool(executor, call.name, call.input, on_event)
            results.append({
                "type": "tool_result",
                "tool_use_id": call.id,
                "content": content,
                "is_error": is_error,
            })
        conversation = conversation + [{"role": "user", "content": results}]
    else:
        on_event("error", {
            "message": "Stopped after %d tool rounds — the task did not finish. "
                       "Send another message to continue." % max_iterations})

    return conversation


def _is_parameter_rejection(exc):
    """True when the API refused the request over thinking/effort specifically."""
    if getattr(exc, "status_code", None) not in (400, None):
        return False
    text = str(exc).lower()
    if "400" not in text and getattr(exc, "status_code", None) != 400:
        return False
    return any(token in text for token in
               ("thinking", "output_config", "effort", "adaptive", "budget_tokens"))


def _dispatch_tool(executor, name, arguments, on_event):
    """Run one tool call. Returns (tool_result content, is_error)."""
    arguments = arguments or {}
    try:
        if name == "run_python":
            code = arguments.get("code", "")
            outcome = executor.run_python(code)
            on_event("tool_result", {"name": name, "summary": outcome.get("summary", "")})
            return _text_result(outcome.get("text", "")), bool(outcome.get("failed"))

        if name == "inspect_stage":
            outcome = executor.inspect_stage(
                mode=arguments.get("mode", "summary"),
                pattern=arguments.get("pattern"),
                type_name=arguments.get("type_name"),
                limit=arguments.get("limit"),
                prim_path=arguments.get("prim_path"),
            )
            on_event("tool_result", {"name": name, "summary": outcome.get("summary", "")})
            return _text_result(outcome.get("text", "")), False

        if name == "capture_viewport":
            outcome = executor.capture_viewport(note=arguments.get("note", ""))
            on_event("tool_result", {"name": name, "summary": outcome.get("summary", ""),
                                     "image_b64": outcome.get("image_b64")})
            blocks = [{"type": "text", "text": outcome.get("text", "")}]
            if outcome.get("image_b64"):
                blocks.append({
                    "type": "image",
                    "source": {"type": "base64", "media_type": "image/png",
                               "data": outcome["image_b64"]},
                })
            return blocks, bool(outcome.get("failed"))

        return _text_result("Unknown tool: %s" % name), True
    except Exception as exc:
        message = "%s raised %s: %s" % (name, type(exc).__name__, exc)
        on_event("tool_result", {"name": name, "summary": message})
        return _text_result(message), True


def _text_result(text):
    return [{"type": "text", "text": text if text else "(no output)"}]
