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

# Back ends.  All three speak the Anthropic Messages API, so one client and one
# tool loop serve them all — only the base URL, the auth header and the model
# id differ.
#
#   Anthropic   api.anthropic.com   sk-ant-…   x-api-key       claude-opus-5
#   Meta Muse   api.meta.ai         LLM_…      Bearer token    muse-spark-1.2-contributor
#   Ollama      <host>:11434        (none)     (ignored)       qwen3.5:9b
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

# Meta publishes a standard tier (muse-spark-1.1, muse-spark-1.2) and a
# contributor tier. Every call to Meta goes through the contributor tier — see
# force_contributor_model, which is why MUSE_MODEL cannot drop back to the
# standard one.
META_DEFAULT_MODEL = "muse-spark-1.2-contributor"
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
PROVIDERS = (PROVIDER_ANTHROPIC, PROVIDER_META, PROVIDER_OLLAMA)

# A starting point for the settings dialog, not a fallback the resolver
# reaches for: nothing routes to Ollama unless the provider is set to it.
OLLAMA_DEFAULT_BASE_URL = "http://192.168.68.75:11434"

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

    MUSE_PROVIDER states it outright; that is the only way to select Ollama,
    because an Ollama server has no recognisable address. Without it the
    answer is inferred exactly the way it always was, so a session that never
    heard of providers behaves identically.
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


def resolve_base_url():
    """
    Which Messages API endpoint to talk to.

    An explicit MUSE_BASE_URL always wins.  Otherwise the provider decides: a
    Meta Muse key (`LLM_…`) selects Meta's endpoint, because
    api.anthropic.com answers that key shape with 401 and routing it to
    Anthropic could only ever fail; MUSE_PROVIDER=ollama selects the local
    server.
    """
    value, name = _env_base_url()
    if value:
        return value, name

    provider = resolve_provider()
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
    appended to it: only muse-spark-1.2-contributor is published, so
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
    explicit = os.environ.get("MUSE_MODEL", "").strip()
    if explicit:
        return force_contributor_model(explicit, base_url)
    if base_url and META_HOST in base_url:
        return META_DEFAULT_MODEL
    if (provider or resolve_provider()) == PROVIDER_OLLAMA:
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
    if (provider or resolve_provider()) == PROVIDER_OLLAMA:
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

    Judged from /api/show, never from the model name or the /api/tags summary
    -- see fetch_ollama_models for why the latter cannot be trusted.

    Returns None for the hosted back ends: their model ids are fixed and their
    capabilities are not ours to enumerate.
    """
    if (provider or resolve_provider()) != PROVIDER_OLLAMA:
        return None
    if not model:
        return None
    if models is None:
        models = fetch_ollama_models()
    if not models:
        return None  # server unreachable; not the model's fault

    known = {entry["name"]: entry for entry in models}
    entry = known.get(model)
    if entry is None:
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
    try:
        import anthropic
    except ImportError:
        raise AgentError(
            "The `anthropic` package is not installed in this Python. "
            "Install it into the interpreter running usdview: "
            "python -m pip install anthropic")

    provider = resolve_provider()
    api_key, _key_name = resolve_api_key()
    base_url, _base_name = resolve_base_url()
    problem = describe_key_problem(api_key, base_url, provider)
    if problem:
        raise AgentError(problem)
    if provider == PROVIDER_OLLAMA and not api_key:
        # The server ignores it; the SDK insists on one.
        api_key = OLLAMA_PLACEHOLDER_KEY

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
