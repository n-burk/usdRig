# Muse Assistant — usdview Plugin

An assistant embedded in usdview that **works your live session** rather than
suggesting things you then paste. It inspects the stage, runs Python in the
usdview process, captures the viewport to look at its own results, and keeps
going until the task is done.

## Location

```
plugin/museAssistant/
  plugInfo.json        — PluginContainer registration (Type: python)
  museAssistant.py     — chat window, usdview integration, main-thread marshalling
  museAgent.py         — agent core: message shaping, tool loop, image metadata
```

`museAgent.py` has no Qt or usdview dependency, so the parts that historically
broke silently are testable without a display.

## Setup

Five back ends are supported. The four Anthropic Messages providers require
the Anthropic SDK in **the interpreter that launches usdview**; only the two
hosted providers also require an API key:

```bash
python -m pip install anthropic
export MUSE_API_KEY=...               # or ANTHROPIC_API_KEY
```

Apple Foundation Models, Ollama and LM Studio are keyless providers selected in
**Muse ▸ Settings…** or with `MUSE_PROVIDER`. Apple uses the Chat Completions
endpoint built into `fm serve`; the other four use the Anthropic Messages
protocol. LM Studio and Ollama still need the SDK even though they need no real
credential.

| Key looks like | Endpoint | Auth header | Default model |
|---|---|---|---|
| `LLM_…` (Meta Muse) | `https://api.meta.ai` | `Authorization: Bearer` | `muse-spark-1.3-contributor` |
| `sk-ant-…` (Anthropic) | `https://api.anthropic.com` | `x-api-key` | `claude-opus-5` |
| *(none)* — **Ollama** | `http://127.0.0.1:11434` by default | none | `qwen3.5:9b` |
| *(none)* — **LM Studio** | `http://hivemind.local:1234` by default | inert `lmstudio` placeholder only | `MUSE_MODEL`, or selected from the server |
| *(none)* — **Apple FM** | `http://127.0.0.1:1976` | none | `system` (on-device) |

### Apple Foundation Models (local, on-device)

Accept the Apple Foundation Models license once, start the system server in a
separate Terminal, then select Apple in Muse:

```bash
fm serve --host 127.0.0.1 --port 1976

export MUSE_PROVIDER=apple
export MUSE_APPLE_URL=http://127.0.0.1:1976   # optional; this is the default
```

Muse checks `/health` and requires `system` to report `available: true`. It
always sends `model: "system"`: `MUSE_MODEL=pcc` and an explicit `model="pcc"`
are ignored, so this route never silently leaves the device.

On the current FoundationModels 2.0.68 server, text and inline viewport images
work, but the model does not emit usable first-step OpenAI `tool_calls`; forced
tool choice returns HTTP 500 and streamed tool use can hang. Muse therefore
uses two small JSON-schema calls: one selects the next tool, and the second
fills only that tool's arguments. It executes the existing `run_python` /
`inspect_stage` / `capture_viewport` tool, feeds the real result back, and
repeats. Calls are non-streaming with a bounded timeout (`MUSE_APPLE_TIMEOUT`,
default 120 seconds).

### LM Studio on Hivemind (local network, no key)

On the Hivemind machine, start LM Studio's local server on port 1234 and enable
**Serve on Local Network** in LM Studio's server settings. Then select
**LM Studio (Hivemind)** in Muse, or launch with:

```bash
export MUSE_PROVIDER=lmstudio
export MUSE_LMSTUDIO_URL=http://hivemind.local:1234   # optional; this is the default
export MUSE_MODEL=qwen3.8-27b@q4_k_m                  # optional
```

The bare `hivemind` alias does not resolve from this Mac. Use the default
Bonjour name above; if Bonjour is unavailable, set
`MUSE_LMSTUDIO_URL=http://192.168.68.54:1234` as the current direct-IP fallback.

Muse discovers LLMs through LM Studio's native `/api/v1/models` inventory. If
`MUSE_MODEL` is unset, it prefers an already-loaded model, then native tool
support and vision; choosing a model in **Muse ▸ Settings…** saves the same
value. Requests go to the Anthropic-compatible `/v1/messages` endpoint, so the
`anthropic` Python package is still required.

LM Studio itself requires no API key. Muse gives the SDK the inert local
placeholder `lmstudio` and never forwards a hosted `MUSE_API_KEY` or
`ANTHROPIC_API_KEY` to Hivemind.

The default endpoint is currently live and has been verified with the loaded
`qwen3.8-27b@q4_k_m` model, including native tool use and vision. Availability
still depends on Hivemind being reachable, LM Studio's server running, and
**Serve on Local Network** remaining enabled. `bin/launch.sh` probes it directly
without using a configured HTTP proxy and reports that state before usdview
opens.

### Ollama (local, no key)

Ollama serves `/v1/messages` in Anthropic format — streaming SSE, `tool_use`
blocks and `thinking` blocks included — so it needs no translation layer and no
credential. Verified against a live server: a tools request comes back with
`stop_reason: "tool_use"` and a correctly-typed block, and
`tests/testUsdviewMuseLive.py` creates a prim on the real stage through it.

Unlike the hosted back ends, Ollama is **selected explicitly** rather than
inferred. A server lives at whatever address you run it on, so there is nothing
to recognise, and guessing "this unfamiliar host is probably local" would be a
guess that sends an Anthropic key somewhere it should never go. Pick it in
**Muse ▸ Settings…**, or:

```bash
export MUSE_PROVIDER=ollama
export MUSE_OLLAMA_URL=http://127.0.0.1:11434   # optional; this is the default
export MUSE_MODEL=qwen3.5:9b                        # optional
```

**The model must support tools.** Muse does not answer questions about a stage,
it edits one, entirely through tool calls — so a completion-only model streams a
fluent reply and changes nothing, which reads as the assistant working. The
settings dialog labels each model, and choosing one without tools is refused
with the list of models that do have them.

Capabilities are read from `/api/show`, one call per model, **not** from the
summary in `/api/tags` — the two disagree. On a live server `/api/tags`
reported `completion` only for four of ten models (every user-namespaced one:
`smtek/…`, `kwangsuklee/…`, `zfujicute/…`) while `/api/show` reported `tools`
for all four, and a real tools request to one came back with
`stop_reason: "tool_use"`. Trusting `/api/tags` refuses working models, which
for a local server is the worst direction to be wrong in. Ten models cost about
0.4s, so the list stays a click rather than a wait.

`qwen3.5:9b` is the default because it is the only one that also does **vision**,
and Muse sends viewport screenshots. Bigger tool-capable models work too —
`smtek/Qwen3.8-27B` (27B, 262k context) is verified end-to-end by
`tests/testUsdviewMuseLive.py`, it just cannot see the screenshots.

Meta documents the Messages API at `https://api.meta.ai/v1` for Anthropic-format
harnesses ([dev.meta.ai/docs/overview](https://dev.meta.ai/docs/overview/));
streaming, tool use, tool results and image input are all verified working there
by `tests/testUsdviewMuseLive.py`.

**The auth header differs between the two, and this is not cosmetic.** Meta
documents a bearer token (`MODEL_API_KEY`, or `ANTHROPIC_AUTH_TOKEN` for Claude
Code); Anthropic takes `x-api-key`. The SDK picks the header from which argument
it is handed — `api_key=` sends `X-Api-Key`, `auth_token=` sends
`Authorization: Bearer` — so a perfectly valid Meta key passed as `api_key=`
comes back `401 Unauthorized`, indistinguishable from a dead key. `resolve_auth_style()`
keys this off the endpoint rather than the key, because the scheme belongs to
the service. A `MUSE_BASE_URL` gateway is a third service whose scheme we do not
know, so it keeps `x-api-key`.

Meta publishes a standard tier (`muse-spark-1.1`, `muse-spark-1.2`, `muse-spark-1.3`) and a
contributor tier. **Every Meta call goes through `muse-spark-1.3-contributor`.**
`force_contributor_model()` is applied to both routes a model can arrive by —
`MUSE_MODEL` and an explicit `model=` argument — so `MUSE_MODEL` can select a
different *contributor* model but cannot drop back to the standard tier; a
standard id is replaced and the substitution is reported on stderr. A
standard-tier id is swapped outright rather than having `-contributor`
appended, because only `muse-spark-1.3-contributor` is published and a
synthesised `muse-spark-1.1-contributor` would fail at the API instead of
here. Anthropic and `MUSE_BASE_URL` gateways are untouched — the tier is a
Meta concept.

`bin/launch.sh` prints the selected provider's state at startup, and the window
says so in its transcript when no key is set. Optional overrides:

| Variable | Default | Meaning |
|---|---|---|
| `MUSE_MODEL` | `claude-opus-5`, `muse-spark-1.3-contributor` on Meta, or server-selected on LM Studio | Model id. Ignored by Apple, which is fixed to on-device `system` |
| `MUSE_EFFORT` | `xhigh` | `low` / `medium` / `high` / `xhigh` / `max` |
| `MUSE_MAX_TOKENS` | `32000` | Output cap per turn |
| `MUSE_BASE_URL` | — | An Anthropic-compatible endpoint other than `api.anthropic.com` (a gateway, proxy, or local relay). Ignored by the explicit Apple and LM Studio providers |
| `MUSE_PROVIDER` | inferred from the key | `anthropic`, `meta`, `ollama`, `lmstudio`, or `apple`. Local providers are selected explicitly |
| `MUSE_OLLAMA_URL` | `http://127.0.0.1:11434` | Ollama server address, used when `MUSE_PROVIDER=ollama` |
| `MUSE_LMSTUDIO_URL` | `http://hivemind.local:1234` | LM Studio server address, used when `MUSE_PROVIDER=lmstudio` |
| `MUSE_APPLE_URL` | `http://127.0.0.1:1976` | `fm serve` address, used when `MUSE_PROVIDER=apple` |
| `MUSE_APPLE_TIMEOUT` | `120` | Seconds allowed for one local Apple completion |
| `MUSE_APPLE_MAX_ITERATIONS` | `12` | Maximum Apple action rounds per send |

### On keys and endpoints

`api.anthropic.com` accepts `sk-ant-…` and answers anything else with
`401 invalid x-api-key`, so a Meta Muse key sent there could only ever fail.
The plugin therefore routes `LLM_…` keys to `https://api.meta.ai` on its own,
and sends them as a bearer token once there.

`MUSE_BASE_URL` overrides this for any other Messages-API-compatible endpoint,
and a key with an unrecognised prefix and no endpoint is refused up front with
that explanation rather than being sent somewhere it will 401.

A 401 from Meta with the routing and the header both correct means the token
itself is rejected — expired, revoked, or issued for another service. The tell
is that an identical request carrying **no key at all** gets the same response,
so there is nothing to fix on this side but the key.

An earlier build handled `LLM_…` keys with `_call_custom_muse()`, a stub that
returned canned text **without contacting any model** — Muse replied fluently
and changed nothing. That stub is gone.

If a hosted provider has no key, or a local provider is not ready, the window
reports that state and does not fabricate a fallback answer.

## The window

One window, opened with **Cmd+Shift+M** (Ctrl+Shift+M off macOS), or from the
**Muse** menu. The same chord closes it again.

```
┌ Muse  ⠿ ───────────────────────────┐   <- drag here
│ you   delete the selected prim     │
│ muse  deleted /World/Cube          │
│ you   now add a sphere             │
│ muse  …                            │
├────────────────────────────────────┤
│ > _                        [📷]    │
└────────────────────────────────────┘
  ⏎ send · ⇧⏎ newline · esc close
```

The window is frameless, so the header strip is its title bar: drag it (or the
status line, or the margins) to move the window. It centres on the usdview
window the first time only — once you place it, it stays there across opens.
Presses inside the transcript or the input are hit-tested out of the drag, so
moving the window never competes with selecting text.

`⏎` sends, `⇧⏎` is a newline, and **esc** closes the window when idle or stops
the agent when it is working — one key for both, so there is a brake on a
forty-step tool loop without a button for one.

**↑/↓ recall earlier prompts**, filtered by what is already typed: `add a` then
`↑` offers only the prompts that began that way, newest first. An empty box
matches everything, so `↑` is plain "previous prompt". Coming back down past
the newest match restores the draft you were typing, and editing restarts the
search on the new text. On a multi-line draft the arrows keep their usual
meaning — recall only fires from the first line going up and the last going
down. History lasts for the session.

**📷** attaches what the viewport currently shows, together with the camera
that produced it, to your next message. Attach several before sending; they go
in order and each carries its own camera. They clear once sent.

Plain `Cmd+M` is macOS's Minimize Window shortcut, so it is deliberately left
alone.

### Settings

**File ▸ Muse ▸ Settings…** (also in the top-level **Muse** menu) chooses the
back end and shows where requests will be sent:

```
Back end  [ Anthropic / Meta Muse (from the key) ▾ ]
API key   ••••••••••••••••••••
          In use: MUSE_API_KEY, 48 chars, ends T3ST
☐ Remember on this machine (~/.config/muse/credentials.json, readable only by you)

Endpoint  https://api.meta.ai        (Meta Muse key)
Header    Authorization: Bearer
Model     muse-spark-1.3-contributor
```

For the hosted back ends the endpoint, header and model are read-only because
they are derived from the key rather than chosen — seeing them is what turns a
bare `401 Unauthorized` into a fact about the key rather than a mystery.

Choosing **Ollama (local server)** swaps the key field for the server address
and a model list read from the server itself, each entry labelled with what it
can do:

```
Back end  [ Ollama (local server) ▾ ]
Ollama server  http://127.0.0.1:11434
Model     [ qwen3.5:9b   [tools, vision] ▾ ]  [Refresh]
          10 model(s) on this server, 6 can call tools.

Endpoint  http://127.0.0.1:11434/v1/messages
Header    none — Ollama authenticates nothing
Model     qwen3.5:9b
```

Choosing **LM Studio (Hivemind)** similarly shows Hivemind's server address and
native model inventory. The model list marks loaded models, native tool support
and vision; an explicit selection is saved as `MUSE_MODEL`:

```
Back end  [ LM Studio (Hivemind) ▾ ]
LM Studio server  http://hivemind.local:1234
Model     [ Qwen3.8 27B UD   [loaded, native tools, vision] ▾ ]  [Refresh]
          4 model(s) on Hivemind, 4 with native tool support.

Endpoint  http://hivemind.local:1234/v1/messages
Header    LM Studio placeholder — hosted keys are never sent
Model     qwen3.8-27b@q4_k_m
```

Choosing **Apple Foundation Models (on-device)** shows only the local server.
The model is fixed and there is no credential or model picker:

```
Back end  [ Apple Foundation Models (on-device) ▾ ]
fm serve  http://127.0.0.1:1976
          Ready — system model available on-device.

Endpoint  http://127.0.0.1:1976/v1/chat/completions
Header    none — loopback only
Model     system (on-device)
```

Everything here is stored beside the key in `~/.config/muse/credentials.json`
(0600) under its environment-variable name, so what you would export in a shell
is what appears in the file. The shell still wins: an export is the more
explicit statement of intent, and a stale saved setting quietly overriding it is
the kind of thing you debug for an hour.

Both menu items are pinned to `NoRole`. Qt defaults actions to
`TextHeuristicRole`, which lets the macOS native menu bar scan the text and
lift anything reading like "Settings" or "Preferences" into the application
menu beside the Apple logo — the same reason usdview's own `File ▸ Quit` is
not where you would expect it there. Without the pin, `Settings…` silently
leaves the Muse menu on macOS.

Unchecked, the key lives in the process and is gone at exit. Checked, it is
written to `~/.config/muse/credentials.json` with mode `0600` and reapplied at
startup — but **only when the shell supplied none**, so `MUSE_API_KEY` in the
launching environment always wins and a stale saved key can never quietly
override it.

## What it can do

The assistant drives the session through three tools. Everything it does is
visible in the transcript as it happens.

| Tool | What it gives the assistant |
|---|---|
| `run_python` | Executes Python in the usdview process. `stage`, `api`, `app`, `Usd`, `UsdGeom`, `Sdf`, `Gf`, `Vt`, `Tf`, `QtCore`, `QtGui`, `QtWidgets` are pre-bound. Stage edits, selection, timeline, view settings, and building or restyling Qt widgets in the running window all go through it. stdout comes back to the model. |
| `inspect_stage` | Cheap structured reads: stage summary, filtered prim listing, single-prim detail. |
| `capture_viewport` | Renders the current viewport back to the model as an image, with the camera matrices, eye position, fov and frame that produced it. |

Because results feed back, it can do the loop you would do by hand: read the
prim, edit it, capture the viewport, notice the edit missed, fix it.

**Stage edits are always allowed.** The old "Allow stage edits" checkbox went
with the panel that held it — the window is one input and one button by
design, so `run_python` is unconditional and undo is the brake.

## Architecture notes

The model call runs on a worker thread, so usdview stays responsive while the
assistant works. USD and Qt are both main-thread-only, so every tool call and
every transcript update is marshalled back with `QCoreApplication.postEvent`,
which is thread-safe and does not depend on PySide-vs-PyQt signal spelling.

Message construction is normalized before every request: `system`-role entries
are lifted into the system prompt, consecutive same-role turns are merged, and
a leading assistant turn is dropped. This is not cosmetic — the Messages API
rejects all three, and a chat panel produces all three naturally.

The canonical history stays in that Messages-shaped form for every provider.
The Apple adapter translates it at the boundary: system prompt to a `system`
message, `tool_use` to synthetic OpenAI `tool_calls`, results to `role: tool`,
and image blocks to inline `image_url` data URLs.

## Verification

```
bin/test_muse.sh
```

Neither default test contacts a model — both script their transport — so no
API key or local server is used.

* `tests/testMuseAgent.py` — headless: message shaping against both API forms,
  the tool loop, Apple health/model enforcement, tool and inline-image
  conversion, tool-error propagation, graceful degrade when a model rejects
  adaptive thinking, and PNG camera-metadata round-trips.
`MUSE_LIVE=1 bin/test_muse.sh` additionally runs
`tests/testUsdviewMuseLive.py`, which makes **real** API calls with the
configured provider: it asks in English for a sphere at a given path and
radius, then asserts the prim exists on the stage with that radius. It is the
only test that proves the whole chain, so it is opt-in rather than default.

* `tests/testUsdviewMuse.py` — inside a real usdview: the window constructs, the
  assistant's edits actually land on the stage (asserted by reading the prims
  back), tool calls marshal across threads without deadlocking, the viewport
  capture is a real image, an attached screenshot's camera and annotation reach
  the model, it can add a dock widget and drive view settings, and read-only
  mode genuinely refuses edits.

## Programmatic use (no Qt required)

```python
import museAgent

extras, messages = museAgent.normalize_messages(raw)
system = museAgent.build_system_prompt(stage_context="…", goal="…")

item = museAgent.Attachment.from_file("shot.png")
item.has_camera(), item.camera, item.to_context_block(0)

png = museAgent.png_write_text(png_bytes, {museAgent.CAMERA_METADATA_KEY: json.dumps(cam)})
museAgent.png_read_text(png)

museAgent.run_agent(messages, executor, system, on_event)   # executor supplies
                                                            # run_python /
                                                            # inspect_stage /
                                                            # capture_viewport
```
