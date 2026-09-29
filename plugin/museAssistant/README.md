# Muse Assistant for usdview

Muse is an optional assistant panel that can inspect the loaded stage, run
Python inside usdview, and capture the viewport. `museAssistant.py` owns the
Qt panel and main-thread execution; `museAgent.py` handles provider requests,
tool calls, and image metadata.

## Setup

Use the Python interpreter that launches usdview. For Messages API providers,
install `anthropic` in that environment. Add this plugin directory to
`PXR_PLUGINPATH_NAME` if your launcher does not already do so.

Choose a provider in **Muse → Settings** or through `MUSE_PROVIDER`. Hosted
providers require credentials; local providers require a running compatible
server. Configure the model explicitly when your endpoint needs one.

| Provider | Environment | Default endpoint |
|---|---|---|
| Anthropic | `MUSE_PROVIDER=anthropic`, `MUSE_API_KEY` or `ANTHROPIC_API_KEY` | `https://api.anthropic.com` |
| Meta | A supported Meta key and `MUSE_MODEL` | `https://api.meta.ai` |
| Ollama | `MUSE_PROVIDER=ollama`, optional `MUSE_OLLAMA_URL` | `http://127.0.0.1:11434` |
| LM Studio | `MUSE_PROVIDER=lmstudio`, optional `MUSE_LMSTUDIO_URL` | `http://127.0.0.1:1234` |
| Apple local server | `MUSE_PROVIDER=apple`, optional `MUSE_APPLE_URL` | `http://127.0.0.1:1976` |

For a local LM Studio server:

```sh
export MUSE_PROVIDER=lmstudio
export MUSE_LMSTUDIO_URL=http://127.0.0.1:1234
export MUSE_MODEL=your-installed-model
bin/usdview.sh examples/ArmShotAnim.usda
```

Use `set NAME=value` in Windows Command Prompt. Set the endpoint explicitly
for a remote server. A model must support tool calls to operate the stage.
Local provider requests do not forward hosted-service credentials. The Apple
adapter requests the local `system` model and requires a compatible server.

## Configuration

- `MUSE_MODEL`: model identifier; local server discovery can select a default.
- `MUSE_BASE_URL`: custom Messages API endpoint.
- `MUSE_MAX_TOKENS`: per-response output limit.
- `MUSE_EFFORT`: provider-dependent reasoning setting.
- `MUSE_API_KEY`: hosted provider credential; keep it out of source control.

Settings can be saved locally by the panel. Credentials, stage text, prompts,
and viewport captures may be sent to the selected endpoint when requested by
the tool loop. Select an endpoint appropriate for the stage you are editing.
The panel runs Python with the permissions of the usdview process.

## Verification

`tests/testMuseAgent.py` checks routing, tool results, error handling, and image
metadata using local test doubles. Run it with the project's USD environment.
`tests/testUsdviewMuse.py` exercises the panel in a graphical usdview session.
Provider availability and model identifiers are service configuration, not
a guarantee made by this repository.
