#
# Opt-in LIVE verification: the real panel, the real model, the real stage.
#
# Everything else in the suite scripts the SDK, which proves the plumbing but
# cannot prove the thing the user actually cares about — that asking for a
# change in English results in the stage changing. This one makes real API
# calls through the configured provider (including local Apple FM or Ollama)
# and asserts the outcome by reading the stage back.
#
# Run:  MUSE_LIVE=1 bin/test_muse.sh
#
import os
import time

TIMEOUT_SECONDS = 300.0


def _pump(panel, predicate, what):
    from pxr.Usdviewq.qt import QtWidgets
    deadline = time.time() + TIMEOUT_SECONDS
    while time.time() < deadline:
        QtWidgets.QApplication.processEvents()
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("timed out after %gs waiting for %s" % (TIMEOUT_SECONDS, what))


def _idle(panel):
    return panel._agent_thread is None or not panel._agent_thread.is_alive()


def testUsdviewInputFunction(appController):
    import museAgent
    import museAssistant

    api = appController._usdviewApi
    stage = api.stage

    provider = museAgent.resolve_provider()
    key, key_name = museAgent.resolve_api_key()
    # Local Apple FM and Ollama authenticate nothing, so a missing key is only
    # fatal for the hosted back ends.
    if not key and provider not in (museAgent.PROVIDER_OLLAMA,
                                    museAgent.PROVIDER_APPLE):
        raise AssertionError("MUSE_API_KEY is not set — nothing to test live")
    base_url, base_source = museAgent.resolve_base_url()
    model = museAgent.resolve_model(base_url, provider)
    print("LIVE: provider=%s key=%s base=%s (%s) model=%s"
          % (provider, key_name or "(none needed)",
             base_url or "api.anthropic.com", base_source or "default", model))

    problem = museAgent.describe_model_problem(model, provider)
    if problem:
        raise AssertionError("this back end cannot run the test: %s" % problem)

    panel = museAssistant.MuseChatPopup.GetInstance(api)
    panel._report_backend_readiness()

    target = "/MuseLive/Ball"
    panel._input.setPlainText(
        "Create a UsdGeom Sphere at exactly %s with radius 3.0, then read the "
        "radius back and tell me what it is." % target)
    panel._on_send()
    if panel._agent_thread is None:
        raise AssertionError("Send did not start the assistant")
    _pump(panel, lambda: _idle(panel), "the live model to finish")

    transcript = panel._transcript.toPlainText()
    prim = stage.GetPrimAtPath(target)
    if not prim or not prim.IsValid():
        raise AssertionError(
            "the live model did not create %s.\n--- transcript ---\n%s"
            % (target, transcript[-3000:]))
    radius = prim.GetAttribute("radius").Get()
    if radius != 3.0:
        raise AssertionError("radius is %r, expected 3.0" % radius)

    print("LIVE: transcript tail ---\n%s\n---" % transcript[-1200:])
    print("MUSE_LIVE_OK  %s created with radius %s by %s" % (target, radius, model))
