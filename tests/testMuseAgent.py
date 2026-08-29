#
# Headless verification of the Muse agent core (no Qt, no display, no network).
#
# What is asserted here is everything that made the panel silently useless
# before: the message list handed to the Messages API must be one the API
# accepts (roles alternate, no `system` role inside messages, first turn is
# user, the real system prompt survives), the tool loop must actually run
# tools and feed their results back, and camera metadata must survive a
# round-trip through a PNG so a screenshot the user saves and re-sends still
# carries the camera that produced it.
#
# Run with the USD venv:
#   PYTHONPATH=<usd site-packages>:plugin/museAssistant python tests/testMuseAgent.py
#
import base64
import http.server
import json
import os
import pathlib
import sys
import threading
import types

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "plugin", "museAssistant"))

import museAgent as ma


_TEST_META_KEY = "LLM_" + "test-only-example-key"
_TEST_META_KEY_WITH_FINGERPRINT = _TEST_META_KEY + "T3ST"


# ---------------------------------------------------------------------------
# The validation the Messages API performs server-side.  Every request the
# agent builds must pass this or the panel is dead in the water.
# ---------------------------------------------------------------------------

def assertValidRequest(system, messages):
    if not messages:
        raise AssertionError("messages: at least one message is required")
    for index, message in enumerate(messages):
        if message.get("role") not in ("user", "assistant"):
            raise AssertionError(
                "messages.%d.role: must be 'user' or 'assistant', got %r"
                % (index, message.get("role")))
    if messages[0]["role"] != "user":
        raise AssertionError("messages: first message must use the 'user' role")
    for index in range(1, len(messages)):
        if messages[index]["role"] == messages[index - 1]["role"]:
            raise AssertionError(
                "messages: roles must alternate, found two %r turns in a row "
                "at index %d" % (messages[index]["role"], index))
    if not system or "You are Muse" not in system:
        raise AssertionError(
            "system prompt was lost or overwritten: %r" % (system[:120],))


def testMessagesSurviveARealisticSession():
    """
    The exact shape the panel produces: a goal was set (logged as a system
    entry), an exchange happened, and the live prompt is already in history.
    Before the fix this yielded ['user','assistant','user','user'] with the
    system prompt clobbered by the goal notice.
    """
    history = [
        ("system", "Goal set: rig the arm to match the sketch"),
        ("user", "create a sphere"),
        ("assistant", "done"),
        ("system", "Stage replaced: /tmp/x.usda"),
        ("user", "move the sphere up 2 units"),
    ]
    raw = [{"role": role, "content": content} for role, content in history]
    extras, messages = ma.normalize_messages(raw)

    system = ma.build_system_prompt(stage_context="Stage: /tmp/x.usda",
                                    goal="rig the arm to match the sketch")
    if extras:
        system = system + "\n\nSession notes:\n" + "\n".join(extras)

    assertValidRequest(system, messages)

    roles = [m["role"] for m in messages]
    if roles != ["user", "assistant", "user"]:
        raise AssertionError("expected user/assistant/user, got %s" % roles)
    if "rig the arm to match the sketch" not in system:
        raise AssertionError("goal did not reach the system prompt")
    if "Goal set:" not in system:
        raise AssertionError("system-role history was dropped instead of lifted")


def testConsecutiveUserTurnsAreMerged():
    raw = [
        {"role": "user", "content": "first"},
        {"role": "user", "content": "second"},
    ]
    _, messages = ma.normalize_messages(raw)
    assertValidRequest(ma.build_system_prompt(), messages)
    if len(messages) != 1 or "first" not in messages[0]["content"] \
            or "second" not in messages[0]["content"]:
        raise AssertionError("consecutive user turns were not merged: %s" % messages)


def testLeadingAssistantTurnIsDropped():
    raw = [
        {"role": "assistant", "content": "unprompted"},
        {"role": "user", "content": "hello"},
    ]
    _, messages = ma.normalize_messages(raw)
    assertValidRequest(ma.build_system_prompt(), messages)


# ---------------------------------------------------------------------------
# The tool loop
# ---------------------------------------------------------------------------

class FakeExecutor(object):
    """Records what the agent asked the session to do."""

    def __init__(self):
        self.python_calls = []
        self.inspect_calls = []
        self.capture_calls = []

    def run_python(self, code):
        self.python_calls.append(code)
        return {"text": "Created /World/Ball", "summary": "ok", "failed": False}

    def inspect_stage(self, mode, pattern=None, type_name=None, limit=None, prim_path=None):
        self.inspect_calls.append(mode)
        return {"text": "Stage: /tmp/x.usda\nPrim count: 3", "summary": "read"}

    def capture_viewport(self, note=""):
        self.capture_calls.append(note)
        return {"text": "Viewport 800x600, camera freeCamera",
                "image_b64": base64.b64encode(b"fake-png").decode("ascii"),
                "summary": "captured", "failed": False}


def _block(kind, **kwargs):
    block = types.SimpleNamespace(type=kind)
    for key, value in kwargs.items():
        setattr(block, key, value)
    return block


def installFakeAnthropic(scripted_turns, captured_requests):
    """
    Replace the anthropic SDK with one that replays *scripted_turns* and
    records every request, so the loop can be exercised without a network or
    an API key.
    """
    class _Stream(object):
        def __init__(self, message):
            self._message = message

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def get_final_message(self):
            return self._message

    class _Messages(object):
        def stream(self, **kwargs):
            captured_requests.append(kwargs)
            index = len(captured_requests) - 1
            turn = scripted_turns[min(index, len(scripted_turns) - 1)]
            return _Stream(turn)

    class _Client(object):
        def __init__(self, api_key=None, **kwargs):
            self.messages = _Messages()

    module = types.ModuleType("anthropic")
    module.Anthropic = _Client
    sys.modules["anthropic"] = module


class FakeFmServer(object):
    """Small HTTP double for ``fm serve``'s health and completions routes."""

    def __init__(self, completions=None, system_available=True,
                 unavailable_reason="system model unavailable for test"):
        self.completions = list(completions or [])
        self.system_available = system_available
        self.unavailable_reason = unavailable_reason
        self.requests = []
        self.get_requests = []
        owner = self

        class _Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, _format, *args):
                pass

            def _send(self, status, payload):
                encoded = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def do_GET(self):
                owner.get_requests.append(self.path)
                if self.path != "/health":
                    self._send(404, {"error": "not found"})
                    return
                model = {"name": "system", "available": owner.system_available}
                if not owner.system_available:
                    model["reason"] = owner.unavailable_reason
                self._send(200, {
                    "status": "fm serve is running",
                    "models": [model, {"name": "pcc", "available": False}],
                })

            def do_POST(self):
                if self.path != "/v1/chat/completions":
                    self._send(404, {"error": "not found"})
                    return
                length = int(self.headers.get("Content-Length", "0"))
                owner.requests.append(json.loads(self.rfile.read(length)))
                if not owner.completions:
                    self._send(500, {"error": {"message": "no scripted turn"}})
                    return
                content = json.dumps(owner.completions.pop(0))
                self._send(200, {
                    "id": "chatcmpl-test",
                    "object": "chat.completion",
                    "model": "system",
                    "choices": [{
                        "index": 0,
                        "finish_reason": "stop",
                        "message": {"role": "assistant", "content": content},
                    }],
                })

        self._server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        self.url = "http://127.0.0.1:%d" % self._server.server_address[1]
        self._thread = threading.Thread(target=self._server.serve_forever)
        self._thread.daemon = True

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)


def _appleDecision(action, tool="", message=""):
    """The small first phase; selected-tool arguments are a second phase."""
    return {"action": action, "tool": tool, "message": message}


def testAppleFmLoopNeedsNoAnthropicAndConvertsToolsAndImages():
    """The Apple route is local, pinned to system, and runs Muse's real tools."""
    scripted = [
        _appleDecision("tool", "run_python", "Editing the stage."),
        {"code": "print('apple edit')"},
        _appleDecision("tool", "capture_viewport", "Checking the result."),
        {"note": "verify apple edit"},
        _appleDecision("respond", message="Edited and verified with Apple FM."),
    ]
    saved_anthropic = sys.modules.get("anthropic")
    had_anthropic = "anthropic" in sys.modules
    with FakeFmServer(scripted) as server:
        for name in ("MUSE_API_KEY", "ANTHROPIC_API_KEY", "MUSE_BASE_URL",
                     "ANTHROPIC_BASE_URL", "MUSE_MODEL"):
            os.environ.pop(name, None)
        os.environ["MUSE_PROVIDER"] = "apple"
        os.environ["MUSE_APPLE_URL"] = server.url + "/"
        os.environ["MUSE_BASE_URL"] = "https://must-not-receive-apple-data.example"
        os.environ["MUSE_MODEL"] = "pcc"
        # ``None`` makes an accidental ``import anthropic`` fail immediately.
        sys.modules["anthropic"] = None
        try:
            executor = FakeExecutor()
            events = []
            result = ma.run_agent(
                messages=[{"role": "user", "content": "edit and verify"}],
                executor=executor,
                system_prompt=ma.build_system_prompt(stage_context="Stage: /tmp/x.usda"),
                on_event=lambda kind, payload: events.append((kind, payload)),
                model="pcc",
            )
        finally:
            if had_anthropic:
                sys.modules["anthropic"] = saved_anthropic
            else:
                sys.modules.pop("anthropic", None)

    if ma.resolve_provider() != ma.PROVIDER_APPLE:
        raise AssertionError("MUSE_PROVIDER=apple was not honoured")
    if executor.python_calls != ["print('apple edit')"]:
        raise AssertionError("Apple action did not execute run_python: %r"
                             % executor.python_calls)
    if executor.capture_calls != ["verify apple edit"]:
        raise AssertionError("Apple action did not capture the viewport: %r"
                             % executor.capture_calls)
    if len(server.requests) != 5:
        raise AssertionError("expected 5 two-phase Apple round trips, got %d"
                             % len(server.requests))
    if any(request.get("model") != "system" for request in server.requests):
        raise AssertionError("Apple request escaped to a non-system model: %r"
                             % [r.get("model") for r in server.requests])

    first = server.requests[0]
    schema = ((first.get("response_format") or {}).get("json_schema") or {}).get("schema") or {}
    properties = schema.get("properties") or {}
    if set(properties) != {"action", "tool", "message"}:
        raise AssertionError("Apple decision schema is not compact: %r"
                             % set(properties))
    required = set(schema.get("required") or [])
    if required != set(properties):
        raise AssertionError("fm schema fields must all be required: %r vs %r"
                             % (required, set(properties)))
    if schema.get("x-order") != schema.get("required"):
        raise AssertionError("fm schema x-order must match required")
    argument_schema = (((server.requests[1].get("response_format") or {})
                       .get("json_schema") or {}).get("schema") or {})
    if set((argument_schema.get("properties") or {})) != {"code"}:
        raise AssertionError("run_python did not get its own small schema: %r"
                             % argument_schema)

    second_messages = server.requests[2].get("messages") or []
    if not second_messages or second_messages[0].get("role") != "system":
        raise AssertionError("Apple prompt was not carried as a system message")
    tool_call_messages = [message for message in second_messages
                          if message.get("tool_calls")]
    tool_results = [message for message in second_messages
                    if message.get("role") == "tool"]
    if len(tool_call_messages) != 1 or len(tool_results) != 1:
        raise AssertionError("tool call/result were not converted: %r"
                             % second_messages)
    tool_call = tool_call_messages[0]["tool_calls"][0]
    if tool_call.get("function", {}).get("name") != "run_python":
        raise AssertionError("wrong OpenAI tool call: %r" % tool_call)
    arguments = json.loads(tool_call["function"]["arguments"])
    if arguments != {"code": "print('apple edit')"}:
        raise AssertionError("tool arguments changed in conversion: %r" % arguments)
    if tool_results[0].get("tool_call_id") != tool_call.get("id"):
        raise AssertionError("tool result lost its call id")
    if "Created /World/Ball" not in tool_results[0].get("content", ""):
        raise AssertionError("tool output did not reach Apple FM: %r"
                             % tool_results[0])

    # The capture result cannot be put into an OpenAI role=tool message as an
    # Anthropic image block. It must become an inline data URL on the next turn.
    last_messages = server.requests[-1].get("messages") or []
    image_urls = [
        part.get("image_url", {}).get("url", "")
        for message in last_messages
        for part in (message.get("content") if isinstance(message.get("content"), list) else [])
        if isinstance(part, dict) and part.get("type") == "image_url"
    ]
    if not image_urls or not image_urls[0].startswith("data:image/png;base64,"):
        raise AssertionError("viewport image was not converted for fm serve: %r"
                             % image_urls)
    if not any(kind == "text" and payload.get("text") ==
               "Edited and verified with Apple FM." for kind, payload in events):
        raise AssertionError("Apple final response was not emitted: %r" % events)
    if result[-1].get("role") != "assistant":
        raise AssertionError("Apple conversation did not end with assistant")
    os.environ.pop("MUSE_PROVIDER", None)
    os.environ.pop("MUSE_APPLE_URL", None)
    os.environ.pop("MUSE_BASE_URL", None)
    os.environ.pop("MUSE_MODEL", None)


def testAppleFmHealthFailsClosedWhenSystemIsUnavailable():
    """A running server is not enough: the on-device model must be available."""
    reason = "Apple Intelligence is unavailable in this login session"
    with FakeFmServer(system_available=False, unavailable_reason=reason) as server:
        for name in ("MUSE_API_KEY", "ANTHROPIC_API_KEY", "MUSE_BASE_URL",
                     "ANTHROPIC_BASE_URL", "MUSE_MODEL"):
            os.environ.pop(name, None)
        os.environ["MUSE_PROVIDER"] = "apple"
        os.environ["MUSE_APPLE_URL"] = server.url
        try:
            ma.run_agent(
                messages=[{"role": "user", "content": "hello"}],
                executor=FakeExecutor(),
                system_prompt=ma.build_system_prompt(),
                on_event=lambda kind, payload: None,
            )
        except ma.AgentError as error:
            if reason not in str(error):
                raise AssertionError("health failure hid fm's reason: %s" % error)
        else:
            raise AssertionError("unavailable system model was accepted")
    if server.requests:
        raise AssertionError("completion was sent after failed health preflight")
    os.environ.pop("MUSE_PROVIDER", None)
    os.environ.pop("MUSE_APPLE_URL", None)


def testAppleLoopbackRequestsIgnoreHttpProxies():
    names = ("http_proxy", "HTTP_PROXY", "https_proxy", "HTTPS_PROXY",
             "no_proxy", "NO_PROXY")
    saved = {name: os.environ.get(name) for name in names}
    with FakeFmServer() as real_server, FakeFmServer() as proxy_trap:
        try:
            os.environ["http_proxy"] = proxy_trap.url
            os.environ["HTTP_PROXY"] = proxy_trap.url
            os.environ["https_proxy"] = proxy_trap.url
            os.environ["HTTPS_PROXY"] = proxy_trap.url
            os.environ["no_proxy"] = ""
            os.environ["NO_PROXY"] = ""
            health = ma.fetch_apple_health(real_server.url)
        finally:
            for name, value in saved.items():
                if value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = value
    if ma.describe_apple_health_problem(health):
        raise AssertionError("direct loopback health request failed: %r" % health)
    if proxy_trap.get_requests:
        raise AssertionError("Apple loopback data reached HTTP proxy: %r"
                             % proxy_trap.get_requests)


def testAppleContextCompactionKeepsTheActiveToolChain():
    old = "old exchange " + ("x" * 1800)
    history = [
        {"role": "user", "content": old},
        {"role": "assistant", "content": [{"type": "text", "text": "old answer"}]},
        {"role": "user", "content": "current edit"},
        {"role": "assistant", "content": [{
            "type": "tool_use", "id": "active", "name": "run_python",
            "input": {"code": "print(3)"}}]},
        {"role": "user", "content": [{
            "type": "tool_result", "tool_use_id": "active", "is_error": False,
            "content": [{"type": "text", "text": "3"}]}]},
    ]
    os.environ["MUSE_APPLE_CONTEXT_CHARS"] = "1000"
    try:
        compacted = ma._compact_apple_conversation(history)
    finally:
        os.environ.pop("MUSE_APPLE_CONTEXT_CHARS", None)
    if any(entry.get("content") == old for entry in compacted):
        raise AssertionError("old Apple exchange survived the context budget")
    if len(compacted) != 3 or compacted[0].get("content") != "current edit":
        raise AssertionError("active Apple exchange was split: %r" % compacted)


def testAppleContinuationAfterToolLimitIsNotDroppedOrOrphaned():
    raw = [
        {"role": "user", "content": "old task " + ("x" * 1800)},
        {"role": "assistant", "content": [{
            "type": "tool_use", "id": "last-call", "name": "run_python",
            "input": {"code": "print(1)"}}]},
        {"role": "user", "content": [{
            "type": "tool_result", "tool_use_id": "last-call",
            "content": [{"type": "text", "text": "1"}],
            "is_error": False}]},
        {"role": "user", "content": "continue please"},
    ]
    _extras, normalized = ma.normalize_messages(raw)
    full_chat = ma._apple_chat_messages(normalized, "system")
    if not any(message.get("role") == "user" and
               message.get("content") == [{"type": "text",
                                             "text": "continue please"}]
               for message in full_chat):
        raise AssertionError("merged Apple continuation text was dropped: %r"
                             % full_chat)

    os.environ["MUSE_APPLE_CONTEXT_CHARS"] = "1000"
    try:
        compacted = ma._compact_apple_conversation(normalized)
    finally:
        os.environ.pop("MUSE_APPLE_CONTEXT_CHARS", None)
    compact_chat = ma._apple_chat_messages(compacted, "system")
    if any(message.get("role") == "tool" for message in compact_chat):
        raise AssertionError("compaction orphaned a tool result: %r" % compact_chat)
    if compact_chat[-1].get("content") != [{"type": "text",
                                             "text": "continue please"}]:
        raise AssertionError("compaction lost the continuation: %r" % compact_chat)


def testAppleSingleTurnCompactionDropsOlderViewportPairs():
    history = [{"role": "user", "content": "compare viewport iterations"}]
    for index in range(3):
        call_id = "capture-%d" % index
        history.extend([
            {"role": "assistant", "content": [{
                "type": "tool_use", "id": call_id,
                "name": "capture_viewport", "input": {"note": call_id}}]},
            {"role": "user", "content": [{
                "type": "tool_result", "tool_use_id": call_id,
                "is_error": False,
                "content": [
                    {"type": "text", "text": call_id},
                    {"type": "image", "source": {
                        "type": "base64", "media_type": "image/png",
                        "data": str(index) * 2000}},
                ]}]},
        ])
    os.environ["MUSE_APPLE_CONTEXT_CHARS"] = "1000"
    try:
        compacted = ma._compact_apple_conversation(history)
    finally:
        os.environ.pop("MUSE_APPLE_CONTEXT_CHARS", None)
    encoded = json.dumps(compacted)
    if "capture-2" not in encoded or "capture-0" in encoded or "capture-1" in encoded:
        raise AssertionError("single-turn viewport history was not bounded")
    chat = ma._apple_chat_messages(compacted, "system")
    calls = [call["id"] for message in chat for call in message.get("tool_calls", [])]
    results = [message.get("tool_call_id") for message in chat
               if message.get("role") == "tool"]
    if calls != ["capture-2"] or results != calls:
        raise AssertionError("bounded viewport pair became invalid: %r" % chat)


def testToolLoopRunsToolsAndFeedsResultsBack():
    captured = []
    turns = [
        types.SimpleNamespace(
            stop_reason="tool_use",
            content=[
                _block("text", text="Looking at the stage first."),
                _block("tool_use", id="t1", name="inspect_stage", input={"mode": "summary"}),
            ]),
        types.SimpleNamespace(
            stop_reason="tool_use",
            content=[
                _block("tool_use", id="t2", name="run_python",
                       input={"code": "stage.DefinePrim('/World/Ball','Sphere')"}),
            ]),
        types.SimpleNamespace(
            stop_reason="tool_use",
            content=[
                _block("tool_use", id="t3", name="capture_viewport", input={"note": "verify"}),
            ]),
        types.SimpleNamespace(
            stop_reason="end_turn",
            content=[_block("text", text="Created /World/Ball and confirmed it on screen.")]),
    ]
    installFakeAnthropic(turns, captured)
    os.environ["MUSE_API_KEY"] = "sk-ant-" + "test-not-a-real-key"

    executor = FakeExecutor()
    events = []
    result = ma.run_agent(
        messages=[{"role": "user", "content": "create a sphere at /World/Ball and show me"}],
        executor=executor,
        system_prompt=ma.build_system_prompt(stage_context="Stage: /tmp/x.usda"),
        on_event=lambda kind, payload: events.append((kind, payload)),
    )

    if len(captured) != 4:
        raise AssertionError("expected 4 round trips, got %d" % len(captured))
    if executor.inspect_calls != ["summary"]:
        raise AssertionError("inspect_stage was not run: %s" % executor.inspect_calls)
    if len(executor.python_calls) != 1:
        raise AssertionError("run_python was not run: %s" % executor.python_calls)
    if executor.capture_calls != ["verify"]:
        raise AssertionError("capture_viewport was not run: %s" % executor.capture_calls)

    # Every single request must be one the API would accept.
    for request in captured:
        assertValidRequest(request["system"], request["messages"])
        if "temperature" in request or "top_p" in request or "top_k" in request:
            raise AssertionError("sampling params are rejected on this model")
        if request["thinking"] != {"type": "adaptive", "display": "summarized"}:
            raise AssertionError("expected adaptive thinking, got %s" % request["thinking"])

    # The captured screenshot must reach the model as an actual image block.
    last = captured[-1]["messages"]
    image_blocks = [
        block
        for message in last if isinstance(message.get("content"), list)
        for entry in message["content"] if isinstance(entry, dict)
        for block in (entry.get("content") or []) if isinstance(block, dict)
        if block.get("type") == "image"
    ]
    if not image_blocks:
        raise AssertionError("the viewport capture never reached the model as an image")

    kinds = [kind for kind, _ in events]
    for expected in ("tool_use", "tool_result", "text"):
        if expected not in kinds:
            raise AssertionError("no %r event was emitted: %s" % (expected, kinds))
    if result[-1]["role"] != "assistant":
        raise AssertionError("conversation should end on the assistant turn")


def testToolErrorsAreReportedNotSwallowed():
    class Failing(FakeExecutor):
        def run_python(self, code):
            raise RuntimeError("Accessed schema on invalid prim")

    captured = []
    turns = [
        types.SimpleNamespace(
            stop_reason="tool_use",
            content=[_block("tool_use", id="t1", name="run_python", input={"code": "boom"})]),
        types.SimpleNamespace(stop_reason="end_turn",
                              content=[_block("text", text="That path does not exist.")]),
    ]
    installFakeAnthropic(turns, captured)
    os.environ["MUSE_API_KEY"] = "sk-ant-" + "test-not-a-real-key"

    ma.run_agent(
        messages=[{"role": "user", "content": "do the thing"}],
        executor=Failing(),
        system_prompt=ma.build_system_prompt(),
        on_event=lambda kind, payload: None,
    )
    results = [
        entry
        for message in captured[-1]["messages"] if isinstance(message.get("content"), list)
        for entry in message["content"]
        if isinstance(entry, dict) and entry.get("type") == "tool_result"
    ]
    if not results or not results[0].get("is_error"):
        raise AssertionError("the tool failure was not reported back to the model")
    text = json.dumps(results[0]["content"])
    if "invalid prim" not in text:
        raise AssertionError("the error text did not reach the model: %s" % text)


def testThinkingRejectionDegradesInsteadOfFailingTheSession():
    """
    Adaptive thinking and effort are model-gated. A model that refuses them
    must cost the user a retry, not the whole turn.
    """
    captured = []
    calls = {"n": 0}

    class _Rejected(Exception):
        status_code = 400

        def __str__(self):
            return ("Error code: 400 - {'error': {'message': "
                    "'thinking: unsupported on this model'}}")

    class _Stream(object):
        def __init__(self, message):
            self._message = message

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def get_final_message(self):
            return self._message

    class _Messages(object):
        def stream(self, **kwargs):
            calls["n"] += 1
            captured.append(kwargs)
            if "thinking" in kwargs:
                raise _Rejected()
            return _Stream(types.SimpleNamespace(
                stop_reason="end_turn",
                content=[_block("text", text="worked without thinking")]))

    class _Client(object):
        def __init__(self, api_key=None, **kwargs):
            self.messages = _Messages()

    module = types.ModuleType("anthropic")
    module.Anthropic = _Client
    sys.modules["anthropic"] = module
    os.environ["MUSE_API_KEY"] = "sk-ant-" + "test-not-a-real-key"

    events = []
    ma.run_agent(messages=[{"role": "user", "content": "hi"}],
                 executor=FakeExecutor(),
                 system_prompt=ma.build_system_prompt(),
                 on_event=lambda kind, payload: events.append((kind, payload)))

    if calls["n"] != 2:
        raise AssertionError("expected one retry, got %d calls" % calls["n"])
    if "thinking" in captured[-1]:
        raise AssertionError("the retry still carried the rejected parameter")
    if not any(k == "text" for k, _ in events):
        raise AssertionError("the degraded retry produced no answer")


def testMissingKeyIsAClearMessageNotACrash():
    for name in ("MUSE_API_KEY", "ANTHROPIC_API_KEY", "MUSE_BASE_URL", "ANTHROPIC_BASE_URL"):
        os.environ.pop(name, None)
    installFakeAnthropic([], [])
    try:
        ma.run_agent(messages=[{"role": "user", "content": "hi"}],
                     executor=FakeExecutor(),
                     system_prompt=ma.build_system_prompt(),
                     on_event=lambda kind, payload: None)
    except ma.AgentError as error:
        if "MUSE_API_KEY" not in str(error):
            raise AssertionError("unhelpful key error: %s" % error)
        return
    raise AssertionError("a missing key should raise AgentError")


def testMetaMuseKeyRoutesToMetaWithTheRightModel():
    """
    A Meta Muse key (`LLM_…`) must go to Meta's Messages API on the
    contributor tier, not to api.anthropic.com — which answers that key shape
    with 401. The old plugin instead handed such keys to a local stub that
    fabricated replies, which is why the panel looked alive while changing
    nothing.
    """
    for name in ("MUSE_BASE_URL", "ANTHROPIC_BASE_URL", "MUSE_MODEL"):
        os.environ.pop(name, None)
    os.environ["MUSE_API_KEY"] = _TEST_META_KEY

    base_url, source = ma.resolve_base_url()
    if base_url != ma.META_BASE_URL:
        raise AssertionError("a Meta Muse key routed to %r (%s)" % (base_url, source))
    if ma.resolve_model(base_url) != "muse-spark-1.2-contributor":
        raise AssertionError("wrong default model: %s" % ma.resolve_model(base_url))
    if ma.describe_key_problem("LLM_x", base_url) is not None:
        raise AssertionError("a routed Meta key should not be reported as a problem")

    seen = {}

    class _Stream(object):
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def get_final_message(self):
            return types.SimpleNamespace(
                stop_reason="end_turn", content=[_block("text", text="ok")])

    class _Client(object):
        def __init__(self, api_key=None, base_url=None, **kwargs):
            seen["base_url"] = base_url
            self.messages = types.SimpleNamespace(
                stream=lambda **kw: (seen.update(model=kw["model"]), _Stream())[1])

    module = types.ModuleType("anthropic")
    module.Anthropic = _Client
    sys.modules["anthropic"] = module

    ma.run_agent(messages=[{"role": "user", "content": "hi"}],
                 executor=FakeExecutor(),
                 system_prompt=ma.build_system_prompt(),
                 on_event=lambda kind, payload: None)

    if seen.get("base_url") != ma.META_BASE_URL:
        raise AssertionError("client was not pointed at Meta: %s" % seen)
    if seen.get("model") != "muse-spark-1.2-contributor":
        raise AssertionError("wrong model sent to Meta: %s" % seen)

    # MUSE_MODEL may not drop Meta back to the standard tier.
    os.environ["MUSE_MODEL"] = "muse-spark-1.1"
    if ma.resolve_model(ma.META_BASE_URL) != "muse-spark-1.2-contributor":
        raise AssertionError("MUSE_MODEL escaped the contributor tier")
    os.environ.pop("MUSE_MODEL")


def testUnknownKeyWithNoEndpointIsRefusedWithTheReason():
    for name in ("MUSE_BASE_URL", "ANTHROPIC_BASE_URL"):
        os.environ.pop(name, None)
    problem = ma.describe_key_problem("xoxb-" + "some-other-service", None)
    if not problem or "MUSE_BASE_URL" not in problem:
        raise AssertionError("unhelpful description: %r" % problem)
    if ma.describe_key_problem("sk-ant-real", None) is not None:
        raise AssertionError("an Anthropic key must not be flagged")


def testTheRequestCarriesTheContributorModelAndXhighEffort():
    """What actually goes on the wire, not what the helpers return.

    Model and effort are set in different places — resolve_model/
    force_contributor_model for one, DEFAULT_EFFORT for the other — so this
    reads them back off the captured request together.
    """
    os.environ["MUSE_API_KEY"] = _TEST_META_KEY
    for name in ("MUSE_BASE_URL", "ANTHROPIC_BASE_URL", "MUSE_MODEL", "MUSE_EFFORT"):
        os.environ.pop(name, None)

    captured = []
    installFakeAnthropic(
        [types.SimpleNamespace(stop_reason="end_turn",
                               content=[_block("text", text="ok")])],
        captured)

    ma.run_agent(messages=[{"role": "user", "content": "hi"}],
                 executor=FakeExecutor(),
                 system_prompt=ma.build_system_prompt(),
                 on_event=lambda kind, payload: None)

    request = captured[0]
    if request["model"] != "muse-spark-1.2-contributor":
        raise AssertionError("model on the wire was %r" % request["model"])
    effort = (request.get("output_config") or {}).get("effort")
    if effort != "xhigh":
        raise AssertionError("effort on the wire was %r" % effort)
    os.environ.pop("MUSE_API_KEY", None)


def testSavedKeyRoundTripsAndNeverBeatsTheEnvironment(tmpdir):
    """
    Muse ▸ Settings… can persist a key so it survives a restart. Two things
    matter beyond "it writes a file": the file must not be world-readable,
    since it holds a bearer credential, and a saved key must never override
    MUSE_API_KEY from the shell — a stale saved key silently winning is very
    hard to debug from the symptom.
    """
    import stat
    import museAssistant as mu

    path = str(pathlib.Path(tmpdir) / "credentials.json")
    if mu.load_saved_api_key(path) is not None:
        raise AssertionError("a missing file must read as no key")

    mu.save_api_key("LLM_saved_example", path)
    if mu.load_saved_api_key(path) != "LLM_saved_example":
        raise AssertionError("the saved key did not round-trip")
    mode = stat.S_IMODE(os.stat(path).st_mode)
    if mode & 0o077:
        raise AssertionError("credentials are readable by others: %o" % mode)

    for name in ("MUSE_API_KEY", "ANTHROPIC_API_KEY"):
        os.environ.pop(name, None)
    if mu.apply_saved_api_key(path) != "LLM_saved_example":
        raise AssertionError("the saved key was not applied to an empty env")
    if os.environ.get("MUSE_API_KEY") != "LLM_saved_example":
        raise AssertionError("apply_saved_api_key did not set the variable")

    os.environ["MUSE_API_KEY"] = "LLM_from_the_shell"
    if mu.apply_saved_api_key(path) is not None:
        raise AssertionError("a saved key must not override the environment")
    if os.environ.get("MUSE_API_KEY") != "LLM_from_the_shell":
        raise AssertionError("the shell's key was overwritten")

    if not mu.forget_api_key(path):
        raise AssertionError("forget_api_key did not report removing it")
    if mu.load_saved_api_key(path) is not None:
        raise AssertionError("the key survived being forgotten")
    if mu.forget_api_key(path):
        raise AssertionError("forgetting twice should report nothing to remove")
    os.environ.pop("MUSE_API_KEY", None)


def testCorruptCredentialsFileIsNotFatal(tmpdir):
    """A half-written or hand-edited file must not stop usdview loading."""
    import museAssistant as mu

    path = pathlib.Path(tmpdir) / "credentials.json"
    path.write_text("{not json at all")
    if mu.load_saved_api_key(str(path)) is not None:
        raise AssertionError("unreadable credentials should read as no key")
    path.write_text('{"MUSE_API_KEY": ""}')
    if mu.load_saved_api_key(str(path)) is not None:
        raise AssertionError("an empty key should read as no key")


def testOllamaProviderRoutesLocallyWithNoKey():
    """The local back end: selected explicitly, needs no credential.

    Explicit selection is the whole design. An Ollama server lives at whatever
    address the user runs it on, so there is nothing to sniff — and guessing
    "this unfamiliar host is probably local" would be a guess that sends an
    Anthropic key somewhere it should never go.
    """
    import museAgent as ma

    for name in ("MUSE_PROVIDER", "MUSE_BASE_URL", "ANTHROPIC_BASE_URL",
                 "MUSE_API_KEY", "ANTHROPIC_API_KEY", "MUSE_MODEL",
                 "MUSE_OLLAMA_URL"):
        os.environ.pop(name, None)

    # Nothing set: unchanged from before providers existed.
    if ma.resolve_provider() != ma.PROVIDER_ANTHROPIC:
        raise AssertionError("the default provider changed")

    os.environ["MUSE_PROVIDER"] = "ollama"
    if ma.resolve_provider() != ma.PROVIDER_OLLAMA:
        raise AssertionError("MUSE_PROVIDER=ollama was not honoured")
    base, source = ma.resolve_base_url()
    if base != ma.OLLAMA_DEFAULT_BASE_URL or source != "Ollama":
        raise AssertionError("ollama did not resolve its endpoint: %r" % (base,))
    if ma.resolve_model(base) != ma.OLLAMA_DEFAULT_MODEL:
        raise AssertionError("ollama did not default its model")

    # The point: no key required, where every other back end demands one.
    if ma.describe_key_problem(None, base) is not None:
        raise AssertionError("ollama demanded an API key")
    os.environ["MUSE_PROVIDER"] = "anthropic"
    if ma.describe_key_problem(None, None) is None:
        raise AssertionError("a hosted back end stopped requiring a key")

    # An explicit address wins over the default.
    os.environ["MUSE_PROVIDER"] = "ollama"
    os.environ["MUSE_OLLAMA_URL"] = "http://elsewhere:11434/"
    if ma.resolve_base_url()[0] != "http://elsewhere:11434":
        raise AssertionError("MUSE_OLLAMA_URL was ignored (or kept its slash)")

    # MUSE_BASE_URL still outranks everything, as it always has.
    os.environ["MUSE_BASE_URL"] = "https://gateway.example/v1"
    if ma.resolve_base_url()[0] != "https://gateway.example/v1":
        raise AssertionError("MUSE_BASE_URL stopped winning")

    for name in ("MUSE_PROVIDER", "MUSE_OLLAMA_URL", "MUSE_BASE_URL"):
        os.environ.pop(name, None)


def testOllamaModelWithoutToolsIsRefused():
    """A completion-only model must be refused, not quietly accepted.

    This is the failure this plugin has a standing rule against: Muse works
    entirely through tool calls, so a model that cannot emit them streams a
    fluent reply and changes nothing — the panel looks alive and the stage is
    untouched. Roughly half the models on a typical Ollama server are
    completion-only, so it is the common case.
    """
    import museAgent as ma

    os.environ["MUSE_PROVIDER"] = "ollama"
    installed = [
        {"name": "toolful:9b", "tools": True, "vision": True, "thinking": True},
        {"name": "chatty:27b", "tools": False, "vision": False, "thinking": False},
    ]

    if ma.describe_model_problem("toolful:9b", models=installed) is not None:
        raise AssertionError("a tool-capable model was refused")

    problem = ma.describe_model_problem("chatty:27b", models=installed)
    if not problem or "tool" not in problem:
        raise AssertionError("a completion-only model was accepted: %r" % problem)
    if "toolful:9b" not in problem:
        raise AssertionError("the refusal does not name a usable alternative")

    missing = ma.describe_model_problem("not-installed:1b", models=installed)
    if not missing or "not installed" not in missing:
        raise AssertionError("an absent model was not reported: %r" % missing)

    # An unreachable server is not the model's fault, and must not block.
    if ma.describe_model_problem("anything", models=[]) is not None:
        raise AssertionError("an unreachable server blocked the model")

    # The hosted back ends are not ours to enumerate.
    os.environ["MUSE_PROVIDER"] = "anthropic"
    if ma.describe_model_problem("claude-opus-5", models=installed) is not None:
        raise AssertionError("a hosted model was judged against Ollama's list")
    os.environ.pop("MUSE_PROVIDER", None)


def testBackEndSettingsRoundTripBesideTheKey(tmpdir):
    """Provider, address and model persist — without eating the API key.

    They share one file, so the two ways of writing it have to merge rather
    than replace: saving a key used to be a way to lose the back end, and
    forgetting a key must not also forget which server to talk to.
    """
    import museAssistant as mu

    path = str(pathlib.Path(tmpdir) / "settings.json")
    mu.save_api_key("sk-ant-example", path)
    mu.save_settings({"MUSE_PROVIDER": "ollama",
                      "MUSE_OLLAMA_URL": "http://box:11434",
                      "MUSE_MODEL": "toolful:9b"}, path)

    if mu.load_saved_api_key(path) != "sk-ant-example":
        raise AssertionError("saving the back end destroyed the key")
    saved = mu.load_saved_settings(path)
    if saved.get("MUSE_PROVIDER") != "ollama" or \
            saved.get("MUSE_MODEL") != "toolful:9b":
        raise AssertionError("the back end did not round-trip: %r" % saved)

    mu.save_api_key("sk-ant-second", path)
    if mu.load_saved_settings(path).get("MUSE_PROVIDER") != "ollama":
        raise AssertionError("re-saving the key destroyed the back end")

    if not mu.forget_api_key(path):
        raise AssertionError("forget_api_key did not report removing it")
    if mu.load_saved_api_key(path) is not None:
        raise AssertionError("the key survived being forgotten")
    if mu.load_saved_settings(path).get("MUSE_PROVIDER") != "ollama":
        raise AssertionError("forgetting the key also forgot the back end")

    for name in ("MUSE_PROVIDER", "MUSE_OLLAMA_URL", "MUSE_APPLE_URL",
                 "MUSE_MODEL"):
        os.environ.pop(name, None)
    applied = mu.apply_saved_settings(path)
    if applied.get("MUSE_PROVIDER") != "ollama":
        raise AssertionError("saved settings were not applied to an empty env")
    if os.environ.get("MUSE_MODEL") != "toolful:9b":
        raise AssertionError("apply_saved_settings did not set the variables")

    # The shell wins, for the same reason it wins for the key.
    os.environ["MUSE_MODEL"] = "from-the-shell"
    mu.apply_saved_settings(path)
    if os.environ.get("MUSE_MODEL") != "from-the-shell":
        raise AssertionError("a saved setting overrode the environment")

    # Apple has its own persisted endpoint, clears stale Ollama/model pins, and
    # still leaves the hosted credential alongside it.
    mu.save_settings({"MUSE_PROVIDER": "apple",
                      "MUSE_APPLE_URL": "http://127.0.0.1:1976",
                      "MUSE_OLLAMA_URL": "", "MUSE_MODEL": ""}, path)
    saved = mu.load_saved_settings(path)
    if saved != {"MUSE_PROVIDER": "apple",
                 "MUSE_APPLE_URL": "http://127.0.0.1:1976"}:
        raise AssertionError("Apple settings did not round-trip cleanly: %r" % saved)

    # Clearing removes the pin rather than storing an empty string.
    mu.save_settings({"MUSE_PROVIDER": "", "MUSE_OLLAMA_URL": "",
                      "MUSE_APPLE_URL": "", "MUSE_MODEL": ""}, path)
    if mu.load_saved_settings(path):
        raise AssertionError("cleared settings were still saved")

    for name in ("MUSE_PROVIDER", "MUSE_OLLAMA_URL", "MUSE_APPLE_URL",
                 "MUSE_MODEL"):
        os.environ.pop(name, None)


def testMetaGetsBearerAndAnthropicGetsApiKey():
    """
    The auth header belongs to the endpoint, not the key. Meta documents a
    bearer token; api.anthropic.com takes x-api-key. Sending Meta an x-api-key
    is a 401 no matter how valid the key is, which is indistinguishable from a
    dead key and cost real debugging time.
    """
    if ma.resolve_auth_style(ma.META_BASE_URL) != "bearer":
        raise AssertionError("Meta must be sent a bearer token")
    if ma.resolve_auth_style("https://api.meta.ai/v1") != "bearer":
        raise AssertionError("the /v1 form is still Meta")
    if ma.resolve_auth_style(None) != "api_key":
        raise AssertionError("Anthropic must keep x-api-key")
    # A gateway is a third service whose scheme we do not know; it keeps the
    # behaviour it has always had.
    if ma.resolve_auth_style("https://gateway.example.internal/anthropic") != "api_key":
        raise AssertionError("a gateway must keep x-api-key")


def testMetaKeyReachesTheClientAsABearerToken():
    """The style must actually reach the SDK, not just be computed."""
    os.environ["MUSE_API_KEY"] = _TEST_META_KEY
    for name in ("MUSE_BASE_URL", "ANTHROPIC_BASE_URL", "MUSE_MODEL"):
        os.environ.pop(name, None)

    seen = {}

    class _Stream(object):
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def get_final_message(self):
            return types.SimpleNamespace(
                stop_reason="end_turn", content=[_block("text", text="ok")])

    class _Messages(object):
        def stream(self, **kwargs):
            seen["model"] = kwargs.get("model")
            return _Stream()

    class _Client(object):
        def __init__(self, api_key=None, auth_token=None, base_url=None, **kwargs):
            seen["api_key"] = api_key
            seen["auth_token"] = auth_token
            seen["base_url"] = base_url
            self.messages = _Messages()

    module = types.ModuleType("anthropic")
    module.Anthropic = _Client
    sys.modules["anthropic"] = module

    ma.run_agent(messages=[{"role": "user", "content": "hi"}],
                 executor=FakeExecutor(),
                 system_prompt=ma.build_system_prompt(),
                 on_event=lambda kind, payload: None)

    if seen.get("auth_token") != _TEST_META_KEY:
        raise AssertionError("the Meta key was not sent as a bearer token: %s" % seen)
    if seen.get("api_key") is not None:
        raise AssertionError("the Meta key must not also go in x-api-key: %s" % seen)
    if seen.get("base_url") != ma.META_BASE_URL:
        raise AssertionError("not routed to Meta: %s" % seen)
    if seen.get("model") != "muse-spark-1.2-contributor":
        raise AssertionError("expected the contributor tier, got %s" % seen.get("model"))
    os.environ.pop("MUSE_API_KEY", None)


def testEveryMetaCallUsesTheContributorTier():
    """
    Both routes to a model — MUSE_MODEL and an explicit model= argument — must
    land on the contributor tier when the endpoint is Meta. A standard-tier id
    is replaced outright rather than having "-contributor" appended, because
    only muse-spark-1.2-contributor is published and a synthesised
    "muse-spark-1.1-contributor" would fail at the API instead of here.
    """
    META = ma.META_BASE_URL
    os.environ.pop("MUSE_MODEL", None)
    if ma.resolve_model(META) != "muse-spark-1.2-contributor":
        raise AssertionError("the Meta default is not the contributor tier")

    for standard in ("muse-spark-1.1", "muse-spark-1.2"):
        os.environ["MUSE_MODEL"] = standard
        if ma.resolve_model(META) != "muse-spark-1.2-contributor":
            raise AssertionError("MUSE_MODEL=%s escaped the contributor tier" % standard)
    os.environ.pop("MUSE_MODEL")

    # An explicit argument goes through the same gate.
    if ma.force_contributor_model("muse-spark-1.2", META) != "muse-spark-1.2-contributor":
        raise AssertionError("an explicit model= escaped the contributor tier")

    # A future contributor model is left alone rather than rewritten.
    if ma.force_contributor_model("muse-spark-9.9-contributor", META) \
            != "muse-spark-9.9-contributor":
        raise AssertionError("an already-contributor model was rewritten")

    # The tier is a Meta concept; nothing else is touched.
    if ma.force_contributor_model("claude-opus-5", None) != "claude-opus-5":
        raise AssertionError("Anthropic's model was rewritten")
    if ma.force_contributor_model("gw-model", "https://gw.internal/anthropic") != "gw-model":
        raise AssertionError("a gateway's model was rewritten")
    if ma.resolve_model(None) != ma.DEFAULT_MODEL:
        raise AssertionError("the Anthropic default changed")


def testExplicitModelArgumentCannotEscapeTheTier():
    """The guarantee has to hold at the request, not just in the helper."""
    os.environ["MUSE_API_KEY"] = _TEST_META_KEY
    for name in ("MUSE_BASE_URL", "ANTHROPIC_BASE_URL", "MUSE_MODEL"):
        os.environ.pop(name, None)

    captured = []
    installFakeAnthropic(
        [types.SimpleNamespace(stop_reason="end_turn",
                               content=[_block("text", text="ok")])],
        captured)

    ma.run_agent(messages=[{"role": "user", "content": "hi"}],
                 executor=FakeExecutor(),
                 system_prompt=ma.build_system_prompt(),
                 on_event=lambda kind, payload: None,
                 model="muse-spark-1.2")

    sent = captured[0]["model"]
    if sent != "muse-spark-1.2-contributor":
        raise AssertionError("the request went out as %r" % sent)
    os.environ.pop("MUSE_API_KEY", None)


def testRejectedKeyNamesTheEndpointAndTheKey():
    """
    A 401 is the endpoint refusing the credential. The SDK's own text is a
    repr of the error JSON, which names neither the endpoint that answered nor
    the key that was sent -- and for a Meta Muse key both were chosen
    implicitly, so neither is obvious to whoever has to fix it.
    """
    class _Unauthorized(Exception):
        status_code = 401

        def __str__(self):
            return ("Error code: 401 - {'error': {'message': 'Unauthorized', "
                    "'type': 'authentication_error'}, 'type': 'error'}")

    described = ma.describe_auth_failure(
        _Unauthorized(), _TEST_META_KEY_WITH_FINGERPRINT, "MUSE_API_KEY",
        ma.META_BASE_URL, "Meta Muse key")

    if not described:
        raise AssertionError("a 401 must be described")
    for expected in (ma.META_BASE_URL, "MUSE_API_KEY", "401", "T3ST"):
        if expected not in described:
            raise AssertionError("%r missing from: %s" % (expected, described))
    # The whole key must never be echoed into a panel or a log.
    if _TEST_META_KEY in described:
        raise AssertionError("the key itself leaked into the message")


def testNonAuthErrorsAreLeftAlone():
    """Only 401 gets the treatment; everything else keeps the SDK's own text."""
    class _Overloaded(Exception):
        status_code = 529

    if ma.describe_auth_failure(_Overloaded(), "k", "MUSE_API_KEY", None, None):
        raise AssertionError("a 529 must not be described as an auth failure")
    if ma.describe_auth_failure(ValueError("boom"), "k", "MUSE_API_KEY", None, None):
        raise AssertionError("a non-HTTP error must not be described as auth")


def testGatewayBaseUrlMakesANonAnthropicKeyUsable():
    """
    A gateway that speaks the Messages API issues its own key format. With a
    base URL configured, the key must be passed through rather than refused.
    """
    os.environ["MUSE_API_KEY"] = _TEST_META_KEY
    os.environ["MUSE_BASE_URL"] = "https://gateway.example.internal/anthropic/"

    seen = {}

    class _Stream(object):
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def get_final_message(self):
            return types.SimpleNamespace(
                stop_reason="end_turn",
                content=[_block("text", text="routed through the gateway")])

    class _Messages(object):
        def stream(self, **kwargs):
            return _Stream()

    class _Client(object):
        def __init__(self, api_key=None, base_url=None, **kwargs):
            seen["api_key"] = api_key
            seen["base_url"] = base_url
            self.messages = _Messages()

    module = types.ModuleType("anthropic")
    module.Anthropic = _Client
    sys.modules["anthropic"] = module

    ma.run_agent(messages=[{"role": "user", "content": "hi"}],
                 executor=FakeExecutor(),
                 system_prompt=ma.build_system_prompt(),
                 on_event=lambda kind, payload: None)

    if seen.get("base_url") != "https://gateway.example.internal/anthropic":
        raise AssertionError("base url was not forwarded (or not normalized): %s" % seen)
    if seen.get("api_key") != _TEST_META_KEY:
        raise AssertionError("the gateway key was not forwarded: %s" % seen)
    os.environ.pop("MUSE_BASE_URL", None)


# ---------------------------------------------------------------------------
# Camera metadata round-trip
# ---------------------------------------------------------------------------

def _minimalPng():
    """A real 1x1 PNG, so the chunk walker is exercised against valid bytes."""
    def chunk(ctype, payload):
        import struct
        import zlib as _zlib
        return (struct.pack(">I", len(payload)) + ctype + payload
                + struct.pack(">I", _zlib.crc32(ctype + payload) & 0xFFFFFFFF))

    import struct
    import zlib as _zlib
    ihdr = struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0)
    idat = _zlib.compress(b"\x00\xff\xff\xff")
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", idat) + chunk(b"IEND", b""))


def testCameraMetadataSurvivesAPngRoundTrip(tmpdir):
    camera = {
        "camera_prim_path": "freeCamera",
        "viewMatrix": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, -10, 1]],
        "position": [0.0, 1.5, 10.0],
        "fov": 60.0,
        "currentFrame": "1001",
    }
    png = ma.png_write_text(_minimalPng(), {
        ma.CAMERA_METADATA_KEY: json.dumps(camera),
        ma.ANNOTATION_METADATA_KEY: "move the cube here",
        ma.SCENE_METADATA_KEY: "Stage: /tmp/x.usda\nPrims: /World/Cube",
    })

    recovered = ma.png_read_text(png)
    if ma.CAMERA_METADATA_KEY not in recovered:
        raise AssertionError("camera chunk did not survive the write")
    if json.loads(recovered[ma.CAMERA_METADATA_KEY]) != camera:
        raise AssertionError("camera metadata changed across the round trip")

    path = os.path.join(tmpdir, "shot.png")
    with open(path, "wb") as handle:
        handle.write(png)

    attachment = ma.Attachment.from_file(path)
    if not attachment.has_camera():
        raise AssertionError("Attachment.from_file did not recover the camera")
    if attachment.camera["fov"] != 60.0:
        raise AssertionError("camera values were mangled: %s" % attachment.camera)
    if attachment.annotation != "move the cube here":
        raise AssertionError("annotation was lost")
    block = attachment.to_context_block(0)
    if "viewMatrix" not in block or "move the cube here" not in block:
        raise AssertionError("context block omitted the camera or annotation")


def testRetaggingReplacesMetadataRatherThanLayeringIt():
    """
    Re-tagging an image must supersede the old camera, not leave it behind as
    a second chunk for the reader to pick up instead.
    """
    first = ma.png_write_text(_minimalPng(), {
        ma.CAMERA_METADATA_KEY: json.dumps({"fov": 10.0}),
        ma.ANNOTATION_METADATA_KEY: "first pass",
    })
    second = ma.png_write_text(first, {
        ma.CAMERA_METADATA_KEY: json.dumps({"fov": 99.0}),
    })
    recovered = ma.png_read_text(second)
    camera = json.loads(recovered[ma.CAMERA_METADATA_KEY])
    if camera["fov"] != 99.0:
        raise AssertionError("stale camera survived a re-tag: %s" % camera)
    if recovered.get(ma.ANNOTATION_METADATA_KEY) != "first pass":
        raise AssertionError("re-tagging one key destroyed an unrelated key")
    if not second.startswith(b"\x89PNG") or b"IEND" not in second:
        raise AssertionError("re-tagging corrupted the PNG structure")


def testSidecarCameraJsonIsPickedUp(tmpdir):
    path = os.path.join(tmpdir, "external.png")
    with open(path, "wb") as handle:
        handle.write(_minimalPng())
    with open(os.path.join(tmpdir, "external.camera.json"), "w") as handle:
        json.dump({"camera": {"fov": 35.0, "camera_prim_path": "/World/Cam"}}, handle)

    attachment = ma.Attachment.from_file(path)
    if not attachment.has_camera() or attachment.camera["fov"] != 35.0:
        raise AssertionError("sidecar camera json was not read: %s" % attachment.camera)


def testImageWithoutMetadataStillSends(tmpdir):
    path = os.path.join(tmpdir, "plain.png")
    with open(path, "wb") as handle:
        handle.write(_minimalPng())
    attachment = ma.Attachment.from_file(path)
    if attachment.has_camera():
        raise AssertionError("invented camera metadata that was never there")
    if "none recorded" not in attachment.to_context_block(0):
        raise AssertionError("did not say the image has no camera metadata")
    content = ma.build_user_content("what am I looking at?", [attachment])
    if not any(b.get("type") == "image" for b in content):
        raise AssertionError("the image never made it into the user turn")


def testMultipleImagesAreOrderedAndLabelled(tmpdir):
    attachments = []
    for index, fov in enumerate((30.0, 60.0)):
        path = os.path.join(tmpdir, "view%d.png" % index)
        png = ma.png_write_text(_minimalPng(), {
            ma.CAMERA_METADATA_KEY: json.dumps({"fov": fov, "capture_index": index}),
        })
        with open(path, "wb") as handle:
            handle.write(png)
        attachments.append(ma.Attachment.from_file(path))

    content = ma.build_user_content("reconcile these two views", attachments)
    text = content[0]["text"]
    if "Image 1: view0.png" not in text or "Image 2: view1.png" not in text:
        raise AssertionError("images were not labelled in order:\n%s" % text)
    if text.index("view0.png") > text.index("view1.png"):
        raise AssertionError("capture order was not preserved")
    images = [b for b in content if b.get("type") == "image"]
    if len(images) != 2:
        raise AssertionError("expected 2 image blocks, got %d" % len(images))


def main():
    import shutil
    import tempfile

    saved_env = {name: os.environ.get(name) for name in
                 ("MUSE_API_KEY", "ANTHROPIC_API_KEY",
                  "MUSE_BASE_URL", "ANTHROPIC_BASE_URL", "MUSE_MODEL",
                  "MUSE_PROVIDER", "MUSE_OLLAMA_URL", "MUSE_APPLE_URL")}
    tmpdir = tempfile.mkdtemp(prefix="museAgentTest")
    try:
        testMessagesSurviveARealisticSession()
        testConsecutiveUserTurnsAreMerged()
        testLeadingAssistantTurnIsDropped()
        testToolLoopRunsToolsAndFeedsResultsBack()
        testToolErrorsAreReportedNotSwallowed()
        testAppleFmLoopNeedsNoAnthropicAndConvertsToolsAndImages()
        testAppleFmHealthFailsClosedWhenSystemIsUnavailable()
        testAppleLoopbackRequestsIgnoreHttpProxies()
        testAppleContextCompactionKeepsTheActiveToolChain()
        testAppleContinuationAfterToolLimitIsNotDroppedOrOrphaned()
        testAppleSingleTurnCompactionDropsOlderViewportPairs()
        testThinkingRejectionDegradesInsteadOfFailingTheSession()
        testMissingKeyIsAClearMessageNotACrash()
        testMetaMuseKeyRoutesToMetaWithTheRightModel()
        testUnknownKeyWithNoEndpointIsRefusedWithTheReason()
        testGatewayBaseUrlMakesANonAnthropicKeyUsable()
        testCameraMetadataSurvivesAPngRoundTrip(tmpdir)
        testRetaggingReplacesMetadataRatherThanLayeringIt()
        testSidecarCameraJsonIsPickedUp(tmpdir)
        testImageWithoutMetadataStillSends(tmpdir)
        testMultipleImagesAreOrderedAndLabelled(tmpdir)
        testOllamaProviderRoutesLocallyWithNoKey()
        testOllamaModelWithoutToolsIsRefused()
        testBackEndSettingsRoundTripBesideTheKey(tmpdir)
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)
        for name, value in saved_env.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value
    print("MUSE_AGENT_OK all checks passed")


if __name__ == "__main__":
    main()
