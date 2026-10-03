#!/usr/bin/env python
"""
Headless test for voice-to-select on the REAL biped.

The pure parts of the feature -- tokenizing, the grammar, the ambiguity
policy, the push-to-talk state machine -- are tested in the shared
TouchPose repo (plain pytest, no
host). What can
only be tested HERE is the join: that the vocabulary built off
`examples/biped/Biped_stack.usda` actually names that rig's controls, and
that the words an animator would say land on the prim paths the rig has.

What it asserts, on the biped's own 219 touch regions:

  * "left shoulder" resolves to L_Shldr's control prim path
  * bare "shoulder" with nothing selected resolves to the L/R PAIR
  * bare "shoulder" with a left control selected resolves to the left one
  * every layer is in the vocabulary -- body, face and both eyes -- which
    is the point of reading the scopes rather than the live TouchModel
  * no phrase in the grammar is a prim name

And, when a speech engine is present, the whole chain with NO MICROPHONE:
System.Speech's own synthesizer writes "left shoulder" into a wav, the
helper reads it back through SetInputToWaveFile, and the decision that
comes out is the shoulder. THAT IS SYNTHESIZED SPEECH, NOT A HUMAN
VOICE -- it measures the grammar and the plumbing, not a microphone, a
room or a person.

No Qt and no usdview: the adapter's vocabulary builder is a free
function over a stage for exactly this reason.

Usage: test_touchpose_voice.py
"""
import os
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
for _extra in (os.path.join(_ROOT, "plugin", "touchPose"),):
    if _extra not in sys.path:
        sys.path.insert(0, _extra)

from pxr import Usd  # noqa: E402

BIPED = os.path.join(_ROOT, "examples", "biped", "Biped_stack.usda")


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Core():
    """The voice subpackage, or None with a reason.

    Loaded the way the plugin loads it -- off its path, under the name
    `touchpose_voice` -- so a move or a rename is caught by this test
    rather than by an animator with the panel open. It cannot be
    imported as `touchpose.voice` here: `plugin/touchPose` carries a
    DIFFERENT package called `touchpose`, and it is already on the path.

    `touchPoseVoice` itself is not imported: it needs Qt, which a
    headless test has no business needing. Its path logic is repeated in
    three lines instead, and `_CheckTheAdapterAgrees` checks the two
    have not drifted.
    """
    import importlib.util

    root = os.environ.get("TOUCHPOSE_VOICE_PATH") or os.path.abspath(
        os.path.join(_ROOT, "..", "touchpose", "scripts", "touchpose",
                     "voice"))
    init = os.path.join(root, "__init__.py")
    if not os.path.isfile(init):
        return None, root
    if "touchpose_voice" in sys.modules:
        return sys.modules["touchpose_voice"], ""
    spec = importlib.util.spec_from_file_location(
        "touchpose_voice", init, submodule_search_locations=[root])
    module = importlib.util.module_from_spec(spec)
    sys.modules["touchpose_voice"] = module
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        del sys.modules["touchpose_voice"]
        return None, "%s (%s)" % (root, error)
    return module, ""


def _CheckTheAdapterAgrees():
    """The adapter computes this path too. They have to be the same one.

    Read out of the source rather than imported, because importing
    `touchPoseVoice` would pull in Qt. A cheap check that catches the one
    way this test can lie: passing against a path the plugin does not use.
    """
    adapter = os.path.join(_ROOT, "plugin", "touchPose", "touchPoseVoice.py")
    with open(adapter, "r", encoding="utf-8") as handle:
        text = handle.read()
    _Check('"touchpose", "scripts", "touchpose", "voice"' in text,
           "touchPoseVoice.py no longer looks for the subpackage where "
           "this test does")
    _Check('"TOUCHPOSE_VOICE_PATH"' in text,
           "touchPoseVoice.py no longer honours TOUCHPOSE_VOICE_PATH")


def _Entries(stage, Entry):
    """The adapter's vocabulary builder, without its Qt import.

    `touchPoseVoice.BuildEntries` is a free function over a stage for
    exactly this reason, but importing the module drags in
    `pxr.Usdviewq.qt`. The reader is called directly instead; it is the
    same code path the panel runs.
    """
    import touchPoseModel

    entries = []
    seen = set()
    for scope in touchPoseModel.FindRegionScopes(stage):
        group = touchPoseModel.LayerLabel(scope)
        for child in scope.GetChildren():
            if child.GetTypeName() != touchPoseModel.REGION_TYPE \
                    and not child.HasAttribute(touchPoseModel.FACES_ATTR):
                continue
            targets = touchPoseModel._TouchTargets(
                child, touchPoseModel.TYPED_CONTROL,
                touchPoseModel.CONTROL_REL)
            if not targets:
                continue
            control = str(targets[0])
            name = child.GetName()
            if name.endswith("_touch"):
                name = name[:-len("_touch")]
            if (control, name) in seen:
                continue
            seen.add((control, name))
            entries.append(Entry(control, name, group=group))
    return entries


def _Say(gram, text, lead=None, confidence=0.99):
    """Resolve a phrase the way one heard over the microphone resolves."""
    key = next((k for k, t in gram.choices() if t == text), None)
    _Check(key is not None, "%r is not in the biped's grammar" % text)
    return gram.resolve(key, confidence, lead_handle=lead)


def _ControlPath(entries, name):
    found = [e.handle for e in entries if e.name == name]
    _Check(len(found) == 1,
           "expected one %s region, found %d" % (name, len(found)))
    return found[0]


def main():
    _CheckTheAdapterAgrees()
    core, reason = _Core()
    if core is None:
        # A checkout with no TouchPose repo beside it cannot run this.
        # Said out loud and skipped, rather than passed quietly.
        print("SKIP: no touchpose_voice package: %s" % reason)
        return 0

    _Check(os.path.isfile(BIPED), "no biped at %s" % BIPED)
    stage = Usd.Stage.Open(BIPED)
    entries = _Entries(stage, core.Entry)
    print("%d touch regions -> vocabulary" % len(entries))
    _Check(len(entries) > 200,
           "expected the biped's full region set, got %d" % len(entries))

    # EVERY LAYER, not just the live one. TouchModel holds one at a time;
    # voice has to reach the eyes while the body is live.
    #
    # "Face" is NOT in this list any more, and its absence is the point:
    # the body carried two sets over the same mesh (TouchPose and
    # TouchPoseFace) and the import merged them,
    # measured as 0 overlapping region names and 0 faces claimed by both.
    # The assertion was left behind by that merge and failed on the
    # biped with ['Body', 'Eye L', 'Eye R'].
    groups = set(e.group for e in entries)
    for wanted in ("Body", "Eye L", "Eye R"):
        _Check(wanted in groups,
               "layer %r missing from the vocabulary (%s)"
               % (wanted, sorted(groups)))

    gram = core.Grammar(entries)
    print("%d phrases" % len(gram.choices()))

    # The grammar is WORDS. A prim name in it would mean the rig's
    # internals leaked into something a person is expected to say.
    for _key, text in gram.choices():
        _Check("_" not in text and "/" not in text,
               "grammar phrase is not English: %r" % text)

    # The rig publishes this control as L_Shldr, not shoulder_l: the
    # delivered names replaced the build names, so what an animator says
    # follows the published name too -- "left shoulder", not "left
    # shldr".
    shoulder_l = _ControlPath(entries, "L_Shldr")
    shoulder_r = _ControlPath(entries, "R_Shldr")
    _Check(shoulder_l.endswith("/L_Shldr"),
           "L_Shldr binds %s" % shoulder_l)

    # THE GOAL, in one line: an animator says "left shoulder". The
    # published name is L_Shldr; the vocabulary knows shldr is said
    # "shoulder", so the abbreviation never reaches a person's mouth.
    decision = _Say(gram, "left shoulder")
    _Check(decision.kind == core.SELECT,
           "left shoulder -> %s (%s)" % (decision.kind, decision.message))
    _Check(decision.handles == (shoulder_l,),
           "left shoulder -> %s, wanted %s"
           % (list(decision.handles), shoulder_l))
    print("  'left shoulder' -> %s" % shoulder_l)

    decision = _Say(gram, "right shoulder")
    _Check(decision.handles == (shoulder_r,),
           "right shoulder -> %s" % list(decision.handles))

    # Bare, with nothing selected: the PAIR.
    decision = _Say(gram, "shoulder")
    _Check(decision.kind == core.SELECT,
           "bare shoulder -> %s" % decision.kind)
    _Check(decision.handles == (shoulder_l, shoulder_r),
           "bare shoulder -> %s, wanted the pair" % list(decision.handles))
    print("  'shoulder'      -> the pair")

    # Bare, following the side of the lead selection.
    lead = _ControlPath(entries, "L_UpArm")
    decision = _Say(gram, "shoulder", lead=lead)
    _Check(decision.handles == (shoulder_l,),
           "shoulder after a left lead -> %s" % list(decision.handles))
    lead = _ControlPath(entries, "R_UpArm")
    decision = _Say(gram, "shoulder", lead=lead)
    _Check(decision.handles == (shoulder_r,),
           "shoulder after a right lead -> %s" % list(decision.handles))
    print("  'shoulder' follows the selection's side")

    # A face control and an eye control, while the body is the live
    # layer -- the thing reading the live TouchModel could not do.
    decision = _Say(gram, "left eye")
    _Check(decision.handles == (_ControlPath(entries, "L_Eye"),),
           "left eye -> %s" % list(decision.handles))
    decision = _Say(gram, "right cheek puff")
    _Check(decision.handles == (_ControlPath(entries, "R_CheekPuff"),),
           "right cheek puff -> %s" % list(decision.handles))
    print("  the face and the eyes are reachable too")

    # A half-heard phrase never moves the selection.
    decision = _Say(gram, "left shoulder", confidence=0.1)
    _Check(decision.kind == core.LOW_CONFIDENCE and not decision.handles,
           "a 10%% hit selected something: %s" % decision.kind)

    # The lip / lid pair that made `lid` be spoken "eyelid". Both have to
    # exist and they have to be different phrases.
    # The published names are L_LoLip / L_LoLid, which the vocabulary
    # reads as "left lower lip" and "left lower eyelid" -- the pair that
    # made `lid` be spoken "eyelid" in the first place.
    lip = _Say(gram, "left lower lip").handles
    lid = _Say(gram, "left lower eyelid").handles
    _Check(lip and lid and lip != lid,
           "lip and eyelid resolve to the same control")
    print("  lip and eyelid are different controls")

    _Speech(core, gram, entries, shoulder_l)
    print("OK")
    return 0


def _Speech(core, gram, entries, shoulder_l):
    """The whole chain with no microphone, when there is an engine.

    SYNTHESIZED, NOT A HUMAN VOICE: System.Speech's own TTS reads the
    phrase into a wav and the recogniser reads it back. It proves the
    helper, the grammar upload, the semantic id and the resolution join
    up on this rig's real vocabulary -- not how the recogniser handles a
    person.
    """
    import tempfile

    if sys.platform != "win32":
        print("SKIP: System.Speech is Windows only")
        return
    import importlib
    sapi = importlib.import_module("touchpose_voice.sapi")

    backend = sapi.SapiBackend()
    try:
        backend.start()
    except sapi.SapiUnavailable as error:
        print("SKIP: no speech helper: %s" % error)
        return
    try:
        loaded = backend.set_grammar(gram.choices())
        _Check(loaded == len(gram.choices()),
               "helper loaded %d of %d phrases" % (loaded,
                                                   len(gram.choices())))
        heard = []
        backend.add_event_sink(
            lambda e: heard.append(e) if e.get("event") == "heard" else None)

        folder = tempfile.mkdtemp(prefix="touchpose-voice-")
        wav = os.path.join(folder, "left-shoulder.wav")
        backend.say("left shoulder", wav)
        backend.use_wav(wav)
        backend.listen()
        _Check(backend.wait_for(("idle",), 60.0) is not None,
               "the recogniser never finished the file")
        _Check(heard, "nothing was recognised in the synthesized phrase")

        hit = heard[0]
        decision = gram.resolve(hit["id"], hit["confidence"])
        _Check(decision.handles == (shoulder_l,),
               "spoken 'left shoulder' -> %s (heard %r at %.2f)"
               % (list(decision.handles), hit.get("text"),
                  hit.get("confidence", 0.0)))
        print("  SPOKEN (synthesized, not a human): 'left shoulder' -> %s "
              "at %.2f confidence over %d phrases"
              % (shoulder_l.rsplit("/", 1)[-1], hit["confidence"],
                 len(gram.choices())))
        try:
            os.remove(wav)
            os.rmdir(folder)
        except OSError:
            pass
    finally:
        backend.close()


if __name__ == "__main__":
    sys.exit(main())
