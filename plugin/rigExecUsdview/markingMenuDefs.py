#
# RigExec usdview marking menus: the two built-in menus, as DATA.
#
# Nothing in this file runs anything. Every item is a record of the kind
# markingMenuModel understands -- an id, a label, a slot, and the NAMES
# of the predicates and state it reads -- and the ids are the whole
# contract with the layer above: the widget hands an id back to the
# plugin and the plugin decides what an id does. That split is why this
# file has no imports beyond the model's own constants, and why a menu
# eventually authored in a USD layer will be able to replace it without
# anything else moving.
#
# The two menus are the two halves of "what do I want from the thing in
# front of me": SELECTION acts on what is selected, MODES changes how the
# tool behaves. Both pin all eight slots. Pinning matters more here than
# anywhere else in the codebase: a marking menu earns its keep only once
# the hand knows that West is Key without looking, and an item that
# shuffles into a different slot because something else was hidden
# destroys exactly that. So every ring item names its direction, and the
# only things that move are the ones that go to the overflow column,
# which is read, not flicked.
#
# STUBS. Three items are declared here that have no action behind them
# yet; each is marked `stub=True` and called out in a comment where it is
# declared. Stage 2 wires the rest and must decide, for each of these,
# whether to implement it or drop it:
#
#   selection.counterpart   -- "Select counterpart" (the mirrored control)
#                              needs a naming convention or an authored
#                              pairing on the rig; neither exists yet.
#   selection.addToPicker   -- "Add to picker" needs a picker edit path;
#                              pickerScene reads, it does not write.
#   modes.evaluation        -- the Evaluation submenu's children need the
#                              engine to expose its modes by name.
#
# A stub item still resolves and still draws; `ResolvedItem.stub` is how
# the widget can grey it out or badge it rather than silently doing
# nothing when it is picked.
#
try:
    from markingMenuModel import ACTION, RADIO, SUBMENU, TOGGLE, Menu
except ImportError:                    # loader that did not add our dir
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from markingMenuModel import ACTION, RADIO, SUBMENU, TOGGLE, Menu


# --- Predicate and state names ---------------------------------------
#
# Every string a menu item points the context at, gathered here so the
# selection-summary object on the other side has one list to satisfy and
# a typo shows up as a name in this file that nothing answers.
#
# Predicates (`when`), asked through context.Test(name):
#   limb        the selection sits on a limb that has an IK/FK switch
#   hasSpaces   the selection has more than one space to live in
#   control     the selection is a rig control at all
#
# State (`read`), asked through context.Read(name):
#   limbIsIk        is that limb currently on IK rather than FK
#   guides          are the guide drawings on
#   touchPoseLive   is TouchPose driving the rig live
#   touchPosePaint  is the TouchPose paint tool active
#   focusIsPicker   does the picker hold the keyboard, not the viewport
#   writeMode       which layer edits land in: session / edit / rig
#   toolVisible     are the viewport tool overlays drawn
#
# Dynamic children (`children`), asked through context.Children(name):
#   spaces      the spaces the selected control can switch to
#   tools       the manipulators available right now
#   evaluation  the evaluation modes the engine offers (stub)


SELECTION = Menu(
    "selection",
    label="Selection",
    items=[
        # W and E are the two most-used commands in an animation session,
        # and they take the two easiest flicks. Key first: it is the one
        # an animator makes hundreds of times an hour.
        dict(id="selection.key", label="Key", kind=ACTION, dir="W",
             write="KeySelection"),
        dict(id="selection.reset", label="Reset to rest", kind=ACTION,
             dir="E", write="ResetToRest"),
        # Zero pose and Frame are the vertical pair: both are "put things
        # back where I can see them" and neither is destructive.
        dict(id="selection.zero", label="Zero pose", kind=ACTION, dir="S",
             write="ZeroPose"),
        dict(id="selection.frame", label="Frame", kind=ACTION, dir="N",
             write="FrameSelection"),
        # The two conditional items take diagonals, because a diagonal is
        # the flick people miss and these are the two that may not be
        # there: missing one costs nothing mid-flick (it is hidden), and
        # in click mode it is greyed out with the reason visible.
        dict(id="selection.fkik", label="FK/IK toggle", kind=TOGGLE,
             dir="NW", when="limb", read="limbIsIk", write="ToggleFkIk"),
        dict(id="selection.space", label="Space", kind=SUBMENU, dir="NE",
             when="hasSpaces", children="spaces"),
        # STUB: no action yet. "Select counterpart" means the mirrored
        # control, and nothing on the rig says which control mirrors
        # which -- the biped's names imply it, the schema does not.
        dict(id="selection.counterpart", label="Select counterpart",
             kind=ACTION, dir="SW", when="control",
             write="SelectCounterpart", stub=True),
        # STUB: no action yet. pickerScene reads pickers off the stage;
        # writing a button back into one is unbuilt.
        dict(id="selection.addToPicker", label="Add to picker",
             kind=ACTION, dir="SE", when="control", write="AddToPicker",
             stub=True),
        # The stacked column beside the ring, and declared so rather
        # than merely left over: usdview's own prim context menu is long,
        # rarely wanted and impossible to flick at, and it must not be
        # promoted into the ring on the days the two conditional items
        # are hidden.
        dict(id="selection.usdviewMenu", label="usdview prim menu…",
             kind=ACTION, overflow=True, write="ShowUsdviewPrimMenu"),
    ])


MODES = Menu(
    "modes",
    label="Modes",
    items=[
        # The three toggles that change what the viewport is doing take
        # W, E and S: they are flicked blind, mid-pose, and they are the
        # reason this menu exists at all.
        dict(id="modes.guides", label="Guides", kind=TOGGLE, dir="W",
             read="guides", write="SetGuides"),
        dict(id="modes.touchPoseLive", label="TouchPose live", kind=TOGGLE,
             dir="E", read="touchPoseLive", write="SetTouchPoseLive"),
        dict(id="modes.touchPosePaint", label="TouchPose paint",
             kind=TOGGLE, dir="S", read="touchPosePaint",
             write="SetTouchPosePaint"),
        # Focus is a two-state flip rather than a checkbox: the label the
        # widget draws should say where focus will GO, which is what
        # context.Label is for. It is declared as a TOGGLE reading
        # `focus` so the check mark means "the picker has it".
        dict(id="modes.focus", label="Picker/Viewport focus", kind=TOGGLE,
             dir="N", read="focusIsPicker", write="ToggleFocus"),
        dict(id="modes.write", label="Write mode", kind=SUBMENU, dir="NW",
             items=[
                 # A radio group: all three read the same piece of state
                 # and the one whose value matches is the checked one.
                 dict(id="modes.write.session", label="Session layer",
                      kind=RADIO, read="writeMode", value="session",
                      write="SetWriteMode"),
                 dict(id="modes.write.edit", label="Edit layer",
                      kind=RADIO, read="writeMode", value="edit",
                      write="SetWriteMode"),
                 dict(id="modes.write.rig", label="Rig layer", kind=RADIO,
                      read="writeMode", value="rig", write="SetWriteMode"),
             ]),
        # The tool list is whatever the gizmo offers at this moment, so
        # it is fetched when the submenu unfolds rather than written out.
        dict(id="modes.tool", label="Tool", kind=SUBMENU, dir="NE",
             children="tools"),
        # STUB: no action yet. The evaluation modes exist in the engine
        # but nothing names them for a UI, so the provider has nothing to
        # return and this opens empty until stage 2 gives it a list.
        dict(id="modes.evaluation", label="Evaluation", kind=SUBMENU,
             dir="SW", children="evaluation", stub=True),
        dict(id="modes.toolVisibility", label="Viewport tools",
             kind=TOGGLE, dir="SE", read="toolVisible",
             write="SetToolVisibility"),
    ])


# Every built-in menu by id, so a caller binds a key to a NAME and the
# binding survives the menus being rewritten.
MENUS = {SELECTION.id: SELECTION, MODES.id: MODES}

# The items declared with no action behind them. Kept as a list, not
# just a flag on each record, so stage 2 has one thing to read and one
# thing to empty.
STUBS = ("selection.counterpart", "selection.addToPicker",
         "modes.evaluation")
