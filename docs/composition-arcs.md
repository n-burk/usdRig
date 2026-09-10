# Guided composition arcs (usdview)

Added 2026-09-08. Editing extended to the arcs that already exist
2026-09-09.

The Layer Opinions panel authors composition arcs and edits the ones a
prim already has. Right-click anywhere in its tree and take **Add
Composition Arc**:

| Item | Authors | Where |
|---|---|---|
| Reference… | `references` | the selected prim |
| Payload… | `payload` | the selected prim |
| Inherit… | `inheritPaths` | the selected prim |
| Specialize… | `specializes` | the selected prim |
| Variant Set… | `variantSetNames`, the variant specs, `variantSelection` | the selected prim |
| Sublayer… | `subLayerPaths` | the **layer** |
| Relocate… | `relocates` | the **layer** |

Each opens its own guided dialog. The submenu is offered from every part
of the tree, including the empty space below the rows — a prim with no
opinions yet has nothing to right-click, and that is exactly when you
want to give it a reference.

Right-clicking a *layer* group aims the flow at that layer. Otherwise it
opens on usdview's own edit target, which is where every other edit in
the application goes.

## Arcs that are already there

An arc field holds a *list* of arcs, so the panel shows it as one. A
prim with two references gets a `references` heading with a row under it
per reference, labelled the way usda writes it — `prepend references
[0]` — and showing the arc itself:

```
▼ ArmRig.usda  [edit target]
    ▼ subLayers                             1 layer
         subLayer [0]        @./lighting.usda@ (offset = 12; scale = 2)
    ▼ references                            2 items
         prepend references [0]  @./hand.usda@</Hand> (offset = 5)
         prepend references [1]  </_class_Ctrl>
      kind                                  "component"
```

Each of those rows can be:

- **retyped in place**, by double-clicking the value — the text is usda
  and is parsed back through Sdf, so `@./hand.usda@</Palm> (offset = 3)`
  is a valid edit and `not a reference` is a parse error that authors
  nothing;
- **reopened in its own guided flow**, from *Edit Reference…* on its
  right-click menu — the same dialog that would have added it, prefilled
  with the arc it holds;
- **removed on its own** (*Remove This Entry*), which is not the same act
  as deleting the layer's whole say about references and is not called
  the same thing;
- **moved** against the arcs beside it (*Move Stronger* / *Move Weaker*).

The heading itself is still one opinion: deleting it removes the field.

### Arcs that belong to the layer

`subLayers` and `relocates` are the layer's composition, not the prim's,
and this panel is the only place they can be seen. They are listed at the
top of their layer's group, and only for layers in the stage's own local
layer stack — a referenced asset's sublayers are not this stage's
composition, and listing them under the prim they arrive at would invite
editing the asset for every shot that uses it. A layer with neither
grows no rows for them, so the common case adds nothing.

They are only visible under a group that is *shown*, and a group appears
only when its layer holds a spec for the selected prim. A layer that
sublayers others but says nothing about this prim contributes nothing to
it, and does not appear.

### What editing drops, and why

The edit dialog is the add dialog. Same fields, same explanations, same
refusals, same live preview — the difference is one attribute on the
context (`ArcContext.editRow`), so a second form cannot drift from the
first.

Two fields go, because neither is a question once the arc exists:

- **Author into.** The arc is in the layer it is in. Aiming the dialog
  somewhere else would write a *copy* there and leave the original where
  it was. The dialog states the layer instead.
- **Position.** The item keeps the arm and index it already occupies, so
  retyping an asset path cannot silently re-rank the arc against the ones
  beside it. Reordering is *Move Stronger* / *Move Weaker* in the panel,
  where the order is visible.

A move stays inside its own arm. `prepend` and `append` are different
opinions — moving an arc from one to the other flips it from winning over
the existing arcs to losing to them — and that is a decision, not a
nudge.

**Variant sets are added but not edited.** A row of `variantSetNames` is
the *name* of a set, not the set: the variants are `VariantSetSpec`s
hanging off the prim spec, and retyping the name would leave those behind
under the old name while declaring one that has none. The name can still
be removed, the *selection* is edited on its own row, and the variants
themselves are authored by selecting one and editing the prim.

### Two rules that invert when editing

Both are refusals when adding and the status quo when reopening, so both
excuse the entry being edited — and only that entry:

- a layer **already sublayering** that path. Reopening that very entry to
  correct its offset must not be refused for already being there.
- a relocate's **source and target**. The stage already reflects the
  relocate being edited: the prim is not at its source path any more
  (this relocate moved it) and something *does* exist at its target (this
  relocate put it there). Asked of the entry being edited, "the target
  already exists" and "nothing composes at the source" would refuse every
  edit, including one that changes nothing. The consistency checks
  against the *other* relocates still apply in full — USD ignores both
  halves of a chain, so an edit that made one would break a relocate that
  was working.

### Where the values come from

Every value in these rows round-trips through usda, in both directions.
An item's text is produced by giving a scratch spec exactly that one item
and exporting it; the text typed back is parsed by importing it. Nothing
renders or parses a `Reference`, a layer offset or a relocate by hand, so
what the panel shows is what the file would say, and a typo is a parse
error rather than a value that quietly differs. It is also where the row
labels come from: the keyword is not the info key — `inheritPaths` is
written `inherits` and `variantSetNames` is written `variantSets` — and a
row labelled otherwise would not match the file.

One consequence worth stating: removing the **last** entry of an arc
field clears the field rather than leaving it empty. An emptied *explicit*
list reads `references = None`, which is an authored opinion that blocks
every weaker reference, and the other arms leave a field that exports as
nothing but still answers `HasInfo`. "This layer no longer says anything
about references" is what removing the last arc means.

### Which rows are struck through

A list op that is **not** explicit does not shadow the weaker layers'
opinions of the same field — it composes with them. A stronger
`prepend references` and a weaker one both contribute, in that order, so
the weaker row is not marked as overridden. Only an explicit list
(`references = [...]`) replaces what is underneath, and that one does
strike out the rows below it.

## What makes the flows guided

Composition is the part of USD people get wrong, and the failure is
never a typo — it is picking a reference when the situation wanted an
inherit, or prepending when the intent was to lose to what is already
there. So each dialog carries three things a plain form does not:

- **The arc's own paragraph**, above the fields. "Reference vs inherit
  vs specialize" is the real question, and no field label answers it.
- **A sentence under every field** saying what that field does, and what
  the non-obvious choice means. Prepend/append is described in terms of
  who wins, not in terms of list order.
- **The usda that will be authored**, live, with the lines the arc adds
  highlighted and scrolled into view. The seed includes the prim's
  existing ancestors exactly as the layer has them, so an untouched
  `def "Rig"` is not previewed as a new `over` and not highlighted as
  something the arc is adding.

The preview is not a description of the authoring. It is the same
`_Apply` function run against a scratch layer seeded with the spec the
chosen layer already holds, then exported — so it cannot drift from what
lands, and an arc *prepended* ahead of existing ones shows in its real
position. The highlight comes from diffing that against the same seed
with nothing applied.

### Where "Target prim" gets its list

For an **external** reference or payload, from the asset — not from the
stage. "Which prim inside the target to compose" is a question about the
layer being referenced, and the stage being edited does not contain
those prims; that is the point of the arc. Seeded from the stage the
combo offers the one list that cannot hold the answer, and on a shot
file that has not been assembled yet that list is a single root prim.

So the combo refills every time the asset path changes — the only field
in the dialog that does. It is refilled in place rather than by
rebuilding the form, which would take the keyboard focus out of the
field being typed into.

The asset's paths are read off its **layer**, not by opening it as a
stage. An asset layer need not compose on its own — one made of `over`s
composes nothing at all — and a combo that could not offer those prims
would be empty for exactly the assets that most need naming. Only
`nameChildren` is descended, so nothing inside a variant is offered: an
arc's target must be a plain prim path, and `/A{lod=hi}B` is not one.
The list is bounded, like the stage's, and the field stays typeable
either way.

The layer's **defaultPrim** is promoted to the top of the list, because
that is what leaving the field empty means. An asset that declares none
warns — an arc with no target prim would compose nothing — and the
answer is to name a prim from this list.

Three cases fall back to the stage's own prims, because there the stage
really is the target or the best guess available: an **internal** arc,
and an asset path that is still **empty**. An asset path that is typed
but does not **resolve** offers nothing instead: the asset may not exist
yet, which is legal to author, and listing this stage's prims as though
they were its contents would be a lie rather than a fallback.

## Refusals and warnings

They are different things and the dialog treats them differently.

A **refusal** disables Author and says why. These are requests USD would
reject or that produce a stage nobody wanted: a prim targeting itself, an
ancestor **or a descendant** (all three are `ErrorArcCycle`), a relative
target path, a non-finite time offset or scale, a zero time scale, a
variant set with a non-identifier name, a layer sublayering itself **or
any layer that already pulls it in** (`ErrorSublayerCycle`), and four
relocate cases — a root prim as the source (Pcp
ignores it), a target that already exists, a target whose parent does
not, and a source that no composition arc brought in.

That last one is worth stating plainly, because USD gives no error for
it. A relocate only moves what a composition arc brought in. Applied to
a prim no arc introduced, it removes the prim from its old path,
composes nothing at the new one, and reports nothing — the prim simply
disappears. The flow refuses it and says to rename the spec instead.
Note that "introduced by an arc" is asked of the prim *index*
(`Usd.PrimCompositionQuery`), not of which layers hold its specs: an
**internal** reference targets this same layer stack, so every spec is
local while the prim is very much arriving through an arc.

Relocates also have to stay consistent with each other. USD does not
follow a chain — authoring `/D/One → /D/X` beside an existing
`/D/X → /D/Y` is a conflict, and Pcp then ignores *both*, so the
relocate that was already working stops working. Those combinations are
refused rather than allowed to break what is there.

Reparenting, on the other hand, **is** legal: a relocate may move a prim
under a different parent, not only rename it in place. That warns rather
than refusing.

A **warning** is shown and does not block. Authoring an arc to an asset
that does not exist yet is a normal thing to do, and a dialog that
forbids it is wrong. So is targeting a class prim that has not been built
yet. The warnings that fire:

- the asset does not resolve from the layer being authored into;
- it resolves but declares no `defaultPrim`, and no target prim was named
  (the arc would compose nothing);
- an inherit or specialize target is not a `class` prim, or does not
  exist on the stage;
- a variant set is being authored with no variants, or with no selection;
- a variant name is not an Sdf identifier (legal, and common — `8k`,
  `foo-bar` and `.abc` are all valid variant names; only the **set** name
  must be an identifier);
- the set already exists, so the new variants are being *added* to it;
- a relocate's source does not compose on the stage at all;
- a relocate moves a prim under a different parent rather than renaming
  it in place.

Warnings that survive to Author are repeated in the panel's status line
afterwards, as a record of what was accepted.

## Which layers may be authored into

Only the stage's **local layer stack** — session, root, and its
sublayers — and only those that will take an edit. A muted layer is not
in that stack, so it is not offered.

This matters because the panel groups a prim's opinions by every layer in
its prim stack, and that includes layers reached *through* a reference.
There, `OpinionRow.specPath` is the referenced prim's path, not the
prim's stage path. Authoring an arc into one of those would edit the
referenced asset for every place it is used, which is never what "add a
reference to this prim" meant. `AuthoringLayers()` is that rule, and
`Author()` re-checks it rather than trusting the dialog's combo — the
combo is a convenience, the rule is a correctness property.

usdview's own edit target is used as the default **only when its mapping
is the identity**. A variant edit target maps `/A/B` to `/A{v=x}B`, and
these flows author at the composed prim path; taking just its layer would
put the arc on the outer prim, where it would apply to every variant
instead of the one being edited.

## Undo

Every flow returns a `rigExecUndo.Edit` that the panel pushes onto the
same shared stack as its inline edits, so `Ctrl+Z` from anywhere in the
plugin takes an arc back out.

The prim-scoped arcs snapshot the whole prim spec through
`layerOpinionsModel.SpecCopySnapshot`, which is `Sdf.CopySpec` under the
hood and therefore carries variant sets and their contents without this
code enumerating them.

Two things that snapshot has to get right, and that a naive
remove-and-copy does not:

- **Sibling order.** `Sdf.CopySpec` *appends*, so restoring a spec into
  the middle of its parent lands it at the end: `B,C,D` comes back as
  `B,D,C`. In RigExec that is not cosmetic — mover evaluation order **is**
  sibling order (README: "the bottom sibling fires first") — so an undo
  would silently change which mover runs when. `Restore` records the
  siblings that followed and pushes them back behind the restored spec.
- **The ancestors the arc created.** Authoring into a layer holding
  nothing for the prim creates the whole chain as overs. The snapshot is
  taken at the *highest* path the arc will create, so undo removes the
  chain rather than leaving `over Rig { over IK {} }` behind for the
  panel to list forever as a layer with no opinions.

The two layer-scoped arcs need their own snapshots, because neither is a
spec:

- `SublayerSnapshot` captures `subLayerPaths` **and** `subLayerOffsets`.
  Assigning `subLayerPaths` resets every offset to identity, so restoring
  the paths alone would silently un-retime a sublayer that was already
  there.
- `RelocatesSnapshot` captures the layer's `relocates`.

Redo restores the spec as it was *when the arc was authored*. Anything
authored inside a variant afterwards does not come back with it — that is
a later entry on the stack, and undo is last-in-first-out.

## Relocates in USD 26.08

A relocate composes only as **layer** metadata. The older
per-prim `relocates` field still parses and still round-trips through
`Sdf`, but Pcp ignores it: authoring it moves nothing. The flow authors
`layer.relocates`, and the dialog says the arc is on the layer rather
than on the selected prim.

## Writing a list-op arc

References, payloads, inherits, specializes and `variantSetNames` are
`SdfListOp` fields, and they are written through the proxy's
`Prepend()` / `Append()` operations — never by appending to
`prependedItems`. Writing an arm directly is both destructive and
mis-ordered:

- an **explicit** opinion (`references = [@a@, @b@]`) is thrown away the
  moment `prependedItems` is written. The result is
  `prepend references = @new@`, and the two arcs that were there are
  gone;
- appending to `prependedItems` puts the new arc *after* the prepends
  already there — the opposite of what "Prepend (stronger than existing
  arcs)" promises.

## Where the code is

| File | Holds |
|---|---|
| `plugin/rigExecUsdview/compositionArcsModel.py` | every rule: the fields each arc asks for, what is refused, what warns, the preview, the authoring and the re-authoring, the snapshots. Which arc a panel row is (`ArcForRow`). Imports no Qt. |
| `plugin/rigExecUsdview/compositionArcsUI.py` | one dialog class that builds its form from the arc's `Field` descriptors and serves both adding and editing, plus the submenu builder and `RunArcEditFlow`. |
| `plugin/rigExecUsdview/layerOpinionsModel.py` | expanding an arc field into one row per arc, formatting and parsing an entry, and the writes that set, remove and reorder one. Imports no Qt. |
| `plugin/rigExecUsdview/layerOpinionsUI.py` | `_AddArcMenu` / `_OnArcAuthored`: raising the submenu and pushing what it returns. `_AddRowActions` / `_EditArc`: the per-arc menu and the route into the edit flow. |

Adding a field — or a whole arc — is a change to the model and nothing
in the Qt file. The split is the same one
[the panel itself uses](../plugin/rigExecUsdview/layerOpinionsModel.py):
rules in a headless module, Qt as a thin driver.

## Tests

```sh
bin/run_python_tests.sh test_composition_arcs_model test_layer_opinions_model
bin/run_testusdview_arcs.sh                   # the dialogs in real usdview
```

Both also run under ctest (`testCompositionArcsModel`,
`testLayerOpinionsModel`), and both helpers have `.bat` twins for Windows:
`bin\run_python_tests.bat`, `bin\run_testusdview_arcs.bat`.

The headless tests cover every authoring rule against in-memory layers,
and the row expansion, formatting, parsing and reordering that editing
rests on. One of them is worth naming: reopening each editable arc and
applying it untouched must leave the layer byte-identical — the property
that makes the prefill trustworthy, since a disagreement anywhere between
reading an arc back and writing it would show up as an untouched edit
changing the file.

The `testusdview` script covers what they cannot see: that the right-click
menu offers the arcs, that the form rebuilds when a choice hides a field,
that a refusal disables Author instead of throwing on commit, that the
Target prim combo refills from the asset as its path is typed and keeps
what was typed into it, that the preview re-renders and highlights the
changed line, that an existing arc
is listed as a row of its own with the flow, the removal and the moves on
its menu, that reopening it prefills the dialog without a layer or a
position field, and that authoring — new or edited — reaches the stage
and the panel's undo stack.

Set `RIGEXEC_ARCS_SHOT=path.png` on the runner to keep a grab of the
reference flow.
