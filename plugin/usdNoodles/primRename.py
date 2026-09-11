#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#

"""Renaming a prim, and everything a rename implies.

A prim's name is its identity in the namespace, so changing it is never a
one-field edit. Everything that named the old path has to move with it:

  * its children, and their children, all the way down;
  * relationship targets pointing at it or at anything beneath it;
  * attribute connections pointing at its properties;
  * internal references, inherits, specializes and payloads that target it.

Hand-authoring that set is where renaming goes wrong, because the failure is
silent -- a missed relationship target does not error, it just stops resolving,
and the rig or shading network quietly loses a connection. So none of it is
hand-authored here. ``Usd.NamespaceEditor`` performs the whole edit as one
namespace operation and does the fixups itself, which is exactly the reason it
exists.

**Relocates are deliberately not authored.** A prim that arrives across a
reference cannot be renamed in its own layer -- the opinion lives in the
referenced layer, and the only way to express the rename locally is a
``relocates`` entry on the root layer. That is a heavier composition concept
than a rename looks like from the UI, so the edit is refused and USD's own
explanation is handed back to the caller to show. Nothing is authored on a
refusal.
"""

from pxr import Sdf, Usd

# rename_prim outcomes.
RENAMED = "renamed"
UNCHANGED = "unchanged"  # the new name equals the old one; nothing authored
INVALID_NAME = "invalidName"
NAME_TAKEN = "nameTaken"
REFUSED = "refused"  # USD declined; the message is its own whyNot


def validate_new_name(prim, new_name):
    """Check *new_name* for *prim* without touching the stage.

    Cheap and side-effect free, so a UI can call it per keystroke to decide
    whether to let the user commit.

    Args:
        prim: The Usd.Prim being renamed.
        new_name: Candidate name.

    Returns:
        tuple[str, str]: (status, message). Status is RENAMED when the name
        would be accepted -- the caller has not renamed anything yet -- or
        UNCHANGED, INVALID_NAME or NAME_TAKEN.
    """
    if not prim or not prim.IsValid():
        return INVALID_NAME, "no prim to rename"

    if not new_name:
        return INVALID_NAME, "a prim name cannot be empty"

    if new_name == prim.GetName():
        return UNCHANGED, ""

    # USD prim names are identifiers: no spaces, no dashes, no leading digit,
    # and no namespace separator (that is a property-name concept).
    if not Sdf.Path.IsValidIdentifier(new_name):
        return (
            INVALID_NAME,
            f"'{new_name}' is not a valid prim name (letters, digits and "
            "underscores; cannot start with a digit)",
        )

    parent = prim.GetParent()
    if parent and parent.IsValid():
        sibling = parent.GetChild(new_name)
        if sibling and sibling.IsValid():
            return NAME_TAKEN, f"'{new_name}' already exists here"

    return RENAMED, ""


def rename_prim(stage, prim, new_name):
    """Rename *prim* to *new_name*, fixing up everything that referred to it.

    Args:
        stage: The Usd.Stage owning *prim*.
        prim: The Usd.Prim to rename.
        new_name: The new prim name.

    Returns:
        tuple[str, Sdf.Path | None, str]: (status, newPath, message). newPath
        is filled only on RENAMED. On any other status nothing was authored.
    """
    status, message = validate_new_name(prim, new_name)
    if status != RENAMED:
        return status, None, message

    oldPath = prim.GetPath()
    newPath = oldPath.GetParentPath().AppendChild(new_name)

    # Relocates are opt-OUT, not opt-in: Usd.NamespaceEditor.EditOptions()
    # defaults allowRelocatesAuthoring to True, so constructing the editor
    # with no options quietly authors a relocates map the moment a rename
    # touches a prim that arrives across a reference. Turned off explicitly.
    options = Usd.NamespaceEditor.EditOptions()
    options.allowRelocatesAuthoring = False

    editor = Usd.NamespaceEditor(stage, options)
    editor.RenamePrim(prim, new_name)

    # Ask before authoring. CanApplyEdits is what reports the relocates case,
    # and its whyNot is a better message than anything worth writing here --
    # it names the actual composition reason.
    canApply = editor.CanApplyEdits()
    if not canApply:
        return REFUSED, None, canApply.whyNot or "USD declined the rename"

    if not editor.ApplyEdits():
        return REFUSED, None, "the rename could not be applied"

    return RENAMED, newPath, ""
