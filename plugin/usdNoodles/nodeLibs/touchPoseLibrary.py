#
# TouchPose node library for usdNoodles.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#

"""Node descriptions for TouchPose regions and their group scope.

TouchPose prims are typeless-as-far-as-the-graph-is-concerned: they are
`Scope`s carrying `touchpose:` properties (see ``UsdNoodles.touchPose``).
The generic `UsdPrimLibrary` already turns them into nodes and already
draws `touchpose:control` as a link -- measured, not assumed, on the
shipped biped: 98 regions, 98 relationship links. What it cannot know is
which of the rows are the data:

    MEASURED on /Biped/TouchPose/thumb_002_r_touch, before this library:
        proxyPrim, purpose, visibility, touchpose:color,
        touchpose:control, touchpose:elementType, touchpose:faces,
        touchpose:hilight

Three of the eight rows are `UsdGeomImageable`'s, unauthored on every one
of the 98 regions, and they are the three a rigger scanning a column of
region nodes has to read past. This library claims TouchPose prims ahead
of the catch-all (priority 100 vs 1000) and keeps only the `touchpose:`
rows, which the UI then nests under one foldable `touchpose` header.

`touchpose:faces` STAYS. It is 144 to 1,362 ints per region (16,739 over
the asset) and the obvious worry is that the editor renders the array:
it does not. Pin rows are names and type labels only -- the only
attribute VALUES usdNoodles reads anywhere are `ui:nodegraph:*` and
`info:implementationSource` -- so the row costs one `int[]` label, and
dropping it would hide the one property that says what a region IS.
"""

from __future__ import annotations

from pxr import Gf, Sdf, Tf, Usd

from ..touchPose import is_touch_group, is_touch_prim, is_touch_region, NAMESPACE
from .usdTypedPrimBase import UsdTypedPrimLibraryBase


def _read_noodles_config(key: str, default: object) -> object:
    """Read a NoodlesConfig value, tolerating headless environments.

    ``NoodlesConfig`` imports Qt at module load; descriptor creation has
    to work in batch tooling and unit tests, so fall back to the caller's
    default when Qt is not importable.
    """
    try:
        from ..noodlesConfig import NoodlesConfig
    except ImportError:
        return default

    return NoodlesConfig.get(key, default)


class TouchPoseLibrary(UsdTypedPrimLibraryBase):
    """Claims TouchPose region and group prims for the node graph."""

    def get_name(self) -> str:
        return "TouchPose"

    def get_families(self) -> list[str]:
        """No creatable node types.

        TouchPose prims are authored by the importer or by painting in the
        viewport, from face indices that only exist there. A hotbox entry
        that made an empty region bound to nothing would be a trap, so
        this library describes prims and creates none.
        """
        return []

    def get_node_types(self, family: str) -> list[dict[str, str]]:
        return []

    def create_node(
        self,
        stage: Usd.Stage,
        parent_path: Sdf.Path,
        identifier: str,
        position: Gf.Vec2d,
    ) -> Sdf.Path | None:
        return None

    def can_handle_prim(self, prim: Usd.Prim) -> bool:
        return is_touch_prim(prim)

    def create_node_descriptor_from_prim(
        self, prim: Usd.Prim, stage: Usd.Stage
    ) -> dict[str, object] | None:
        if not self.can_handle_prim(prim):
            return None

        descriptor: dict[str, object] = {
            "primPath": prim.GetPath(),
            "inputPins": [],
            "outputPins": [],
            "inputPinTypes": {},
            "outputPinTypes": {},
            "uiStyle": _read_noodles_config("nodeRendererType", "default"),
            "renderer": None,
        }
        # Built by the shared discovery first and then narrowed, rather
        # than enumerated here: the shared path is what decides pin sides,
        # dual pins and namespace grouping, and a second enumeration would
        # be a second set of rules to keep in step with it.
        from .._schema_pin_names import populate_descriptor_pins

        populate_descriptor_pins(prim, descriptor, check_port_type=True)
        _keep_touchpose_pins(descriptor)
        return descriptor

    def get_prim_ui_style(self, prim: Usd.Prim) -> str | None:
        if not self.can_handle_prim(prim):
            return None
        return _read_noodles_config("nodeRendererType", "default")

    # -- what the graph-population paths ask for --------------------------

    @staticmethod
    def is_group_prim(prim: Usd.Prim) -> bool:
        """Whether this prim stands for a set of nodes rather than one node."""
        return is_touch_group(prim)

    @staticmethod
    def is_region_prim(prim: Usd.Prim) -> bool:
        return is_touch_region(prim)


def _keep_touchpose_pins(descriptor: dict) -> None:
    """Drop every row that is not TouchPose data.

    What survives: the `touchpose:` properties themselves. What goes:
    `UsdGeomImageable`'s proxyPrim / purpose / visibility, which are
    schema rows no TouchPose file authors and which would otherwise be
    three-eighths of every one of the 98 region nodes.
    """
    for side in ("inputPins", "outputPins"):
        kept = [name for name in descriptor[side] if name.startswith(NAMESPACE)]
        descriptor[side] = kept
        types_key = "inputPinTypes" if side == "inputPins" else "outputPinTypes"
        descriptor[types_key] = {
            name: value
            for name, value in descriptor[types_key].items()
            if name in set(kept)
        }


Tf.Type.Define(TouchPoseLibrary)
