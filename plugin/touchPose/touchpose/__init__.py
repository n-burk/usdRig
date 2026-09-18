"""TouchPose for RigExec: the rendered skin is the picker.

Hover a region of the character's mesh and it lights up; click it and the
rig control that owns that region is selected. This package is the port
of that idea onto RigExec + usdview, and it is a PLUGIN: it depends on
RigExec's public surfaces only -- the `rigexec` Python API, the imaging
ctypes entry points reached through `rigExecUsdview.ImagingLibraryPath()`,
and usdview's own data model. Nothing here reaches into `libs/`.

The layering, bottom up, is the whole design:

    touchfile   .touch -> records          no pxr, no Qt, no ctypes
    usdexport   records -> GeomSubsets     pxr only
    (later) imaging     overlay            ctypes only
    (later) model       hover/pick state   Qt-free
    (later) ui          the usdview panel  Qt

Each layer is testable with the ones above it absent, which is what keeps
this a plugin rather than a fork. The the conventional tool product it descends from was
draw-overrides, an app-wide Qt event filter and DG-pull tricks, none of
which ports; what ports is the DATA (a face list, a control binding, a
palette index per region) and the interaction.
"""
