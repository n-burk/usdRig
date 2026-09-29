# View cube

**RigExec → Viewport → View Cube** toggles a camera-orientation widget in the
viewport's upper-right corner. Click a face, edge, or corner to orbit the free
camera. The cube supports 6 faces, 12 edges, and 8 corners. Only visible faces
can be selected.

An orbit preserves the camera's center and distance and clears roll. A top or
bottom view snaps the current heading to the nearest 90 degrees. The labels
follow the stage up axis: front is +Z for Y-up stages and -Y for Z-up stages;
right is +X, and top is the stage's up axis.

Drag the cube to tumble. A four-logical-pixel threshold separates clicking
from dragging. The home button frames the current selection (or whole stage)
and moves to the front-top-right view. Hover highlights follow camera motion.
Alt/Meta gestures and non-left buttons remain available to usdview.

Orbits use a 250 ms smoothstep transition by default. A new orbit, drag, or
stage replacement cancels an active transition. A click while viewing through
a camera prim switches to the free camera first. No layer is edited.

The widget uses `QPainter`; projection, region picking, and orientation math
live in `plugin/rigExecUsdview/viewCubeMath.py`. Test them with
`bin/run_python_tests.sh test_viewcube_math` or the `.bat` equivalent.
`bin/run_testusdview_viewcube.sh` or `.bat` exercises the viewport integration.
Settings are session-local; roll buttons and projection menus are not provided.
