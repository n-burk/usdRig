//
// TouchPose C surface.
//
// The usdview plugin (plugin/touchPose) binds this with ctypes, the same way
// rigExecUsdview binds RigExecImaging_*: the stage crosses in-process as a
// UsdUtilsStageCache id, arrays cross as raw pointers into numpy buffers,
// and nothing needs a Python extension module built against one interpreter.
//
// One HANDLE per touched mesh. It owns:
//
//   * the posed geometry and its ray-cast acceleration (touchPoseMesh.h),
//     kept current from the RigExec snapshot the viewport draws, so a pick
//     lands on the deformed skin and not the rest mesh on the stage;
//   * the face -> region table the Python model edits when painting;
//   * the highlight state -- hover, lead, selection, paint-mode colours --
//     which it composes into the per-region colour table the Storm shader
//     reads (touchPoseHighlight.h). Setting a state that is already drawn
//     sends nothing to Hydra at all.
//
// Every call is main-thread (the Qt thread), which is the thread usdview
// changes its stage and draws on. Queries are internally parallel.
//
// Return conventions: counts and indices are >= 0 on success; -1 means a bad
// handle or bad arguments.
//
#ifndef RIGEXEC_IMAGING_TOUCH_POSE_H
#define RIGEXEC_IMAGING_TOUCH_POSE_H

#include <cstdint>

#if defined(_WIN32)
#  if defined(RIGEXEC_IMAGING_EXPORTS)
#    define RIGEXEC_TOUCHPOSE_C_API __declspec(dllexport)
#  else
#    define RIGEXEC_TOUCHPOSE_C_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define RIGEXEC_TOUCHPOSE_C_API __attribute__((visibility("default")))
#else
#  define RIGEXEC_TOUCHPOSE_C_API
#endif

extern "C" {

// -- lifetime ---------------------------------------------------------------

/// Reads \p meshPath's topology, rest points and transform off the stage in
/// \p stageCacheId. Returns a handle > 0, or 0 when there is no such mesh.
RIGEXEC_TOUCHPOSE_C_API long long RigExecTouchPose_Open(
    long long stageCacheId, const char *meshPath);

/// Builds a handle from raw topology with no stage -- for tests, and for a
/// host that owns its geometry. \p meshPath names the Hydra prim the
/// highlight attaches to (may be null for pick-only use).
RIGEXEC_TOUCHPOSE_C_API long long RigExecTouchPose_OpenTopology(
    const int32_t *faceVertexCounts, int faceCount,
    const int32_t *faceVertexIndices, int indexCount,
    int pointCount, const char *meshPath);

/// Releases the handle and takes its highlight down.
RIGEXEC_TOUCHPOSE_C_API void RigExecTouchPose_Close(long long handle);

RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_GetFaceCount(long long handle);
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_GetPointCount(long long handle);

// -- pose -------------------------------------------------------------------

/// Brings the geometry up to what the viewport draws: the RigExec snapshot's
/// points for this mesh when the rig publishes them, the stage's own points
/// at the time otherwise, and the mesh's local-to-world at the time.
/// Returns 1 when anything moved, 0 when nothing did, -1 on error. Cheap
/// when nothing moved (one atomic load and a pointer compare), so it can be
/// called per hover.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_SyncPose(
    long long handle, double frame, int isDefault, int force);

/// Explicit points (xyz triples, LOCAL space). Returns 0 on success.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_SetPoints(
    long long handle, const float *xyz, int pointCount);

/// Explicit local-to-world, row-major (USD's m[row][col]).
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_SetTransform(
    long long handle, const double *matrix16);

/// Copies the current LOCAL points out. Returns the point count.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_GetPoints(
    long long handle, float *xyzOut, int capacity);

/// World-space bounds of the posed mesh: min xyz, max xyz. 1 when non-empty.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_GetBounds(
    long long handle, double *minMaxOut);

// -- regions ----------------------------------------------------------------

/// face -> region (-1 for none), and how many regions there are. A paint
/// stroke calls this with the edited table; the highlight follows.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_SetFaceRegions(
    long long handle, const int32_t *regionOf, int faceCount,
    int regionCount);

// -- picking ----------------------------------------------------------------

/// Nearest face along a WORLD ray (either side), or -1. \p tOut (nullable)
/// receives the distance along the normalized direction.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_Cast(
    long long handle, const double *origin3, const double *direction3,
    double *tOut);

/// The same test by brute force over every triangle (tests only).
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_CastBruteForce(
    long long handle, const double *origin3, const double *direction3,
    double *tOut);

/// Regions with a front-facing face centroid inside a pixel rectangle.
/// \p viewProjection16 is row-major world-to-clip (row-vector convention).
/// Writes up to \p capacity region indices; returns how many there are.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_RegionsInRect(
    long long handle, const double *viewProjection16,
    double width, double height, const double *eye3,
    double x0, double y0, double x1, double y1,
    int32_t *regionsOut, int capacity);

/// Faces whose centroid is within \p radius of \p center3, facing against
/// \p direction3 when it is non-null and non-zero. Returns the count;
/// writes up to \p capacity.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_Brush(
    long long handle, const double *center3, const double *direction3,
    double radius, int32_t *facesOut, int capacity);

/// One face's world centroid. Returns 1 on success.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_FaceCentroid(
    long long handle, int face, double *out3);

// -- highlight --------------------------------------------------------------

/// Per-region colours, rgb triples: \p hoverRgb is what a hovered region is
/// lit with in control mode, \p editRgb what every region is lit with in
/// paint mode (and what the hovered one is lit with there).
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_SetRegionColors(
    long long handle, const float *hoverRgb, const float *editRgb,
    int regionCount);

/// Attaches (1) or detaches (0) the Storm highlight for this handle's mesh.
/// Attaching is the one step that makes Hydra recompile a shader; every
/// state change after it is a colour-table upload.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_SetHighlightEnabled(
    long long handle, int enabled);

/// The whole highlight state in one call. Composited low to high: paint
/// mode's every-region colours, then selected, then lead, then hover -- the
/// region under the cursor always shows what a click would do. Returns 1
/// when the drawn table changed, 0 when it was already this, -1 on error.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_SetHighlightState(
    long long handle, int hoverRegion, int leadRegion,
    const int32_t *selectedRegions, int selectedCount,
    int editing, float opacity,
    const float *leadRgb3, const float *selectedRgb3);

/// The composed table: (regionCount + 1) rgba rows, row 0 unlit. Returns the
/// row count.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_GetHighlightTable(
    long long handle, float *rgbaOut, int capacityRows);

/// Diagnostics: live TouchPose scene indices (one per imaging chain), and
/// how many colour-table uploads have been sent to Hydra.
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_GetSceneIndexCount();
RIGEXEC_TOUCHPOSE_C_API long long RigExecTouchPose_GetTableUpdateCount();

/// The generated glslfx wrapping UsdPreviewSurface, for inspection. Returns
/// the full length; writes up to \p capacity bytes (NUL-terminated if room).
RIGEXEC_TOUCHPOSE_C_API int RigExecTouchPose_GetShaderSource(
    char *out, int capacity);

}

#endif  // RIGEXEC_IMAGING_TOUCH_POSE_H
