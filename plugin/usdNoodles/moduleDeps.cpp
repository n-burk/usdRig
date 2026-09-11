//
// Copyright 2024 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
////////////////////////////////////////////////////////////////////////

// OpenUSD's pxr_library() generates this file; this build does not use that
// macro, so it is checked in. It tells TfScriptModuleLoader which Python
// modules must be loaded before _usdNoodles wraps anything -- without them the
// Gf, Vt and Sdf values the bindings return have no Python converters. The
// module name is the top-level UsdNoodles, not pxr.UsdNoodles; see
// CMakeLists.txt for why.

#include "pxr/pxr.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/scriptModuleLoader.h"
#include "pxr/base/tf/token.h"

#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

TF_REGISTRY_FUNCTION(TfScriptModuleLoader) {
    // List of direct dependencies for this library.
    const std::vector<TfToken> reqs = {
        TfToken("garch"),
        TfToken("glf"),
        TfToken("hio"),
        TfToken("js"),
        TfToken("tf"),
        TfToken("gf"),
        TfToken("vt"),
        TfToken("sdf"),
        TfToken("usd")
    };
    TfScriptModuleLoader::GetInstance().
        RegisterLibrary(TfToken("usdNoodles"), TfToken("UsdNoodles"), reqs);
}

PXR_NAMESPACE_CLOSE_SCOPE
