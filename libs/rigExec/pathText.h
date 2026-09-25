//
// Path text without SdfPath::GetString().
//
#ifndef RIGEXEC_PATH_TEXT_H
#define RIGEXEC_PATH_TEXT_H

#include "pxr/usd/sdf/path.h"

#include <string>
#include <unordered_map>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The text of a path, spelled exactly as SdfPath::GetString() spells it,
/// without asking SdfPath for it.
///
/// GetString() is not a field read. The first call for a path builds its
/// text and interns it as a TfToken in a process-wide table, and every later
/// call looks it up there again under a lock on the prim's bucket. A loop
/// that writes out thousands of paths -- the structure digest writes every
/// attribute of every pose-input provider, the baked schedule labels every
/// step -- pays that per path, and from several threads at once pays it in
/// contention too. MEASURED on puppetA: those calls were most of the ~8 ms
/// the Solvers digest spent building provider tokens outside its USD reads.
///
/// Here a prim path is its parent's text plus its name, and a prim property
/// path is its prim's text, a dot and its name: the name tokens are fields
/// of the path, so building the text touches no shared table. Every text is
/// kept for the life of the memo, so an ancestor is spelled once however
/// many paths sit under it. Anything else -- a relative path, a variant
/// selection, a target or mapper path, none of which a composed stage hands
/// these loops -- falls back to GetAsString(), which is exact and simply
/// slower.
///
/// Not thread-safe: it is a memo, so each thread that spells paths keeps its
/// own.
class RigExecPathText {
public:
    const std::string &operator()(const SdfPath &path)
    {
        const auto found = _texts.find(path);
        if (found != _texts.end()) {
            return found->second;
        }
        std::string text;
        if (path.IsAbsoluteRootPath()) {
            text = "/";
        } else if (path.IsAbsolutePath() && path.IsPrimPath()) {
            // Node-based, so the parent's text stays where it is while this
            // one is inserted beside it.
            const std::string &parent = (*this)(path.GetParentPath());
            text.reserve(parent.size() + 1 + path.GetNameToken().size());
            text = parent;
            if (parent.size() != 1) {
                text += '/';
            }
            text += path.GetNameToken().GetString();
        } else if (path.IsAbsolutePath() && path.IsPrimPropertyPath()) {
            const std::string &prim = (*this)(path.GetPrimPath());
            text.reserve(prim.size() + 1 + path.GetNameToken().size());
            text = prim;
            text += '.';
            text += path.GetNameToken().GetString();
        } else if (!path.IsEmpty()) {
            text = path.GetAsString();
        }
        return _texts.emplace(path, std::move(text)).first->second;
    }

private:
    std::unordered_map<SdfPath, std::string, SdfPath::Hash>
        _texts;
};

}  // namespace rigExec

#endif  // RIGEXEC_PATH_TEXT_H
