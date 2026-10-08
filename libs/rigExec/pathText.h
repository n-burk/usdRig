// Path text without SdfPath::GetString().
#ifndef RIGEXEC_PATH_TEXT_H
#define RIGEXEC_PATH_TEXT_H

#include "pxr/usd/sdf/path.h"

#include <string>
#include <unordered_map>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace pathTextDetail {

// True for an absolute path to a prim below the root. An absolute prim-like
// path is the root, a prim or a variant selection, so these node-type reads
// decide it; IsPrimPath() may also compare against a function-local static.
inline bool
IsAbsolutePrim(const SdfPath &path)
{
    return path.IsAbsolutePath() && !path.ContainsPropertyElements() &&
           !path.IsAbsoluteRootPath() && !path.IsPrimVariantSelectionPath();
}

// A name token's text is a field of the token; GetString() on an empty token
// reads a function-local static.
inline void
AppendName(const TfToken &name, std::string *text)
{
    text->append(name.GetText(), name.size());
}

// Appends an absolute prim path below the root, root first. False at the
// first ancestor that is not a prim, such as a variant selection: its text
// is not its parent's plus a slash and a name.
inline bool
AppendPrimPath(const SdfPath &prim, std::string *text)
{
    if (prim.IsAbsoluteRootPath()) {
        return true;
    }
    if (!IsAbsolutePrim(prim) ||
        !AppendPrimPath(prim.GetParentPath(), text)) {
        return false;
    }
    *text += '/';
    AppendName(prim.GetNameToken(), text);
    return true;
}

}  // namespace pathTextDetail

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
/// many paths sit under it. Anything else -- a relative path, a path under a
/// variant selection, a target or mapper path, none of which a composed
/// stage hands these loops -- falls back to GetAsString(), which is exact
/// and simply slower.
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
        const bool property = path.IsPrimPropertyPath();
        // A property's parent keeps a variant selection above it, where
        // GetPrimPath() would step over it.
        const SdfPath parent = property || pathTextDetail::IsAbsolutePrim(path)
                                   ? path.GetParentPath()
                                   : SdfPath();
        if (path.IsAbsoluteRootPath()) {
            text = "/";
        } else if (pathTextDetail::IsAbsolutePrim(parent) ||
                   (!property && parent.IsAbsoluteRootPath())) {
            // Node-based, so the parent's text stays where it is while this
            // one is inserted beside it.
            const std::string &head = (*this)(parent);
            text.reserve(head.size() + 1 + path.GetNameToken().size());
            text = head;
            if (property) {
                text += '.';
            } else if (head.size() != 1) {
                text += '/';
            }
            pathTextDetail::AppendName(path.GetNameToken(), &text);
        } else if (!path.IsEmpty()) {
            text = path.GetAsString();
        }
        return _texts.emplace(path, std::move(text)).first->second;
    }

private:
    std::unordered_map<SdfPath, std::string, SdfPath::Hash>
        _texts;
};

/// Sets \p text to the root, an absolute prim path or an absolute prim
/// property path, spelled exactly as SdfPath::GetString() spells it, when
/// every prim above it is a plain prim. No memo, so a worker may call it:
/// the walk reads only node types and name tokens, and \p path keeps alive
/// every parent it visits. False, with \p text cleared, for any other path
/// -- relative, under a variant selection, or a target, mapper or
/// expression path -- whose text only GetAsString() could spell.
inline bool
RigExecSpellPathText(const SdfPath &path, std::string *text)
{
    text->clear();
    if (path.IsAbsoluteRootPath()) {
        *text = "/";
        return true;
    }
    // A property's parent keeps a variant selection above it, which
    // AppendPrimPath rejects; GetPrimPath() would step over it.
    const bool property = path.IsPrimPropertyPath();
    const SdfPath prim = property ? path.GetParentPath() : path;
    if (!pathTextDetail::IsAbsolutePrim(prim) ||
        !pathTextDetail::AppendPrimPath(prim, text)) {
        text->clear();
        return false;
    }
    if (property) {
        *text += '.';
        pathTextDetail::AppendName(path.GetNameToken(), text);
    }
    return true;
}

}  // namespace rigExec

#endif  // RIGEXEC_PATH_TEXT_H
