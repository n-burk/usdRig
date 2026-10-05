// .rigexec path table.
#include "rigExecBake/pathTable.h"

#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

RigExecBakePathTable::RigExecBakePathTable()
    : _names{std::string()},
      _paths{fb::PathNode(0, 0, fb::PathKind::None)}
{
}

uint32_t
RigExecBakePathTable::_Name(const std::string &text)
{
    const auto found = _nameIds.emplace(text, uint32_t(_names.size()));
    if (found.second) {
        _names.push_back(text);
    }
    return found.first->second;
}

uint32_t
RigExecBakePathTable::_Node(uint32_t parent, uint32_t name,
                            fb::PathKind kind)
{
    const auto found = _nodeIds.emplace(
        std::make_tuple(parent, name, uint8_t(kind)),
        uint32_t(_paths.size()));
    if (found.second) {
        _paths.emplace_back(parent, name, kind);
    }
    return found.first->second;
}

uint32_t
RigExecBakePathTable::Path(const SdfPath &path)
{
    if (path.IsEmpty()) {
        return 0;
    }
    const auto found = _pathIds.find(path);
    if (found != _pathIds.end()) {
        return found->second;
    }
    // A tree node needs an absolute prim or prim-property path whose name
    // is one component; its parent is interned first, from the name token
    // rather than the composed text.
    uint32_t id = 0;
    if (path.IsAbsolutePath() && !path.ContainsPrimVariantSelection() &&
        (path.IsPrimPath() || path.IsPrimPropertyPath())) {
        const std::string &name = path.GetNameToken().GetString();
        if (!name.empty() && name.find_first_of("/.") == std::string::npos) {
            if (path.IsPrimPath()) {
                const SdfPath parent = path.GetParentPath();
                const uint32_t parentId =
                    parent.IsAbsoluteRootPath() ? 0 : Path(parent);
                if (parentId == 0 ||
                    _paths[parentId].kind() == fb::PathKind::Prim) {
                    id = _Node(parentId, _Name(name), fb::PathKind::Prim);
                }
            } else {
                const uint32_t parentId = Path(path.GetPrimPath());
                if (parentId != 0 &&
                    _paths[parentId].kind() == fb::PathKind::Prim) {
                    id = _Node(parentId, _Name(name), fb::PathKind::Property);
                }
            }
        }
    }
    if (id == 0) {
        id = Token(path.GetString());
    }
    _pathIds.emplace(path, id);
    return id;
}

uint32_t
RigExecBakePathTable::Token(const TfToken &token)
{
    return Token(token.GetString());
}

uint32_t
RigExecBakePathTable::Token(const std::string &text)
{
    if (text.empty()) {
        return 0;
    }
    return _Node(0, _Name(text), fb::PathKind::Token);
}

std::string
RigExecBakePathTable::Text(uint32_t id) const
{
    std::vector<uint32_t> chain;
    uint32_t at = id;
    while (at != 0 && at < _paths.size()) {
        chain.push_back(at);
        if (_paths[at].kind() == fb::PathKind::Token) {
            break;
        }
        at = _paths[at].parent();
    }
    std::string text;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const fb::PathNode &node = _paths[*it];
        switch (node.kind()) {
        case fb::PathKind::Prim:
            text += '/';
            break;
        case fb::PathKind::Property:
            text += '.';
            break;
        default:
            break;
        }
        text += _names[node.name()];
    }
    return text;
}

void
RigExecBakePathTable::MoveInto(fb::RigExecWireFile *file)
{
    file->names = std::move(_names);
    file->paths = std::move(_paths);
    _names.assign(1, std::string());
    _paths.assign(1, fb::PathNode(0, 0, fb::PathKind::None));
    _nameIds.clear();
    _nodeIds.clear();
    _pathIds.clear();
}

}  // namespace rigExec
