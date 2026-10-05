// .rigexec path table: the File.names and File.paths a bake writes. Every
// prim and property path is a tree node (its parent interned first, so a
// node's parent id is below its own), every token and every other kind of
// path one Token node holding the whole text. Ids follow insertion order,
// and the bake visits the program in a fixed order, so two bakes of one
// stage intern identically. Internal to rigExecBake.
#ifndef RIGEXEC_BAKE_PATH_TABLE_H
#define RIGEXEC_BAKE_PATH_TABLE_H

#include "rigExecBinary/format.h"

#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace rigExec {

/// The path tree of a .rigexec file. names[0] is "" and paths[0] is
/// {0, 0, None}; the empty path and the empty token are id 0.
class RigExecBakePathTable {
public:
    RigExecBakePathTable();

    /// A prim or property path as a tree node; any other path (the
    /// absolute root, a target, relational or variant-selection path) as
    /// one Token node holding its text.
    uint32_t Path(const PXR_NS::SdfPath &path);
    /// The Token node of \p token; 0 for the empty token.
    uint32_t Token(const PXR_NS::TfToken &token);
    uint32_t Token(const std::string &text);

    /// The text of \p id, as RigExecFormatPathText composes it; "" for 0
    /// or an id out of range.
    std::string Text(uint32_t id) const;

    size_t Size() const { return _paths.size(); }

    /// Hands names and paths to \p file; the table is empty afterwards.
    void MoveInto(fb::RigExecWireFile *file);

private:
    uint32_t _Name(const std::string &text);
    uint32_t _Node(uint32_t parent, uint32_t name, fb::PathKind kind);

    std::vector<std::string> _names;
    std::vector<fb::PathNode> _paths;
    // Lookups only: ids come from the vectors' sizes, never from iterating
    // these.
    std::map<std::string, uint32_t> _nameIds;
    std::map<std::tuple<uint32_t, uint32_t, uint8_t>, uint32_t> _nodeIds;
    std::unordered_map<PXR_NS::SdfPath, uint32_t, PXR_NS::SdfPath::Hash>
        _pathIds;
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_PATH_TABLE_H
