//
// RigExec RBF parity: libs/rigExecMath/rbf.{h,cpp} against the studio's
// an independent reference implementation, which is the ORACLE.
//
// The Python is not imported. tools/biped/gen_psd_parity.py ran it once, by
// hand, over every interpolator in the biped's the conventional tool PSD export plus a set of
// hand-built solvers, and froze the answers into
// tests/fixtures/psd_parity.json. This file reads that fixture and asserts
// the C++ agrees to 1e-6 in double precision.
//
// WHY A FIXTURE AND NOT A UNIT TEST OF EACH PIECE. The failure mode for this
// port is a sign or an ordering convention that is wrong in a way only a
// specific pose exposes -- the transposed inverse in Solve() and the
// swing/twist split are both exactly that kind of detail, and both are
// invisible on a symmetric three-pose fan with one shared radius. 67
// interpolators at 200 driver poses each is what catches them. Every case in
// the fixture carries a `why`, printed on failure, so a red line says what
// was being pinned rather than only which index disagreed.
//
// DO NOT DELETE THE HAND-BUILT CASES AS REDUNDANT WITH THE REAL DATA. They
// are not redundant, and the measurement is in the docstring on SAMPLES in
// tools/biped/gen_psd_parity.py. Summarised: breaking the per-pose radii so
// they are measured in the other direction is caught by ZERO of the 67 biped
// interpolators, at any sample count. Every one of them uses a single
// poseType throughout, which makes its distance metric symmetric and hides a
// direction error completely; only `pose_type_mixed` mixes them.
//
// The same sweep shows the reverse too -- an error that lives in the
// per-frame path and leaves every solve constant intact is caught ONLY by
// the sampled real data, and degrades with sample count: moving the
// normalisation refusal threshold falls from 82 failures and 10 distinct
// catchers at 200 samples to four failures and 2 at 25, with none of the
// biped's interpolators among them.
//
// So the two halves of this fixture cover different failure classes and
// neither substitutes for the other. Cutting either one leaves this file
// passing while it stops testing.
//
// The fixture is parsed with a small hand-written JSON reader rather than
// through a library: rigExecMath links arch/tf/gf/vt and nothing else, and a
// dependency added for a test is a dependency the shipped library carries.
//
#include "rigExecMath/rbf.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

// ---------------------------------------------------------------------------
// A minimal JSON value
// ---------------------------------------------------------------------------

namespace {

struct Json;
using JsonPtr = std::shared_ptr<Json>;

struct Json {
    // Prefixed: an unscoped enumerator named `Number` would collide with
    // the accessor of the same name below, and MSVC reports that as a
    // term that does not evaluate to a function.
    enum Kind { KindNull, KindBool, KindNumber, KindString, KindArray,
                KindObject } kind = KindNull;
    bool boolean = false;
    double number = 0.0;
    std::string text;
    std::vector<JsonPtr> items;
    std::map<std::string, JsonPtr> fields;

    bool Has(const std::string &key) const {
        return fields.find(key) != fields.end();
    }
    const JsonPtr &At(const std::string &key) const {
        static const JsonPtr empty;
        auto it = fields.find(key);
        return it == fields.end() ? empty : it->second;
    }
    double Number(const std::string &key, double fallback = 0.0) const {
        const JsonPtr &value = At(key);
        return value && value->kind == KindNumber ? value->number : fallback;
    }
    bool Bool(const std::string &key, bool fallback = false) const {
        const JsonPtr &value = At(key);
        return value && value->kind == KindBool ? value->boolean : fallback;
    }
    std::string Text(const std::string &key) const {
        const JsonPtr &value = At(key);
        return value && value->kind == KindString ? value->text
                                                  : std::string();
    }
};

class JsonParser {
public:
    explicit JsonParser(const std::string &source)
        : _source(source), _at(0) {}

    JsonPtr Parse() {
        JsonPtr value = _Value();
        _Space();
        return value;
    }
    bool Ok() const { return _error.empty(); }
    const std::string &Error() const { return _error; }

private:
    void _Space() {
        while (_at < _source.size() &&
               (_source[_at] == ' ' || _source[_at] == '\t' ||
                _source[_at] == '\n' || _source[_at] == '\r')) {
            ++_at;
        }
    }
    void _Fail(const char *what) {
        if (_error.empty()) {
            std::ostringstream out;
            out << what << " at byte " << _at;
            _error = out.str();
        }
    }
    JsonPtr _Value() {
        _Space();
        if (_at >= _source.size()) {
            _Fail("unexpected end");
            return nullptr;
        }
        const char c = _source[_at];
        if (c == '{') return _Object();
        if (c == '[') return _Array();
        if (c == '"') {
            JsonPtr node = std::make_shared<Json>();
            node->kind = Json::KindString;
            node->text = _String();
            return node;
        }
        if (!_source.compare(_at, 4, "true") ||
            !_source.compare(_at, 5, "false")) {
            JsonPtr node = std::make_shared<Json>();
            node->kind = Json::KindBool;
            node->boolean = _source[_at] == 't';
            _at += node->boolean ? 4 : 5;
            return node;
        }
        if (!_source.compare(_at, 4, "null")) {
            _at += 4;
            return std::make_shared<Json>();
        }
        JsonPtr node = std::make_shared<Json>();
        node->kind = Json::KindNumber;
        // strtod is what json.dumps's repr round-trips through; anything
        // less exact would make the fixture useless as a 1e-6 oracle.
        const char *begin = _source.c_str() + _at;
        char *end = nullptr;
        node->number = std::strtod(begin, &end);
        if (end == begin) {
            _Fail("bad number");
            return nullptr;
        }
        _at += static_cast<size_t>(end - begin);
        return node;
    }
    std::string _String() {
        ++_at;  // the opening quote
        std::string out;
        while (_at < _source.size() && _source[_at] != '"') {
            if (_source[_at] == '\\' && _at + 1 < _source.size()) {
                ++_at;
                const char c = _source[_at++];
                switch (c) {
                case 'n': out.push_back('\n'); break;
                case 't': out.push_back('\t'); break;
                case 'r': out.push_back('\r'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'u': _at += 4; out.push_back('?'); break;
                default: out.push_back(c); break;
                }
                continue;
            }
            out.push_back(_source[_at++]);
        }
        ++_at;  // the closing quote
        return out;
    }
    JsonPtr _Array() {
        JsonPtr node = std::make_shared<Json>();
        node->kind = Json::KindArray;
        ++_at;
        _Space();
        if (_at < _source.size() && _source[_at] == ']') {
            ++_at;
            return node;
        }
        while (_at < _source.size()) {
            JsonPtr item = _Value();
            if (!Ok()) return node;
            node->items.push_back(item);
            _Space();
            if (_at < _source.size() && _source[_at] == ',') {
                ++_at;
                continue;
            }
            if (_at < _source.size() && _source[_at] == ']') {
                ++_at;
                return node;
            }
            _Fail("expected , or ]");
            return node;
        }
        _Fail("unterminated array");
        return node;
    }
    JsonPtr _Object() {
        JsonPtr node = std::make_shared<Json>();
        node->kind = Json::KindObject;
        ++_at;
        _Space();
        if (_at < _source.size() && _source[_at] == '}') {
            ++_at;
            return node;
        }
        while (_at < _source.size()) {
            _Space();
            if (_at >= _source.size() || _source[_at] != '"') {
                _Fail("expected a key");
                return node;
            }
            const std::string key = _String();
            _Space();
            if (_at >= _source.size() || _source[_at] != ':') {
                _Fail("expected :");
                return node;
            }
            ++_at;
            JsonPtr value = _Value();
            if (!Ok()) return node;
            node->fields[key] = value;
            _Space();
            if (_at < _source.size() && _source[_at] == ',') {
                ++_at;
                continue;
            }
            if (_at < _source.size() && _source[_at] == '}') {
                ++_at;
                return node;
            }
            _Fail("expected , or }");
            return node;
        }
        _Fail("unterminated object");
        return node;
    }

    const std::string &_source;
    size_t _at;
    std::string _error;
};

// -- fixture readers --------------------------------------------------------

std::vector<double>
Doubles(const JsonPtr &node)
{
    std::vector<double> out;
    if (!node || node->kind != Json::KindArray) {
        return out;
    }
    out.reserve(node->items.size());
    for (const JsonPtr &item : node->items) {
        out.push_back(item && item->kind == Json::KindNumber ? item->number
                                                         : 0.0);
    }
    return out;
}

std::vector<GfVec3d>
Vectors(const JsonPtr &node)
{
    std::vector<GfVec3d> out;
    if (!node || node->kind != Json::KindArray) {
        return out;
    }
    out.reserve(node->items.size());
    for (const JsonPtr &item : node->items) {
        const std::vector<double> values = Doubles(item);
        out.push_back(GfVec3d(values.size() > 0 ? values[0] : 0.0,
                              values.size() > 1 ? values[1] : 0.0,
                              values.size() > 2 ? values[2] : 0.0));
    }
    return out;
}

std::vector<std::vector<double>>
Rows(const JsonPtr &node)
{
    std::vector<std::vector<double>> out;
    if (!node || node->kind != Json::KindArray) {
        return out;
    }
    out.reserve(node->items.size());
    for (const JsonPtr &item : node->items) {
        out.push_back(Doubles(item));
    }
    return out;
}

std::vector<bool>
Bools(const JsonPtr &node)
{
    std::vector<bool> out;
    if (!node || node->kind != Json::KindArray) {
        return out;
    }
    for (const JsonPtr &item : node->items) {
        out.push_back(item && item->kind == Json::KindBool ? item->boolean
                                                       : false);
    }
    return out;
}

RigExecRbfKernel
KernelFrom(const std::string &name)
{
    return name == "linear" ? RigExecRbfKernel::Linear
                            : RigExecRbfKernel::Gaussian;
}

std::vector<RigExecRbfPoseType>
PoseTypes(const JsonPtr &node)
{
    std::vector<RigExecRbfPoseType> out;
    for (double value : Doubles(node)) {
        const int kind = static_cast<int>(value);
        out.push_back(kind == 1   ? RigExecRbfPoseType::Swing
                      : kind == 2 ? RigExecRbfPoseType::Twist
                                  : RigExecRbfPoseType::Whole);
    }
    return out;
}

// -- comparison -------------------------------------------------------------

const double kTolerance = 1.0e-6;

/// Absolute agreement, and the ONLY comparison used on anything the fixture
/// calls a weight. The task's instruction stands: if a case cannot be made to
/// match, report the inputs, do not widen this.
bool
Agrees(double found, double expected, double tolerance = kTolerance)
{
    if (std::isnan(found) || std::isnan(expected)) {
        return std::isnan(found) && std::isnan(expected);
    }
    return std::abs(found - expected) <= tolerance;
}

struct Worst {
    double value = 0.0;
    std::string where;
    void Note(double delta, const std::string &what) {
        if (delta > value) {
            value = delta;
            where = what;
        }
    }
};

bool
CompareRow(const std::vector<double> &found,
           const std::vector<double> &expected, const std::string &what,
           Worst *worst, int *reported)
{
    if (found.size() != expected.size()) {
        ++failures;
        std::printf("FAIL %s: %zu values, fixture has %zu\n", what.c_str(),
                    found.size(), expected.size());
        return false;
    }
    bool ok = true;
    for (size_t i = 0; i < found.size(); ++i) {
        const double delta = std::abs(found[i] - expected[i]);
        worst->Note(delta, what);
        if (!Agrees(found[i], expected[i])) {
            ok = false;
            // Cap the noise: a systematic sign error disagrees everywhere,
            // and ten thousand identical lines hide the one that is
            // different.
            if (*reported < 12) {
                ++(*reported);
                std::printf("FAIL %s [%zu]: got %.17g, oracle %.17g "
                            "(delta %.3g)\n",
                            what.c_str(), i, found[i], expected[i], delta);
            }
        }
    }
    if (!ok) {
        ++failures;
    }
    return ok;
}

// ---------------------------------------------------------------------------
// The suites
// ---------------------------------------------------------------------------

/// The pieces that can be asserted without the fixture, so a build that is
/// broken outright says so in one line instead of in 13400.
void
TestKernelsAndMetric()
{
    // the reference implementation.
    CHECK(Agrees(RigExecRbfGaussian(0.0, 1.0), 1.0, 0.0));
    CHECK(Agrees(RigExecRbfGaussian(1.0, 1.0), std::exp(-1.0), 0.0));
    CHECK(Agrees(RigExecRbfGaussian(2.0, 4.0), std::exp(-0.25), 0.0));
    CHECK(Agrees(RigExecRbfLinear(0.0, 2.0), 1.0, 0.0));
    CHECK(Agrees(RigExecRbfLinear(1.0, 2.0), 0.5, 0.0));
    CHECK(Agrees(RigExecRbfLinear(3.0, 2.0), 0.0, 0.0));
    // A radius of zero is a pose with no width: 1 exactly on it, 0 off it.
    CHECK(Agrees(RigExecRbfGaussian(0.0, 0.0), 1.0, 0.0));
    CHECK(Agrees(RigExecRbfGaussian(0.1, 0.0), 0.0, 0.0));
    CHECK(Agrees(RigExecRbfLinear(0.1, 0.0), 0.0, 0.0));

    // The rotation-only branch is the BARE ratio, bit for bit -- not
    // sqrt(x*x), which would be right to within an ulp and wrong as a
    // promise.
    for (double angle : {0.1, 0.7, 1.3, 2.9}) {
        const double width = 0.37;
        CHECK(RigExecRbfCombine(angle, width, 0.0, 0.0, true, false) ==
              angle / width);
    }
    // Right-angled triangle when both channels are on.
    {
        const double d = RigExecRbfCombine(0.6, 0.3, 0.008, 0.004, true,
                                           true);
        CHECK(Agrees(d, std::sqrt(2.0 * 2.0 + 2.0 * 2.0), 1e-15));
    }
    // No width and not sitting on the pose is infinite, and both kernels
    // take that to zero without a branch.
    {
        const double d =
            RigExecRbfCombine(0.5, 0.0, 0.0, 0.001, true, true);
        CHECK(std::isinf(d));
        CHECK(RigExecRbfGaussian(d, 1.0) == 0.0);
        CHECK(RigExecRbfLinear(d, 1.0) == 0.0);
    }
    // Both channels off measures nothing at all.
    CHECK(RigExecRbfCombine(1.0, 0.3, 1.0, 0.3, false, false) == 0.0);

    // Euler <-> quaternion round trip, and the unit-norm the whole metric
    // assumes.
    for (const GfVec3d &euler : {GfVec3d(0.3, -0.7, 1.1),
                                 GfVec3d(0.0, 0.0, 0.0),
                                 GfVec3d(-2.5, 0.4, 0.9)}) {
        const GfQuatd q = RigExecRbfQuaternionFromEuler(euler);
        CHECK(Agrees(q.GetLength(), 1.0, 1e-14));
        const GfVec3d back = RigExecRbfEulerFromQuaternion(q);
        CHECK((back - euler).GetLength() < 1e-12);
        CHECK(Agrees(RigExecRbfAngleBetween(q, q), 0.0, 1e-7));
    }

    // Swing/twist: a pure twist about the axis has identity swing, and a
    // pure swing perpendicular to it has identity twist.
    {
        const GfVec3d axis(0.0, 1.0, 0.0);
        GfQuatd swing, twist;
        RigExecRbfSwingTwist(RigExecRbfQuaternionFromEuler(
                                 GfVec3d(0.0, 1.1, 0.0)),
                             axis, &swing, &twist);
        CHECK(Agrees(RigExecRbfAngleBetween(swing, GfQuatd(1.0)), 0.0,
                     1e-7));
        CHECK(Agrees(RigExecRbfAngleBetween(twist, GfQuatd(1.0)), 1.1,
                     1e-12));
        RigExecRbfSwingTwist(RigExecRbfQuaternionFromEuler(
                                 GfVec3d(0.8, 0.0, 0.0)),
                             axis, &swing, &twist);
        CHECK(Agrees(RigExecRbfAngleBetween(twist, GfQuatd(1.0)), 0.0,
                     1e-7));
        CHECK(Agrees(RigExecRbfAngleBetween(swing, GfQuatd(1.0)), 0.8,
                     1e-12));
    }

    // The angle takes the SHORT way round the double cover. Without the
    // |dot| a pose more than a half turn away measures as though it were
    // close, which is the whole point of the reference implementation.
    {
        const GfQuatd a = RigExecRbfQuaternionFromEuler(GfVec3d(0.0, 0.0,
                                                                0.0));
        const GfQuatd b = RigExecRbfQuaternionFromEuler(
            GfVec3d(0.0, 0.0, 3.0));
        const double direct = RigExecRbfAngleBetween(a, b);
        const GfQuatd negated =
            GfQuatd(-b.GetReal(), -b.GetImaginary());
        CHECK(Agrees(RigExecRbfAngleBetween(a, negated), direct, 1e-12));
        CHECK(direct <= 3.141592653589794);
    }

    // Gauss-Jordan against a known inverse, and a singular matrix refused
    // rather than returned as garbage.
    {
        std::vector<std::vector<double>> inverse;
        CHECK(RigExecRbfInvert({{4.0, 7.0}, {2.0, 6.0}}, &inverse));
        CHECK(Agrees(inverse[0][0], 0.6, 1e-15));
        CHECK(Agrees(inverse[0][1], -0.7, 1e-15));
        CHECK(Agrees(inverse[1][0], -0.2, 1e-15));
        CHECK(Agrees(inverse[1][1], 0.4, 1e-15));
        CHECK(!RigExecRbfInvert({{1.0, 2.0}, {2.0, 4.0}}, &inverse));
    }
}

/// One solver, built from a fixture record, against its recorded answers.
///
/// Shared by the hand-built cases and the 67 biped interpolators, because
/// the two differ only in where the widths come from: a case states them,
/// an interpolator has them fitted.
void
CheckSamples(const Json &record, const RigExecRbfSolver &solver,
             const std::string &label, Worst *worst, int *refusedSeen,
             int *negativeSeen)
{
    const JsonPtr &samples = record.At("samples");
    if (!samples) {
        ++failures;
        std::printf("FAIL %s: no samples in the fixture\n", label.c_str());
        return;
    }
    const std::vector<GfVec3d> rotations = Vectors(samples->At("rotations"));
    const std::vector<GfVec3d> translations =
        Vectors(samples->At("translations"));
    const std::vector<std::vector<double>> expected =
        Rows(samples->At("weights"));
    const std::vector<std::vector<double>> clamped =
        samples->Has("clamped") ? Rows(samples->At("clamped"))
                                : std::vector<std::vector<double>>();
    const std::vector<bool> refused = Bools(samples->At("refused"));

    CHECK(rotations.size() == expected.size());
    CHECK(translations.size() == expected.size());

    int reported = 0;
    std::vector<double> found;
    for (size_t i = 0; i < rotations.size() && i < expected.size(); ++i) {
        const GfVec3d *translation =
            solver.GetTranslations().empty() ? nullptr : &translations[i];

        // Always evaluate the UNCLAMPED weights first: the fixture's
        // `weights` column is the solver's own answer, and the clamp is a
        // separate step applied on top of it.
        solver.Evaluate(rotations[i], translation, &found, true);
        std::ostringstream what;
        what << label << " sample " << i;
        CompareRow(found, expected[i], what.str(), worst, &reported);

        for (double value : found) {
            if (value < -1.0e-9) {
                ++(*negativeSeen);
            }
        }

        // The refusal branch: |sum| < 1e-6 leaves the weights alone. The
        // fixture records where the Python took it, so this asserts the
        // BRANCH agrees and not merely the arithmetic.
        if (i < refused.size() && solver.GetNormalize()) {
            double total = 0.0;
            for (double value : found) {
                total += value;
            }
            const bool here = std::abs(total) < RigExecRbfNormalizeFloor;
            // A refused sample keeps its raw sum; an accepted one sums to
            // exactly 1 within rounding. Either way the two must agree
            // about which happened.
            if (here != refused[i]) {
                ++failures;
                std::printf("FAIL %s sample %zu: normalisation %s here, "
                            "%s in the oracle (sum %.17g)\n",
                            label.c_str(), i, here ? "refused" : "applied",
                            refused[i] ? "refused" : "applied", total);
            }
            if (refused[i]) {
                ++(*refusedSeen);
            }
        }

        if (!clamped.empty() && i < clamped.size()) {
            solver.Evaluate(rotations[i], translation, &found, false);
            std::ostringstream clampWhat;
            clampWhat << label << " clamped sample " << i;
            CompareRow(found, clamped[i], clampWhat.str(), worst,
                       &reported);
        }
    }
}

void
TestCases(const Json &fixture, Worst *worst, int *refusedSeen,
          int *negativeSeen)
{
    const JsonPtr &cases = fixture.At("cases");
    if (!cases || cases->kind != Json::KindArray) {
        ++failures;
        std::printf("FAIL: no `cases` in the fixture\n");
        return;
    }
    // Each pins a branch the biped's own data does not happen to take --
    // or, for pose_type_mixed, one it takes but cannot expose. See the
    // header.
    std::printf("-- %zu hand-built cases\n", cases->items.size());
    for (const JsonPtr &item : cases->items) {
        const Json &record = *item;
        const std::string name = record.Text("name");

        RigExecRbfSolverDesc desc;
        desc.poses = Vectors(record.At("poses"));
        desc.translations = Vectors(record.At("translations"));
        desc.kernel = KernelFrom(record.Text("kernel"));
        desc.radius = record.Number("radius_in");
        desc.translationRadius = record.Number("translation_radius_in");
        desc.falloffs = Doubles(record.At("falloffs"));
        desc.poseTypes = PoseTypes(record.At("pose_types"));
        const std::vector<double> axis = Doubles(record.At("twist_axis"));
        if (axis.size() == 3) {
            desc.twistAxis = GfVec3d(axis[0], axis[1], axis[2]);
        }
        desc.regularization = record.Number("regularization");
        desc.normalize = record.Bool("normalize", true);
        desc.enableRotation = record.Bool("enable_rotation_in", true);
        desc.enableTranslation = record.Bool("enable_translation_in", false);

        RigExecRbfSolver solver(desc);
        solver.Solve();

        // The constants the Python settled on, before any weight is
        // evaluated: a width measured a hair differently makes every
        // downstream number wrong, and saying so here is a far more useful
        // failure than 60 disagreeing samples.
        int reported = 0;
        const std::string label = name.empty() ? "case" : name;
        if (!Agrees(solver.GetRadius(), record.Number("radius"), 1e-12)) {
            ++failures;
            std::printf("FAIL %s radius: got %.17g, oracle %.17g\n"
                        "     (%s)\n",
                        label.c_str(), solver.GetRadius(),
                        record.Number("radius"), record.Text("why").c_str());
        }
        CHECK(Agrees(solver.GetTranslationRadius(),
                     record.Number("translation_radius"), 1e-12));
        CompareRow(solver.GetRadii(), Doubles(record.At("radii")),
                   label + " radii", worst, &reported);
        CompareRow(solver.GetTranslationRadii(),
                   Doubles(record.At("translation_radii")),
                   label + " translation radii", worst, &reported);
        CHECK(solver.GetEnableRotation() ==
              record.Bool("enable_rotation", true));
        CHECK(solver.GetEnableTranslation() ==
              record.Bool("enable_translation", false));
        CHECK(solver.Degenerate() == record.Bool("degenerate", false));

        if (record.Bool("check_weights", true)) {
            const std::vector<std::vector<double>> expected =
                Rows(record.At("weights"));
            CHECK(solver.GetWeights().size() == expected.size());
            for (size_t r = 0;
                 r < solver.GetWeights().size() && r < expected.size(); ++r) {
                std::ostringstream what;
                what << label << " weights row " << r;
                CompareRow(solver.GetWeights()[r], expected[r], what.str(),
                           worst, &reported);
            }
        } else {
            // The singular fallback. the reference implementation nudges the diagonal by
            // 1e-12 and inverts again, so the inverse is around 1e12 and no
            // absolute tolerance means anything on it -- what is asserted
            // is that the branch was TAKEN, that the result is finite, and
            // that the evaluated weights (below) still match.
            CHECK(solver.GetRegularizedSingular());
            for (const std::vector<double> &row : solver.GetWeights()) {
                for (double value : row) {
                    CHECK(std::isfinite(value));
                }
            }
        }

        if (failures && reported) {
            std::printf("     ^ %s\n", record.Text("why").c_str());
        }
        // The fixture carries a `clamped` column only where the conventional tool's
        // allowNegativeWeights is off, so CheckSamples reads the flag off the
        // column's presence rather than being told twice.
        CheckSamples(record, solver, label, worst, refusedSeen, negativeSeen);
    }
}

void
TestInterpolators(const Json &fixture, Worst *worst, int *refusedSeen,
                  int *negativeSeen, int *perPose, int *translationDriven)
{
    const JsonPtr &all = fixture.At("interpolators");
    if (!all || all->kind != Json::KindArray) {
        ++failures;
        std::printf("FAIL: no `interpolators` in the fixture\n");
        return;
    }
    std::printf("-- %zu biped interpolators\n", all->items.size());
    size_t poses = 0;
    for (const JsonPtr &item : all->items) {
        const Json &record = *item;
        const std::string name = record.Text("name");

        RigExecRbfSolverDesc desc;
        desc.poses = Vectors(record.At("poses"));
        desc.translations = Vectors(record.At("translations"));
        desc.kernel = KernelFrom(record.Text("kernel"));
        desc.poseTypes = PoseTypes(record.At("pose_types"));
        const std::vector<double> axis = Doubles(record.At("twist_axis"));
        if (axis.size() == 3) {
            desc.twistAxis = GfVec3d(axis[0], axis[1], axis[2]);
        }
        desc.regularization = record.Number("regularization");
        desc.enableRotation = record.Bool("enable_rotation", true);
        desc.enableTranslation = record.Bool("enable_translation", false);
        poses += desc.poses.size();

        // The fitter, not a stated width: the conventional tool does not export the width it
        // solves with, so this is the number the whole conversion hangs off
        // (see RigExecRbfFitWidth).
        RigExecRbfFitReport report;
        RigExecRbfSolver solver = RigExecRbfFitWidth(desc, &report);

        const JsonPtr &fit = record.At("fit");
        int reported = 0;
        if (fit) {
            if (!Agrees(report.width, fit->Number("width"), 1e-12) ||
                report.perPose != fit->Bool("per_pose") ||
                !Agrees(report.translationWidth,
                        fit->Number("translation_width"), 1e-12)) {
                ++failures;
                std::printf(
                    "FAIL %s fit: width %.17g/%.17g, translation "
                    "%.17g/%.17g, per-pose %d/%d\n",
                    name.c_str(), report.width, fit->Number("width"),
                    report.translationWidth,
                    fit->Number("translation_width"),
                    static_cast<int>(report.perPose),
                    static_cast<int>(fit->Bool("per_pose")));
            }
            CHECK(Agrees(report.coverage, fit->Number("coverage"), 1e-9));
            CHECK(Agrees(report.overshoot, fit->Number("overshoot"), 1e-9));
        }
        if (report.perPose) {
            ++(*perPose);
        }
        if (solver.GetEnableTranslation()) {
            ++(*translationDriven);
        }

        CompareRow(solver.GetRadii(), Doubles(record.At("radii")),
                   name + " radii", worst, &reported);
        CompareRow(solver.GetTranslationRadii(),
                   Doubles(record.At("translation_radii")),
                   name + " translation radii", worst, &reported);
        CHECK(Agrees(solver.GetRadius(), record.Number("radius"), 1e-12));
        CHECK(Agrees(solver.GetTranslationRadius(),
                     record.Number("translation_radius"), 1e-12));
        CHECK(solver.GetEnableRotation() ==
              record.Bool("enable_rotation", true));
        CHECK(solver.GetEnableTranslation() ==
              record.Bool("enable_translation", false));
        CHECK(solver.Degenerate() == record.Bool("degenerate", false));

        const std::vector<std::vector<double>> expected =
            Rows(record.At("weights"));
        CHECK(solver.GetWeights().size() == expected.size());
        for (size_t r = 0;
             r < solver.GetWeights().size() && r < expected.size(); ++r) {
            std::ostringstream what;
            what << name << " weights row " << r;
            CompareRow(solver.GetWeights()[r], expected[r], what.str(),
                       worst, &reported);
        }

        CheckSamples(record, solver, name, worst, refusedSeen, negativeSeen);
    }
    std::printf("   %zu poses across them\n", poses);
}

}  // namespace

// ---------------------------------------------------------------------------

int
main(int argc, char **argv)
{
    std::string path = argc > 1 ? argv[1]
                                : std::string(RIGEXEC_RBF_FIXTURE);

    std::ifstream file(path.c_str(), std::ios::binary);
    if (!file) {
        std::printf("FAIL: cannot open the parity fixture at %s\n"
                    "      Regenerate it with "
                    "`python tools/biped/gen_psd_parity.py`.\n",
                    path.c_str());
        return 1;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    const std::string source = buffer.str();

    JsonParser parser(source);
    JsonPtr root = parser.Parse();
    if (!parser.Ok() || !root || root->kind != Json::KindObject) {
        std::printf("FAIL: cannot parse %s: %s\n", path.c_str(),
                    parser.Error().c_str());
        return 1;
    }
    // Name the binary and the fixture on every run. This test links
    // rigExecMath statically and loads no rigExec.dll at all, but a result
    // quoted as "the port is faithful" has to be traceable to the build it
    // came out of -- another agent on this box spent half an hour measuring
    // its own module against a different build's library because nothing
    // printed which one it had.
    std::printf("testRigExecRbf: binary %s\n", argv[0]);
    std::printf("testRigExecRbf: fixture %s (%zu bytes), oracle %s\n",
                path.c_str(), source.size(),
                root->Text("oracle").c_str());

    TestKernelsAndMetric();

    Worst worst;
    int refusedSeen = 0, negativeSeen = 0, perPose = 0, translationDriven = 0;
    TestCases(*root, &worst, &refusedSeen, &negativeSeen);
    TestInterpolators(*root, &worst, &refusedSeen, &negativeSeen, &perPose,
                      &translationDriven);

    // Coverage of the branches this port is most likely to get wrong. A
    // fixture that quietly stopped exercising one of them would leave this
    // file green while testing nothing, so the counts are asserted rather
    // than printed.
    CHECK(refusedSeen > 0);        // |sum| < 1e-6 leaves the weights alone
    CHECK(negativeSeen > 0);       // negatives fall out of the inverse
    CHECK(perPose > 0);            // asymmetric matrix, transposed inverse
    CHECK(translationDriven > 0);  // the face's translation channel

    std::printf("   worst disagreement %.3g (tolerance %.1g)%s%s\n",
                worst.value, kTolerance, worst.where.empty() ? "" : " at ",
                worst.where.c_str());
    std::printf("   %d normalisation refusals, %d negative weights, "
                "%d per-pose fits, %d translation-driven\n",
                refusedSeen, negativeSeen, perPose, translationDriven);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecRbf: all tests passed\n");
    return 0;
}
