#pragma once

// Motion-range guides for a selected joint, as plain line segments in the
// joint's attachment-A frame.  Maya-free so the geometry can be unit-tested;
// RigidBodyJointGeometryOverride only maps the segments to the viewport.
// Nothing here depends on the camera, so every viewport shows the same guides.
//
//   - translation: the allowed segment (offset ruler with witness lines),
//     rectangle or box; a fixed non-zero offset is a target cube with a
//     dashed line from B's current attachment; free axes are open arrows;
//   - rotation: one arc per axis with end stops, an outer zero notch and a
//     needle at the current angle; a fixed non-zero angle is a dashed
//     reference sweep plus a target spoke; a free axis is a full ring
//     without stops.  The measured Y angle cannot pass +-90 degrees
//     (kMaxAngleY), so the part of a Y range beyond that is dashed and has
//     no end stop at the cut; a free Y is still a full ring when X and Z are
//     free too (the Euler angles flip instead).  Angles are drawn where B
//     really turns (kPhysicalSign: the constraint's sign is the opposite).
//     The dials nest like a gimbal, each in the frame its angle is measured
//     in: Z in A's frame, Y turned by the current Z, X turned by Z and Y;
//   - effective springs: on a rotation axis the arc itself zigzags across
//     its allowed span; on a translation axis a zigzag badge plus the
//     equilibrium cross.  Neither encodes stiffness;
//   - a limit B presses on is drawn bold (`highlight`): the needle and its
//     end stop, or the ruler end / region faces in that axis's colour.
//
// Dashed always means "not reachable": a reference, never an allowed range.

#include "common.hpp"
#include "joint_display_ranges.hpp"
#include "joint_display_sizes.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace mmd::core::joint_display
{
enum class GuideColor
{
    Neutral,
    AxisX,
    AxisY,
    AxisZ
};

struct GuideSegment
{
    Double3 a;
    Double3 b;
    GuideColor color = GuideColor::Neutral;
    float width = 2.0f;
    bool dashed = false;
    bool highlight = false; // B presses on the limit this segment marks
};

struct GuideInput
{
    Ranges ranges;
    GuideSizes sizes{};
    Double3 currentB; // B's current attachment, in the A frame
    // The constraint's current angle per axis (currentAngles), drawn as a
    // needle on each dial when known.
    bool hasCurrentAngles = false;
    Double3 currentAngles;
};

namespace guide_detail
{
struct V
{
    double x = 0, y = 0, z = 0;
    V() = default;
    V(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    explicit V(const Double3& d) : x(d.x), y(d.y), z(d.z) {}
    Double3 d3() const { return {x, y, z}; }
    double& operator[](int i) { return i == 0 ? x : i == 1 ? y : z; }
    double operator[](int i) const { return i == 0 ? x : i == 1 ? y : z; }
    V operator+(const V& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V operator-(const V& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V operator*(double s) const { return {x * s, y * s, z * s}; }
};

// `v` turned right-handed by `angle` about coordinate axis `axis`.
inline V rotated(const V& v, int axis, double angle)
{
    const double c = std::cos(angle), s = std::sin(angle);
    const int i = (axis + 1) % 3, j = (axis + 2) % 3;
    V out = v;
    out[i] = v[i] * c - v[j] * s;
    out[j] = v[i] * s + v[j] * c;
    return out;
}

// Which ends of a limited axis B presses on.  The solver holds a body just
// inside its stop or lets it sink slightly past, so "on the limit" means
// within a small tolerance: 2% of the range, at least `floor`, and never
// more than half the range, so the middle of a tiny range is not a hit.
struct Hits
{
    bool lower = false;
    bool upper = false;
};
inline Hits limitHits(const Range& range, double value, double floor)
{
    if (range.kind() != RangeKind::Limited)
        return {};
    const double width = range.upper - range.lower;
    const double tolerance = std::min(std::max(width * 0.02, floor), width * 0.45);
    return {value <= range.lower + tolerance, value >= range.upper - tolerance};
}
} // namespace guide_detail

inline std::vector<GuideSegment> motionGuides(const GuideInput& in)
{
    using guide_detail::Hits;
    using guide_detail::limitHits;
    using guide_detail::rotated;
    using guide_detail::V;
    constexpr double kPi = 3.14159265358979323846;
    // Width of a limit B presses on (needles get their own, bolder width).
    constexpr float kPressedWidth = 4.5f;
    // The smallest angle that counts as pressing on a stop (0.5 degrees).
    constexpr double kAngleHitFloor = 0.5 * kPi / 180.0;
    const Ranges& ranges = in.ranges;
    const double radius = in.sizes.radius;
    const double tick = in.sizes.tick;
    const V origin;
    const V axes[3] = {V(1, 0, 0), V(0, 1, 0), V(0, 0, 1)};
    const GuideColor axisColor[3] = {GuideColor::AxisX, GuideColor::AxisY, GuideColor::AxisZ};
    const V currentB(in.currentB);

    std::vector<GuideSegment> out;
    GuideColor color = GuideColor::Neutral;
    float width = 2.0f;
    bool dashed = false;
    bool highlight = false;
    const auto line = [&](const V& a, const V& b)
    { out.push_back({a.d3(), b.d3(), color, width, dashed, highlight}); };
    const auto cross = [&](const V& p, double size)
    {
        for (const auto& axis : axes)
            line(p - axis * size, p + axis * size);
    };
    // Axis-aligned wire cube: reads as a target from any view.
    const auto cube = [&](const V& p, double size)
    {
        for (int a = 0; a < 3; ++a)
        {
            const V u = axes[(a + 1) % 3] * size, v = axes[(a + 2) % 3] * size;
            for (double su : {-1.0, 1.0})
                for (double sv : {-1.0, 1.0})
                    line(p + u * su + v * sv - axes[a] * size,
                         p + u * su + v * sv + axes[a] * size);
        }
    };

    // --- translation region ------------------------------------------------
    V lo, hi;
    bool region = false;
    for (int a = 0; a < 3; ++a)
    {
        const Range& range = ranges.linear[a];
        // Unbounded axes are shown separately as open arrows, not finite box faces.
        lo[a] = range.kind() == RangeKind::Free ? 0.0 : range.lower;
        hi[a] = range.kind() == RangeKind::Free ? 0.0 : range.upper;
        region = region || lo[a] != 0.0 || hi[a] != 0.0;
    }
    int dimensions = 0, varyingAxis = 0;
    bool anyFree = false;
    for (int a = 0; a < 3; ++a)
    {
        if (lo[a] != hi[a])
        {
            ++dimensions;
            varyingAxis = a;
        }
        anyFree = anyFree || ranges.linear[a].kind() == RangeKind::Free;
    }
    // The ends of the limited axes B presses on.
    Hits linearHits[3];
    for (int a = 0; a < 3; ++a)
        linearHits[a] = limitHits(ranges.linear[a], currentB[a], tick * 0.25);
    if (region && dimensions == 1)
    {
        // Offset the ruler sideways; witness lines keep its physical endpoints
        // unambiguous even when the A/B links lie on this axis.
        const V offset = axes[(varyingAxis + 1) % 3] * in.sizes.rulerOffset;
        width = 3.0f;
        line(lo + offset, hi + offset);
        // An end B presses on turns bold, in its axis colour.
        const auto end = [&](const V& p, bool pressed)
        {
            color = pressed ? axisColor[varyingAxis] : GuideColor::Neutral;
            highlight = pressed;
            width = pressed ? kPressedWidth : 1.0f;
            line(p, p + offset * 1.25);
            width = pressed ? kPressedWidth : 3.0f;
            line(p + offset * .8, p + offset * 1.2);
        };
        end(lo, linearHits[varyingAxis].lower);
        end(hi, linearHits[varyingAxis].upper);
        color = GuideColor::Neutral;
        highlight = false;
        width = 2.0f;
    }
    else if (region)
    {
        // A face B presses on: its edges turn bold, in that axis's colour.
        const auto pressed = [&](int axis, int side)
        {
            return lo[axis] != hi[axis] &&
                   (side ? linearHits[axis].upper : linearHits[axis].lower);
        };
        bool edges = false;
        for (int a = 0; a < 3; ++a)
        {
            if (lo[a] == hi[a])
                continue;
            const int u = (a + 1) % 3, v = (a + 2) % 3;
            for (int iu = 0; iu < (lo[u] == hi[u] ? 1 : 2); ++iu)
                for (int iv = 0; iv < (lo[v] == hi[v] ? 1 : 2); ++iv)
                {
                    V p = lo;
                    p[u] = iu ? hi[u] : lo[u];
                    p[v] = iv ? hi[v] : lo[v];
                    V q = p;
                    q[a] = hi[a];
                    const int face = pressed(u, iu) ? u : pressed(v, iv) ? v : -1;
                    color = face < 0 ? GuideColor::Neutral : axisColor[face];
                    highlight = face >= 0;
                    width = face < 0 ? 2.0f : kPressedWidth;
                    line(p, q);
                    edges = true;
                }
        }
        color = GuideColor::Neutral;
        highlight = false;
        width = 2.0f;
        if (!edges && !anyFree)
        {
            // Required position (cube), distinct from B's current diamond.
            width = 3.0f;
            cube(lo, tick * 2);
            width = 1.5f;
            dashed = true;
            line(currentB, lo);
            dashed = false;
            width = 2.0f;
        }
    }

    // --- free translation arrows and linear spring badges --------------------
    bool linearSpring = false;
    for (int a = 0; a < 3; ++a)
    {
        color = axisColor[a];
        if (ranges.linear[a].kind() == RangeKind::Free)
        {
            V center = lo;
            center[a] = 0;
            for (double sign : {-1.0, 1.0})
            {
                const V tip = center + axes[a] * (radius * 1.25 * sign);
                line(center, tip);
                line(tip, tip - axes[a] * (tick * 2 * sign) + axes[(a + 1) % 3] * tick);
                line(tip, tip - axes[a] * (tick * 2 * sign) - axes[(a + 1) % 3] * tick);
            }
        }
        if (ranges.linearSpring[a])
        {
            // Zigzag axis badge, not a motion boundary or a stiffness-scaled length.
            V prev = origin + axes[a] * (radius * 1.5);
            for (int k = 1; k <= 8; ++k)
            {
                const V p = origin + axes[a] * (radius * 1.5 + k * tick) +
                            axes[(a + 1) % 3] * (k == 8 ? 0.0 : (k % 2 ? tick : -tick));
                line(prev, p);
                prev = p;
            }
            linearSpring = true;
        }
    }
    if (linearSpring)
    {
        color = GuideColor::Neutral;
        cross(origin, tick); // This solver leaves spring equilibrium at zero.
    }

    // --- rotation: per-axis angle guides, not a combined 6DOF envelope -------
    // The constraint composes B's rotation as Z (about A's Z axis), then Y,
    // then X (about B's own X axis; see currentAngles).  Each dial lies in
    // the frame its angle is measured in, like the rings of a gimbal: Z in
    // A's frame, Y turned by the current Z, X turned by the current Z and Y.
    // Every needle then points where B really is, and every stop sits where
    // B meets it from its current pose.  Without current angles the dials
    // are at rest.
    const auto dialFrame = [&](int a, const V& v)
    {
        if (!in.hasCurrentAngles || a == 2)
            return v;
        const V turned = a == 0 ? rotated(v, 1, kPhysicalSign * in.currentAngles.y) : v;
        return rotated(turned, 2, kPhysicalSign * in.currentAngles.z);
    };
    for (int a = 0; a < 3; ++a)
    {
        const auto& range = ranges.angular[a];
        if (range.kind() == RangeKind::Fixed && range.lower == 0)
            continue;
        color = axisColor[a];
        const double r = radius * (1.0 + a * .22);
        const V dialU = dialFrame(a, axes[(a + 1) % 3]);
        const V dialV = dialFrame(a, axes[(a + 2) % 3]);
        // Angles are the constraint's; the dial shows where B really turns
        // (kPhysicalSign), so a limit [lo, hi] is drawn at [-hi, -lo].
        const auto at = [&](double angle, double distance)
        {
            const double turn = kPhysicalSign * angle;
            return origin + (dialU * std::cos(turn) + dialV * std::sin(turn)) * distance;
        };
        const bool fixed = range.kind() == RangeKind::Fixed;
        const bool unlimited = range.kind() == RangeKind::Free;
        const double lower = unlimited ? -kPi : fixed ? std::min(0.0, range.lower) : range.lower;
        const double upper = unlimited ? kPi : fixed ? std::max(0.0, range.upper) : range.upper;
        // The part of the range the solver can actually reach.  The measured
        // Y angle cannot pass +-90 degrees (kMaxAngleY), but with X and Z
        // free too a free Y still turns all the way round, so it is a ring.
        const bool flips = ranges.angular[0].kind() == RangeKind::Free &&
                           ranges.angular[2].kind() == RangeKind::Free;
        const bool capped = a == 1 && !fixed && !(unlimited && flips);
        const double reachLower = capped ? std::max(lower, -kMaxAngleY) : lower;
        const double reachUpper = capped ? std::min(upper, kMaxAngleY) : upper;
        const bool ring = unlimited && !capped;
        // The real stops B presses on (the +-90 degree cut is not a stop).
        Hits hits;
        if (in.hasCurrentAngles)
            hits = limitHits(range, in.currentAngles[a], kAngleHitFloor);
        hits.lower = hits.lower && reachLower == lower;
        hits.upper = hits.upper && reachUpper == upper;

        const auto arc = [&](double from, double to)
        {
            const int segments = std::max(1, static_cast<int>(std::ceil((to - from) * 12.0)));
            V prev = at(from, r);
            for (int k = 1; k <= segments; ++k)
            {
                const V next = at(from + (to - from) * k / segments, r);
                line(prev, next);
                prev = next;
            }
        };

        // For a locked nonzero angle, a dashed reference sweep is NOT an
        // allowed interval. The solid spoke and square mark the single target.
        dashed = fixed;
        if (reachLower >= reachUpper)
        {
            // Nothing of a capped range is reachable; only the dashed part remains.
        }
        else if (ranges.angularSpring[a])
        {
            // An effective spring turns the arc itself into a zigzag over the
            // whole allowed span (one element: limits + spring).  Its ends stay
            // on the dial so the end stops still mark the limits; a full ring
            // gets an even tooth count so it closes cleanly.
            // Teeth are spaced by the zigzag's own depth (each stroke rises
            // 2 ticks over ~1.2 ticks of arc, about 60°), so every tooth has
            // the same shape and the count grows with the angle.
            const double arcLength = (reachUpper - reachLower) * r;
            int teeth = std::max(2, static_cast<int>(std::lround(arcLength / (1.2 * tick))));
            if (ring && teeth % 2)
                ++teeth;
            const auto tooth = [&](int k)
            {
                const bool end = !ring && (k == 0 || k == teeth);
                const double offset = end ? 0.0 : (k % 2 ? tick : -tick);
                return at(reachLower + (reachUpper - reachLower) * k / teeth, r + offset);
            };
            for (int k = 0; k < teeth; ++k)
                line(tooth(k), tooth(k + 1));
        }
        else
            arc(reachLower, reachUpper);
        dashed = true;
        if (lower < reachLower)
            arc(lower, std::min(reachLower, upper));
        if (upper > reachUpper)
            arc(std::max(reachUpper, lower), upper);
        dashed = false;
        if (fixed)
        {
            // Target spoke and a square in the dial's plane, plus its stop.
            const V p = at(range.lower, r);
            const V u = dialU * tick, v = dialV * tick;
            line(origin, p);
            line(p - u - v, p + u - v);
            line(p + u - v, p + u + v);
            line(p + u + v, p - u + v);
            line(p - u + v, p - u - v);
            line(at(range.lower, r - tick), at(range.lower, r + tick));
        }
        else if (!ring && reachLower < reachUpper)
        {
            // End stops only at the limits themselves: where a capped Y range
            // is cut at +-90 degrees, nothing stops the body.  The stop B
            // presses on is bold and longer.
            const auto stop = [&](double angle, bool pressed)
            {
                highlight = pressed;
                width = pressed ? kPressedWidth : 2.0f;
                const double reach = pressed ? tick * 1.5 : tick;
                line(at(angle, r - reach), at(angle, r + reach));
                highlight = false;
                width = 2.0f;
            };
            if (reachLower == lower)
                stop(reachLower, hits.lower);
            if (reachUpper == upper)
                stop(reachUpper, hits.upper);
        }
        if (in.hasCurrentAngles)
        {
            // Needle at B's current angle: bolder and longer than the end
            // stops, crossing the dial.  Pressing on a stop makes it bolder
            // still, so a joint held at its limit reads at once.
            const bool pressed = hits.lower || hits.upper;
            highlight = pressed;
            width = pressed ? 6.0f : 3.5f;
            line(at(in.currentAngles[a], r * 0.6),
                 at(in.currentAngles[a], r + tick * (pressed ? 2.5 : 1.5)));
            highlight = false;
            width = 2.0f;
        }
        // A notch outside the dial cannot disappear inside a radial body link.
        color = GuideColor::Neutral;
        const V zero = at(0, r + tick * 2);
        const V tangent = dialV * tick;
        line(zero - tangent, at(0, r));
        line(at(0, r), zero + tangent);
        line(zero - tangent, zero + tangent);
    }
    return out;
}
} // namespace mmd::core::joint_display
