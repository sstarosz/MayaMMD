#pragma once

#include "rigid_body_simulation.hpp"
#include <array>
#include <cmath>

namespace mmd::core::joint_display
{
// The single freedom model behind every joint display (state colour, motion
// guides, tests).  It follows btGeneric6DofSpringConstraint exactly: there
// are no display thresholds, so what is drawn is what the solver does.
enum class RangeKind
{
    Fixed,
    Limited,
    Free
};
struct Range
{
    double lower = 0.0;
    double upper = 0.0;
    RangeKind kind() const
    {
        // Match Bullet: only equal endpoints are locked; even tiny ranges count.
        return lower > upper    ? RangeKind::Free
               : lower == upper ? RangeKind::Fixed
                                : RangeKind::Limited;
    }
};
struct Ranges
{
    std::array<Range, 3> linear{};
    std::array<Range, 3> angular{};
    // A spring only acts on an axis that can move: on a locked axis the
    // limit wins and the stiffness has no effect, so it is not reported.
    std::array<bool, 3> linearSpring{};
    std::array<bool, 3> angularSpring{};
};

// The constraint measures rotation as XYZ Euler angles, so the middle (Y)
// angle never leaves [-pi/2, pi/2]: wider Y limits, or a free Y axis, cannot
// be reached past that.  Turning further reads as Y coming back while X and
// Z jump by pi, which only free X and Z axes let through.
constexpr double kMaxAngleY = 1.57079632679489661923;

namespace range_detail
{
// Bullet stores limits as btScalar (float): compare at that precision, or two
// values a float apart would read as a tiny range here and as locked there.
inline double scalar(double v)
{
    return static_cast<double>(static_cast<float>(v));
}
// btNormalizeAngle, in float like Bullet (setAngularLower/UpperLimit).
inline double normalizeAngle(double radians)
{
    constexpr float kPi = 3.1415926535897932384626433832795029f;
    constexpr float kTwoPi = 2.0f * kPi;
    const float a = std::fmod(static_cast<float>(radians), kTwoPi);
    return static_cast<double>(a < -kPi ? a + kTwoPi : a > kPi ? a - kTwoPi : a);
}
} // namespace range_detail

// Spring6Dof (the only PMX 2.0 joint type): XYZ intervals in the joint frame.
inline Ranges describe(const RigidBodySimulation::JointDefinition& j)
{
    using range_detail::normalizeAngle;
    using range_detail::scalar;
    Ranges out;
    for (int i = 0; i < 3; ++i)
    {
        out.linear[i] = {scalar(j.linearMin[i]), scalar(j.linearMax[i])};
        out.angular[i] = {normalizeAngle(j.angularMin[i]), normalizeAngle(j.angularMax[i])};
        // The solver enables a spring for any non-zero stiffness.
        out.linearSpring[i] =
            j.linearStiffness[i] != 0.0 && out.linear[i].kind() != RangeKind::Fixed;
        out.angularSpring[i] =
            j.angularStiffness[i] != 0.0 && out.angular[i].kind() != RangeKind::Fixed;
    }
    return out;
}

// The joint's current angle per axis, measured exactly as the constraint
// measures it against its limits (btGeneric6DofConstraint::calculateAngleInfo:
// matrixToEulerXYZ of A^-1 * B).  `a` / `b` are the attachment frames' world
// rotations as unit quaternions {x, y, z, w}.
//
// Bullet's sign is the OPPOSITE of a right-handed rotation: its
// matrixToEulerXYZ reads the matrix column-major (btGetMatrixElem), so it
// decomposes the transpose.  B turned right-handed by +t about A's X reads
// -t, and a limit [lo, hi] lets B turn through [-hi, -lo] (kPhysicalSign).
inline Double3 currentAngles(const Double4& a, const Double4& b)
{
    // Row-major rotation matrix of a unit quaternion (btMatrix3x3::setRotation).
    const auto matrix = [](const Double4& q, double m[3][3])
    {
        const double xx = q.x * q.x * 2, yy = q.y * q.y * 2, zz = q.z * q.z * 2;
        const double xy = q.x * q.y * 2, xz = q.x * q.z * 2, yz = q.y * q.z * 2;
        const double wx = q.w * q.x * 2, wy = q.w * q.y * 2, wz = q.w * q.z * 2;
        const double rows[3][3] = {{1 - (yy + zz), xy - wz, xz + wy},
                                   {xy + wz, 1 - (xx + zz), yz - wx},
                                   {xz - wy, yz + wx, 1 - (xx + yy)}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m[i][j] = rows[i][j];
    };
    double ma[3][3], mb[3][3], r[3][3];
    matrix(a, ma);
    matrix(b, mb);
    for (int i = 0; i < 3; ++i) // r = A^T * B
        for (int j = 0; j < 3; ++j)
            r[i][j] = ma[0][i] * mb[0][j] + ma[1][i] * mb[1][j] + ma[2][i] * mb[2][j];
    // matrixToEulerXYZ with btGetMatrixElem(m, k) = m[k % 3][k / 3],
    // including its gimbal-lock branches.
    const auto elem = [&](int k) { return r[k % 3][k / 3]; };
    const double sy = elem(2);
    if (sy >= 1.0)
        return {std::atan2(elem(3), elem(4)), kMaxAngleY, 0.0};
    if (sy <= -1.0)
        return {-std::atan2(elem(3), elem(4)), -kMaxAngleY, 0.0};
    return {std::atan2(-elem(5), elem(8)), std::asin(sy), std::atan2(-elem(1), elem(0))};
}

// The right-handed rotation of B about each axis of A is kPhysicalSign times
// the constraint's angle (see currentAngles): the guides draw every angle
// times this, so arcs, stops and needles sit where the body really turns.
constexpr double kPhysicalSign = -1.0;

// Whole-joint summary for the unselected state display: which kinds of
// motion the joint permits.
struct Freedom
{
    bool translates = false; // some linear axis is limited or free
    bool rotates = false;    // some angular axis is limited or free
};
inline Freedom summarize(const Ranges& ranges)
{
    Freedom out;
    for (int i = 0; i < 3; ++i)
    {
        out.translates = out.translates || ranges.linear[i].kind() != RangeKind::Fixed;
        out.rotates = out.rotates || ranges.angular[i].kind() != RangeKind::Fixed;
    }
    return out;
}
} // namespace mmd::core::joint_display
