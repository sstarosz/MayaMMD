/*
 * SPDX-License-Identifier: MIT
 *
 * test_rigid_body_simulation.cpp
 *
 * Unit tests for the Maya-free physics engine (RigidBodySimulation).  This target
 * links ONLY Bullet + Catch2 (no Maya SDK) â€” compiling and passing here is
 * what proves the engine is truly Maya-free.
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "joint_display_ranges.hpp"
#include "joint_display_sizes.hpp"
#include "joint_guides.hpp"
#include "joint_parameters.hpp"
#include "physics_math.hpp"
#include "rigid_body_simulation.hpp"

#include <btBulletDynamicsCommon.h>

#include <algorithm>
#include <cmath>
#include <vector>

using mmd::core::applyShapeSize;
using mmd::core::Double3;
using mmd::core::Double4;
using mmd::core::RigidBodySimulation;
using namespace mmd::core::physics_math;

TEST_CASE("Joint guides scale with the hub and clear it", "[joint-display]")
{
    using mmd::core::joint_display::guideSizes;
    for (double hub : {.005, .13, 2.0})
    {
        const auto size = guideSizes(hub);
        REQUIRE(size.radius - size.tick > hub);
        REQUIRE(size.rulerOffset > hub);
        REQUIRE(size.attachmentMarker > hub);
    }
    REQUIRE(guideSizes(.26).radius == Catch::Approx(2 * guideSizes(.13).radius));
}

namespace
{

void requireDouble3(const Double3& actual, const Double3& expected, double tol = 1e-12)
{
    REQUIRE(std::fabs(actual.x - expected.x) <= tol);
    REQUIRE(std::fabs(actual.y - expected.y) <= tol);
    REQUIRE(std::fabs(actual.z - expected.z) <= tol);
}

// A single dynamic sphere at the origin with gravity.
RigidBodySimulation::Definition gravityDefinition(double gy)
{
    RigidBodySimulation::Definition def;
    def.gravity = Double3(0.0, gy, 0.0);

    RigidBodySimulation::BodyDefinition bd;
    bd.mass = 1.0;
    bd.colliderType = RigidBodySimulation::ColliderType::eSphere;
    bd.radius = 0.5;
    bd.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    def.bodies.push_back(bd);
    return def;
}

// A kinematic anchor at the origin + a dynamic body 1 unit above it, rigidly
// welded (SPRING_6DOF with zero springs + zero limits = locked).
RigidBodySimulation::Definition weldDefinition()
{
    RigidBodySimulation::Definition def;
    def.gravity[0] = 0.0;
    def.gravity[1] = 0.0;
    def.gravity[2] = 0.0;

    RigidBodySimulation::BodyDefinition anchor;
    anchor.colliderType = RigidBodySimulation::ColliderType::eSphere;
    anchor.radius = 0.5;
    anchor.physicsMode = RigidBodySimulation::PhysicsMode::eFollowBone;
    def.bodies.push_back(anchor);

    RigidBodySimulation::BodyDefinition dynamic;
    dynamic.colliderType = RigidBodySimulation::ColliderType::eSphere;
    dynamic.radius = 0.5;
    dynamic.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    dynamic.restPos = Double3(0.0, 1.0, 0.0);
    dynamic.resetAnchorIndex = 0; // anchored to the kinematic body
    def.bodies.push_back(dynamic);

    RigidBodySimulation::JointDefinition weld;
    weld.bodyA = 0;
    weld.bodyB = 1;
    weld.type = 0; // SPRING_6DOF
    weld.frameT = Double3(0.0, 0.5, 0.0);
    def.joints.push_back(weld);
    return def;
}

} // namespace

TEST_CASE("Joint display distinguishes fixed offsets, tiny ranges and free axes", "[joint-display]")
{
    using namespace mmd::core::joint_display;
    RigidBodySimulation::JointDefinition j;
    j.type = 0;
    j.linearMin = Double3(2.0, 0.0, 1.0);
    j.linearMax = Double3(2.0, 1e-7, -1.0);
    j.linearStiffness = Double3(3, 0, 0);
    j.angularStiffness = Double3(0, 4, 0);
    const auto ranges = describe(j);
    REQUIRE(ranges.linear[0].kind() == RangeKind::Fixed);
    REQUIRE(ranges.linear[0].lower == 2.0);
    REQUIRE(ranges.linear[1].kind() == RangeKind::Limited);
    REQUIRE(ranges.linear[2].kind() == RangeKind::Free);
    // Stiffness on a locked axis has no effect, so it is not a spring.
    REQUIRE_FALSE(ranges.linearSpring[0]);
    REQUIRE_FALSE(ranges.linearSpring[1]);
    REQUIRE_FALSE(ranges.angularSpring[1]);
    j.angularMin.y = -.01;
    j.angularMax.y = .01;
    REQUIRE(describe(j).angularSpring[1]);
    j.linearStiffness = Double3(0, 0, 5);
    REQUIRE(describe(j).linearSpring[2]); // free axes can be sprung too
}

TEST_CASE("Joint freedom summary reports the kinds of motion", "[joint-display]")
{
    using namespace mmd::core::joint_display;
    RigidBodySimulation::JointDefinition j;
    auto f = summarize(describe(j));
    REQUIRE_FALSE(f.translates);
    REQUIRE_FALSE(f.rotates);

    // Welded but with stiffness everywhere: still welded.
    j.linearStiffness = Double3(10, 10, 10);
    j.angularStiffness = Double3(10, 10, 10);
    f = summarize(describe(j));
    REQUIRE_FALSE(f.translates);
    REQUIRE_FALSE(f.rotates);

    // A tiny (sub-degree) range is real freedom.
    j.angularMin.z = -.005;
    j.angularMax.z = .005;
    f = summarize(describe(j));
    REQUIRE(f.rotates);
    REQUIRE_FALSE(f.translates);

    j.linearMin.x = 1;
    j.linearMax.x = -1;
    f = summarize(describe(j));
    REQUIRE(f.translates);
    REQUIRE(f.rotates);
}

namespace
{
using mmd::core::joint_display::GuideColor;
using mmd::core::joint_display::GuideSegment;
// radius 0.56, tick ~0.05, ruler offset ~0.36
const auto kTestSizes = mmd::core::joint_display::guideSizes(0.224);

std::vector<GuideSegment> guidesFor(const RigidBodySimulation::JointDefinition& j,
                                    const Double3& currentB = Double3())
{
    using namespace mmd::core::joint_display;
    GuideInput in;
    in.ranges = describe(j);
    in.sizes = kTestSizes;
    in.currentB = currentB;
    return motionGuides(in);
}

size_t countGuides(const std::vector<GuideSegment>& segments, GuideColor color, bool dashed)
{
    return static_cast<size_t>(std::count_if(segments.begin(), segments.end(),
                                             [&](const GuideSegment& s)
                                             { return s.color == color && s.dashed == dashed; }));
}

double distanceFromOrigin(const Double3& p)
{
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}
} // namespace

TEST_CASE("Motion guides: welded joints draw nothing, locked springs included", "[joint-display]")
{
    RigidBodySimulation::JointDefinition j;
    REQUIRE(guidesFor(j).empty());
    j.linearStiffness = Double3(5, 5, 5);
    j.angularStiffness = Double3(5, 5, 5);
    REQUIRE(guidesFor(j).empty());
}

TEST_CASE("Motion guides: one linear axis is an offset ruler at physical length", "[joint-display]")
{
    RigidBodySimulation::JointDefinition j;
    j.linearMin.x = -1.0;
    j.linearMax.x = 1.5;
    const auto segments = guidesFor(j);
    // The bold bar spans the real limits along X, offset along Y.
    const auto bar = std::find_if(segments.begin(), segments.end(), [](const GuideSegment& s)
                                  { return s.width == 3.0f && std::abs(s.a.x - s.b.x) > 1.0; });
    REQUIRE(bar != segments.end());
    REQUIRE(std::min(bar->a.x, bar->b.x) == Catch::Approx(-1.0));
    REQUIRE(std::max(bar->a.x, bar->b.x) == Catch::Approx(1.5));
    REQUIRE(bar->a.y == Catch::Approx(kTestSizes.rulerOffset));
    REQUIRE(bar->a.z == Catch::Approx(0.0));
    REQUIRE(countGuides(segments, GuideColor::AxisX, false) == 0); // no arrows, no spring
}

TEST_CASE("Motion guides: a fixed linear offset is a target with a dashed error line",
          "[joint-display]")
{
    RigidBodySimulation::JointDefinition j;
    j.linearMin = j.linearMax = Double3(1.2, 0.6, 0.0);
    const auto segments = guidesFor(j, Double3(0.2, 0.1, 0.0));
    REQUIRE(countGuides(segments, GuideColor::Neutral, true) == 1);
    const auto dash = std::find_if(segments.begin(), segments.end(),
                                   [](const GuideSegment& s) { return s.dashed; });
    REQUIRE(dash->a.x == Catch::Approx(0.2));
    REQUIRE(dash->b.x == Catch::Approx(1.2));
    REQUIRE(dash->b.y == Catch::Approx(0.6));
}

TEST_CASE("Motion guides: free linear axes are arrows, not region edges", "[joint-display]")
{
    RigidBodySimulation::JointDefinition j;
    j.linearMin.y = 1.0;
    j.linearMax.y = -1.0;
    const auto segments = guidesFor(j);
    REQUIRE(countGuides(segments, GuideColor::AxisY, false) == 6); // two 3-stroke arrows
    REQUIRE(countGuides(segments, GuideColor::Neutral, false) == 0);
}

TEST_CASE("Motion guides: angular arcs follow the limits; fixed angles are dashed",
          "[joint-display]")
{
    RigidBodySimulation::JointDefinition j;
    j.angularMin.z = -0.5;
    j.angularMax.z = 0.25;
    auto segments = guidesFor(j);
    REQUIRE(countGuides(segments, GuideColor::AxisZ, true) == 0);
    // Every Z arc vertex sits on the Z dial (radius 0.56 * 1.44) in the XY
    // plane, where B really turns: the constraint's [-0.5, 0.25] is the
    // right-handed [-0.25, 0.5] (kPhysicalSign).
    const double dial = 0.56 * 1.44;
    size_t onDial = 0;
    for (const auto& s : segments)
        if (s.color == GuideColor::AxisZ && s.width == 2.0f &&
            std::abs(distanceFromOrigin(s.a) - dial) < 1e-9 &&
            std::abs(distanceFromOrigin(s.b) - dial) < 1e-9)
        {
            REQUIRE(s.a.z == Catch::Approx(0.0));
            REQUIRE(std::atan2(s.a.y, s.a.x) >= -0.25 - 1e-9);
            REQUIRE(std::atan2(s.a.y, s.a.x) <= 0.5 + 1e-9);
            ++onDial;
        }
    REQUIRE(onDial >= 9); // ceil(0.75 * 12)

    j.angularMin.z = j.angularMax.z = 0.4;
    segments = guidesFor(j);
    REQUIRE(countGuides(segments, GuideColor::AxisZ, true) > 0);
}

TEST_CASE("Motion guides: a sprung rotation arc zigzags across its limits", "[joint-display]")
{
    RigidBodySimulation::JointDefinition j;
    j.angularMin.x = -0.6;
    j.angularMax.x = 0.6;
    const double dial = kTestSizes.radius; // X dial radius
    const auto arcPoints = [&](const std::vector<GuideSegment>& segments)
    {
        // The X arc: X-coloured, solid, width 2, in the YZ plane, near the dial.
        std::vector<Double3> points;
        for (const auto& s : segments)
            if (s.color == GuideColor::AxisX && !s.dashed && s.width == 2.0f &&
                std::abs(s.a.x) < 1e-12 && std::abs(s.b.x) < 1e-12 &&
                std::abs(distanceFromOrigin(s.a) - dial) < 0.1 &&
                std::abs(distanceFromOrigin(s.b) - dial) < 0.1 &&
                std::abs(std::atan2(s.a.z, s.a.y) - std::atan2(s.b.z, s.b.y)) > 1e-9)
            {
                points.push_back(s.a);
                points.push_back(s.b);
            }
        return points;
    };

    const auto smooth = arcPoints(guidesFor(j));
    REQUIRE_FALSE(smooth.empty());
    for (const auto& p : smooth)
        REQUIRE(distanceFromOrigin(p) == Catch::Approx(dial));

    j.angularStiffness.x = 10.0;
    const auto sprung = guidesFor(j);
    const auto zigzag = arcPoints(sprung);
    REQUIRE_FALSE(zigzag.empty());
    bool inside = false, outside = false;
    double first = 10, last = -10;
    for (const auto& p : zigzag)
    {
        const double d = distanceFromOrigin(p);
        inside = inside || d < dial - 1e-6;
        outside = outside || d > dial + 1e-6;
        const double angle = std::atan2(p.z, p.y);
        first = std::min(first, angle);
        last = std::max(last, angle);
    }
    REQUIRE(inside);
    REQUIRE(outside);
    // One element: the zigzag spans exactly the limits (no separate badge).
    REQUIRE(first == Catch::Approx(-0.6));
    REQUIRE(last == Catch::Approx(0.6));

    // Teeth keep their shape: the count grows with the angle (~1.2 ticks of
    // arc per stroke), so a span twice as wide has about twice the teeth.
    REQUIRE(zigzag.size() / 2 >= 10);
    j.angularMin.x = -1.2;
    j.angularMax.x = 1.2;
    const double ratio = static_cast<double>(arcPoints(guidesFor(j)).size()) / zigzag.size();
    REQUIRE(ratio == Catch::Approx(2.0).margin(0.15));
    j.angularMin.x = -0.6;
    j.angularMax.x = 0.6;

    j.angularStiffness.y = 10.0; // Y is locked at zero: nothing changes
    REQUIRE(guidesFor(j).size() == sprung.size());
}

TEST_CASE("Motion guides: Y stops only at real limits; free XYZ makes Y a ring", "[joint-display]")
{
    const double dial = kTestSizes.radius * 1.22; // the Y dial
    const auto ticks = [&](const std::vector<GuideSegment>& segments)
    {
        // End stops: Y-coloured radial strokes across the dial.
        return std::count_if(
            segments.begin(), segments.end(),
            [&](const GuideSegment& s)
            {
                return s.color == GuideColor::AxisY &&
                       std::abs(distanceFromOrigin(s.a) - (dial - kTestSizes.tick)) < 1e-9 &&
                       std::abs(distanceFromOrigin(s.b) - (dial + kTestSizes.tick)) < 1e-9;
            });
    };
    RigidBodySimulation::JointDefinition j;

    // Free Y with X and Z locked: solid to +-90 degrees, dashed beyond, and
    // no stops at the cut, where nothing stops the body.
    j.angularMin.y = 1.0;
    j.angularMax.y = -1.0;
    auto segments = guidesFor(j);
    REQUIRE(countGuides(segments, GuideColor::AxisY, false) > 0);
    REQUIRE(countGuides(segments, GuideColor::AxisY, true) > 0);
    REQUIRE(ticks(segments) == 0);

    // With X and Z free too, Y turns all the way round: a solid ring.
    j.angularMin.x = j.angularMin.z = 1.0;
    j.angularMax.x = j.angularMax.z = -1.0;
    segments = guidesFor(j);
    REQUIRE(countGuides(segments, GuideColor::AxisY, true) == 0);
    REQUIRE(ticks(segments) == 0);

    // A limited Y range past 90 degrees keeps its real stop only.
    j = {};
    j.angularMin.y = -0.5;
    j.angularMax.y = 2.0;
    segments = guidesFor(j);
    REQUIRE(countGuides(segments, GuideColor::AxisY, true) > 0);
    REQUIRE(ticks(segments) == 1);
    j.angularMax.y = 0.5;
    REQUIRE(ticks(guidesFor(j)) == 2);
}

TEST_CASE("PMX reflection negates and swaps linear Z and angular X/Y", "[joint-display][sim]")
{
    RigidBodySimulation::JointDefinition j;
    j.linearMin = Double3(.2, 3, 1);
    j.linearMax = Double3(.4, 5, 9);
    j.angularMin = Double3(.1, .2, .3);
    j.angularMax = Double3(.4, .5, .6);
    j.linearStiffness = Double3(1, 2, 3);
    j.angularStiffness = Double3(4, 5, 6);
    auto reflected = j;
    mmd::core::reflectJointParameters(reflected);
    requireDouble3(reflected.linearMin, Double3(.2, 3, -9));
    requireDouble3(reflected.linearMax, Double3(.4, 5, -1));
    requireDouble3(reflected.angularMin, Double3(-.4, -.5, .3));
    requireDouble3(reflected.angularMax, Double3(-.1, -.2, .6));
    requireDouble3(reflected.linearStiffness, j.linearStiffness);
    requireDouble3(reflected.angularStiffness, j.angularStiffness);
    mmd::core::reflectJointParameters(reflected);
    REQUIRE(reflected == j);
}

TEST_CASE("Angular wrapping reveals free axes rather than an apparent limited arc",
          "[joint-display]")
{
    RigidBodySimulation::JointDefinition j;
    j.angularMin.x = -4;
    j.angularMax.x = .2;
    const auto r = mmd::core::joint_display::describe(j).angular[0];
    REQUIRE(r.lower == Catch::Approx(2.283185307179586));
    REQUIRE(r.kind() == mmd::core::joint_display::RangeKind::Free);
}

namespace
{
// Bullet's own current angles for attachment frames at rotations qa / qb
// (the bodies sit at identity, so the frames are the world attachments).
Double3 bulletJointAngles(const Double4& qa, const Double4& qb)
{
    btSphereShape shape(0.1F);
    btRigidBody a(btRigidBody::btRigidBodyConstructionInfo(0.0F, nullptr, &shape));
    btRigidBody b(btRigidBody::btRigidBodyConstructionInfo(1.0F, nullptr, &shape,
                                                           btVector3(1.0F, 1.0F, 1.0F)));
    const auto frame = [](const Double4& q)
    {
        return btTransform(btQuaternion(static_cast<btScalar>(q.x), static_cast<btScalar>(q.y),
                                        static_cast<btScalar>(q.z), static_cast<btScalar>(q.w)));
    };
    btGeneric6DofSpringConstraint constraint(a, b, frame(qa), frame(qb), true);
    constraint.calculateTransforms();
    return {constraint.getAngle(0), constraint.getAngle(1), constraint.getAngle(2)};
}
} // namespace

TEST_CASE("Joint current angles match Bullet's own measurement", "[joint-display][sim]")
{
    // Euler-degree pairs (attachment A, attachment B), incl. near gimbal lock.
    const double pairs[][6] = {{0, 0, 0, 30, 0, 0},          {0, 0, 0, 0, 30, 0},
                               {0, 0, 0, 0, 0, 30},          {10, 20, 30, -40, 15, 70},
                               {5, -60, 170, -20, 10, -150}, {-35, 40, -20, 60, -25, 95},
                               {0, 0, 0, 12, 85, -7}};
    for (const auto& p : pairs)
    {
        const Double4 qa = eulerDegreesToQuat(p[0], p[1], p[2]);
        const Double4 qb = eulerDegreesToQuat(p[3], p[4], p[5]);
        const Double3 ours = mmd::core::joint_display::currentAngles(qa, qb);
        const Double3 bullet = bulletJointAngles(qa, qb);
        for (int i = 0; i < 3; ++i)
            REQUIRE(ours[i] == Catch::Approx(bullet[i]).margin(1e-4));
    }
}

TEST_CASE("The joint angle is kPhysicalSign times B's right-handed rotation",
          "[joint-display][sim]")
{
    // The motion guides draw where B really turns, kPhysicalSign times the
    // constraint's angle; with the wrong sign every asymmetric arc, stop and
    // needle is drawn mirrored.
    const Double4 identity(0.0, 0.0, 0.0, 1.0);
    for (int axis = 0; axis < 3; ++axis)
    {
        const double half = 0.2; // B turned right-handed by +0.4 rad about `axis`
        Double4 q(0.0, 0.0, 0.0, std::cos(half));
        q[axis] = std::sin(half);
        const Double3 bullet = bulletJointAngles(identity, q);
        REQUIRE(bullet[axis] ==
                Catch::Approx(mmd::core::joint_display::kPhysicalSign * 0.4).margin(1e-5));
    }
}

TEST_CASE("A settled joint sits inside its drawn arc, under its needle", "[joint-display][sim]")
{
    using namespace mmd::core::joint_display;
    // X limited to [0.3, 0.5] in the constraint's terms; B starts at 0, so
    // the limit turns it in.  No gravity: it coasts to rest inside the range.
    auto def = weldDefinition();
    def.joints[0].angularMin.x = 0.3;
    def.joints[0].angularMax.x = 0.5;
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));
    for (int i = 0; i < 120; ++i)
        sim.step(RigidBodySimulation::kFixedDt);
    RigidBodySimulation::Pose a, b;
    const auto& joint = def.joints[0];
    REQUIRE(sim.attachmentPose(0, joint.frameT, joint.frameR, a));
    REQUIRE(sim.attachmentPose(1, joint.frameT, joint.frameR, b));

    // The constraint's own measure is inside its limits ...
    const Double3 angles = currentAngles(a.quat, b.quat);
    REQUIRE(angles.x >= 0.3 - 0.02);
    REQUIRE(angles.x <= 0.5 + 0.02);
    // ... and B really turned the other way round (A is at identity, so the
    // attachment's quaternion is B's rotation about A's X).
    const double turned = 2.0 * std::atan2(b.quat.x, b.quat.w);
    REQUIRE(turned == Catch::Approx(kPhysicalSign * angles.x).margin(1e-4));

    // The needle points at B, inside the arc's physical span [-0.5, -0.3].
    GuideInput in;
    in.ranges = describe(joint);
    in.sizes = kTestSizes;
    in.hasCurrentAngles = true;
    in.currentAngles = angles;
    const auto segments = motionGuides(in);
    const auto needle = std::find_if(segments.begin(), segments.end(), [](const GuideSegment& s)
                                     { return s.color == GuideColor::AxisX && s.width >= 3.5f; });
    REQUIRE(needle != segments.end());
    REQUIRE(std::atan2(needle->b.z, needle->b.y) == Catch::Approx(turned).margin(1e-4));
    REQUIRE(distanceFromOrigin(needle->b) > kTestSizes.radius);
    REQUIRE(distanceFromOrigin(needle->a) < kTestSizes.radius);
    for (const auto& s : segments)
        if (s.color == GuideColor::AxisX && s.width == 2.0f && !s.dashed &&
            std::abs(distanceFromOrigin(s.a) - kTestSizes.radius) < 1e-9)
        {
            REQUIRE(std::atan2(s.a.z, s.a.y) >= -0.5 - 1e-6);
            REQUIRE(std::atan2(s.a.z, s.a.y) <= -0.3 + 1e-6);
        }
}

namespace
{
// Unit quaternion {x, y, z, w} turning right-handed by `angle` about `axis`.
Double4 axisQuat(int axis, double angle)
{
    Double4 q(0.0, 0.0, 0.0, std::cos(angle / 2));
    q[axis] = std::sin(angle / 2);
    return q;
}
Double4 quatMul(const Double4& a, const Double4& b)
{
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
// `v` rotated by the unit quaternion `q`.
Double3 quatRotate(const Double4& q, const Double3& v)
{
    const Double4 p = quatMul(quatMul(q, Double4(v.x, v.y, v.z, 0.0)),
                              Double4(-q.x, -q.y, -q.z, q.w));
    return {p.x, p.y, p.z};
}
double dot(const Double3& a, const Double3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Double3 unit(const Double3& v)
{
    const double length = distanceFromOrigin(v);
    return {v.x / length, v.y / length, v.z / length};
}
} // namespace

TEST_CASE("Motion guides: the dials nest like a gimbal, so needles point at B",
          "[joint-display]")
{
    using namespace mmd::core::joint_display;
    // B turns about A's Z, then about the turned Y, then about its own X:
    // the order the constraint measures its angles in.
    const double z = 0.4, y = 0.3, x = 0.5;
    const Double4 b = quatMul(quatMul(axisQuat(2, z), axisQuat(1, y)), axisQuat(0, x));
    GuideInput in;
    RigidBodySimulation::JointDefinition j;
    j.angularMin = Double3(-1, -1, -1);
    j.angularMax = Double3(1, 1, 1);
    in.ranges = describe(j);
    in.sizes = kTestSizes;
    in.hasCurrentAngles = true;
    in.currentAngles = currentAngles(Double4(0, 0, 0, 1), b);
    requireDouble3(in.currentAngles, Double3(-x, -y, -z), 1e-9);
    const auto segments = motionGuides(in);
    const auto needle = [&](GuideColor color)
    {
        const auto it = std::find_if(segments.begin(), segments.end(), [&](const GuideSegment& s)
                                     { return s.color == color && s.width >= 3.5f; });
        REQUIRE(it != segments.end());
        return unit(it->b);
    };

    // X turns with B: its needle is B's own Y axis, its dial is square to B's X.
    const Double3 xNeedle = needle(GuideColor::AxisX);
    requireDouble3(xNeedle, quatRotate(b, Double3(0, 1, 0)), 1e-9);
    const Double3 bX = quatRotate(b, Double3(1, 0, 0));
    size_t onXDial = 0;
    for (const auto& s : segments)
        if (s.color == GuideColor::AxisX && s.width == 2.0f && !s.dashed &&
            std::abs(distanceFromOrigin(s.a) - kTestSizes.radius) < 1e-9)
        {
            REQUIRE(dot(s.a, bX) == Catch::Approx(0.0).margin(1e-9));
            ++onXDial;
        }
    REQUIRE(onXDial > 0);
    // Y turns with Z only; Z stays in A's frame.
    const Double4 zTurn = axisQuat(2, z);
    requireDouble3(needle(GuideColor::AxisY),
                   quatRotate(quatMul(zTurn, axisQuat(1, y)), Double3(0, 0, 1)), 1e-9);
    requireDouble3(needle(GuideColor::AxisZ), quatRotate(zTurn, Double3(1, 0, 0)), 1e-9);
}

TEST_CASE("Motion guides: a limit B presses on is drawn bold", "[joint-display]")
{
    using namespace mmd::core::joint_display;
    const auto pressed = [](const std::vector<GuideSegment>& segments, GuideColor color)
    {
        return std::count_if(segments.begin(), segments.end(), [&](const GuideSegment& s)
                             { return s.highlight && s.color == color; });
    };

    // Rotation: the needle and the stop it presses on.
    RigidBodySimulation::JointDefinition j;
    j.angularMin.z = -0.5;
    j.angularMax.z = 0.25;
    GuideInput in;
    in.ranges = describe(j);
    in.sizes = kTestSizes;
    in.hasCurrentAngles = true;
    in.currentAngles = Double3(0, 0, 0.255); // slightly past the upper stop
    auto segments = motionGuides(in);
    REQUIRE(pressed(segments, GuideColor::AxisZ) == 2);
    for (const auto& s : segments)
        if (s.highlight)
            REQUIRE(s.width > 3.5f);
    in.currentAngles = Double3(0, 0, -0.1);
    REQUIRE(pressed(motionGuides(in), GuideColor::AxisZ) == 0);
    // The +-90 degree cut of a wider Y range is not a stop, even when the
    // configured end is only just past it.
    j = {};
    j.angularMin.y = -0.5;
    j.angularMax.y = 1.6;
    in.ranges = describe(j);
    in.currentAngles = Double3(0, kMaxAngleY, 0);
    REQUIRE(pressed(motionGuides(in), GuideColor::AxisY) == 0);

    // Translation, one axis: the ruler's end (witness line and end tick).
    j = {};
    j.linearMin.x = -1.0;
    j.linearMax.x = 1.5;
    REQUIRE(pressed(guidesFor(j, Double3(1.5, 0, 0)), GuideColor::AxisX) == 2);
    REQUIRE(pressed(guidesFor(j, Double3(0.2, 0, 0)), GuideColor::AxisX) == 0);

    // A box: each face B presses on, in its axis colour (the edge the two
    // faces share takes one colour).
    j.linearMin = Double3(-1, -1, -1);
    j.linearMax = Double3(1, 1, 1);
    segments = guidesFor(j, Double3(1, 0, -1));
    REQUIRE(pressed(segments, GuideColor::AxisX) + pressed(segments, GuideColor::AxisZ) == 7);
    REQUIRE(pressed(segments, GuideColor::AxisY) == 0);
    REQUIRE(pressed(guidesFor(j, Double3(0.3, 0, -0.2)), GuideColor::AxisX) == 0);
}

TEST_CASE("Joint status says why a joint constrains nothing", "[sim][joint-display]")
{
    using Status = RigidBodySimulation::JointStatus;
    std::vector<RigidBodySimulation::BodyDefinition> bodies(4);
    bodies[0].physicsMode = RigidBodySimulation::PhysicsMode::eFollowBone;
    bodies[1].physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    bodies[2].enabled = false;
    bodies[3].physicsMode = RigidBodySimulation::PhysicsMode::eFollowBone;
    const auto status = [&](long a, long b, long type = 0)
    {
        RigidBodySimulation::JointDefinition j;
        j.bodyA = a;
        j.bodyB = b;
        j.type = type;
        return RigidBodySimulation::jointStatus(j, bodies);
    };
    REQUIRE(status(0, 1) == Status::eActive);
    REQUIRE(status(1, 3) == Status::eActive);
    REQUIRE(status(-1, 1) == Status::eMissingBody);
    REQUIRE(status(0, 4) == Status::eMissingBody);
    REQUIRE(status(1, 1) == Status::eSameBody);
    REQUIRE(status(1, 2) == Status::eDisabledBody);
    REQUIRE(status(0, 1, 3) == Status::eUnsupportedType);
    REQUIRE(status(0, 3) == Status::eBothKinematic);
}

TEST_CASE("Attachment frames follow body rotation about an offset pivot", "[sim][attachment]")
{
    auto def = weldDefinition();
    def.bodies[0].restPos = Double3(2, 0, 0);
    def.bodies[0].restRot = Double3(0, 0, 90);
    def.joints[0].frameT = Double3(2, 1, 0);
    def.joints[0].frameR = Double3(0, 0, 90);
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));
    RigidBodySimulation::Pose a, b;
    const auto& joint = def.joints[0];
    REQUIRE(sim.attachmentPose(0, joint.frameT, joint.frameR, a));
    REQUIRE(sim.attachmentPose(1, joint.frameT, joint.frameR, b));
    requireDouble3(a.pos, joint.frameT, 1e-5);
    requireDouble3(b.pos, joint.frameT, 1e-5);

    RigidBodySimulation::Pose moved;
    moved.pos = Double3(4, 3, 0);
    moved.quat = eulerDegreesToQuat(0, 0, 180);
    REQUIRE(sim.setKinematicPose(0, moved));
    REQUIRE(sim.attachmentPose(0, joint.frameT, joint.frameR, a));
    requireDouble3(a.pos, Double3(3, 3, 0), 1e-5);
    REQUIRE(std::fabs(a.quat.z) == Catch::Approx(1.0).margin(1e-5));
    REQUIRE(sim.attachmentPose(1, joint.frameT, joint.frameR, b));
    // The read-only query does not step the other body toward the moved anchor.
    requireDouble3(b.pos, joint.frameT, 1e-5);
    REQUIRE_FALSE(sim.attachmentPose(99, joint.frameT, joint.frameR, a));
    sim.clear();
    REQUIRE_FALSE(sim.attachmentPose(0, joint.frameT, joint.frameR, a));
}

TEST_CASE("RigidBodySimulation initializes and returns rest pose", "[sim]")
{
    RigidBodySimulation sim;
    REQUIRE_FALSE(sim.initialized());

    REQUIRE(sim.initialize(gravityDefinition(-9.8)));
    REQUIRE(sim.initialized());

    // Before stepping, the solved pose is the rest pose.
    RigidBodySimulation::Pose p0 = sim.bodyPose(0);
    REQUIRE(p0.pos.x == Catch::Approx(0.0));
    REQUIRE(p0.pos.y == Catch::Approx(0.0));
    REQUIRE(p0.pos.z == Catch::Approx(0.0));
}

TEST_CASE("Gravity pulls a dynamic body down", "[sim]")
{
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(gravityDefinition(-9.8)));

    // 1 second of RigidBodySimulation (60 ticks) at -9.8 -> y ~ -4.9.
    for (int i = 0; i < 60; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    RigidBodySimulation::Pose p = sim.bodyPose(0);
    REQUIRE(p.pos.y < -1.0);
}

TEST_CASE("Kinematic anchor drives a welded dynamic body", "[sim]")
{
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    // Setting the anchor to its current (rest) pose reports no movement.
    RigidBodySimulation::Pose rest;
    rest.pos.y = 0.0;
    REQUIRE_FALSE(sim.setKinematicPose(0, rest));

    // Move the anchor up by 1 unit -> the welded dynamic body must follow.
    RigidBodySimulation::Pose moved;
    moved.pos.y = 1.0;
    REQUIRE(sim.setKinematicPose(0, moved));

    for (int i = 0; i < 60; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    RigidBodySimulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y > 1.2); // rest offset +1 kept, so it tracks ~y=2
}

TEST_CASE("resetDynamicBodies places bodies at the current skeleton pose", "[sim]")
{
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    RigidBodySimulation::Pose moved;
    moved.pos[1] = 1.0;
    (void) sim.setKinematicPose(0, moved);
    for (int i = 0; i < 30; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    // Reset teleports the dynamic body to anchorCurrent * (anchorRest^-1 * bodyRest)
    // = (0,1,0) * (0,1,0) = (0,2,0) exactly, zeroing velocities.
    sim.resetDynamicBodies();
    RigidBodySimulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y == Catch::Approx(2.0).margin(1e-3));
}

TEST_CASE("reset captures an anchor registered later in body order", "[sim]")
{
    // A dynamic body whose reset-anchor kinematic body appears LATER in body
    // order (Endmin's skirt — anchored to a bone whose kinematic body follows
    // it).  The reset offset must still be captured so a rewind rebuild pins
    // the body to the CURRENT skeleton pose; previously the in-loop capture
    // saw an mAnchorRest too small to hold the anchor and silently skipped
    // the body, which sat at its fresh-world rest pose on every rewind.
    RigidBodySimulation::Definition def;
    def.gravity = Double3(0.0, 0.0, 0.0);

    // Body 0: dynamic, anchored to kinematic-order anchor 0 (body 1 below).
    RigidBodySimulation::BodyDefinition dynamic;
    dynamic.colliderType = RigidBodySimulation::ColliderType::eSphere;
    dynamic.radius = 0.5;
    dynamic.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    dynamic.restPos = Double3(0.0, 1.0, 0.0);
    dynamic.resetAnchorIndex = 0; // kinematic order 0 = body index 1
    def.bodies.push_back(dynamic);

    // Body 1: the kinematic anchor, registered AFTER the dynamic body.
    RigidBodySimulation::BodyDefinition anchor;
    anchor.colliderType = RigidBodySimulation::ColliderType::eSphere;
    anchor.radius = 0.5;
    anchor.physicsMode = RigidBodySimulation::PhysicsMode::eFollowBone;
    def.bodies.push_back(anchor);

    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));

    // Raw reset with the anchor moved from (0,0,0) to (0,1,0): the dynamic
    // body must be pinned to anchorCurrent * (anchorRest^-1 * bodyRest)
    // = (0,1,0) * (0,1,0) = (0,2,0).
    std::vector<RigidBodySimulation::Pose> rawRest(1);
    rawRest[0].pos = Double3(0.0, 0.0, 0.0);
    std::vector<RigidBodySimulation::Pose> rawCurrent(1);
    rawCurrent[0].pos = Double3(0.0, 1.0, 0.0);
    sim.resetDynamicBodies(rawRest, rawCurrent);

    RigidBodySimulation::Pose p = sim.bodyPose(0);
    REQUIRE(p.pos.y == Catch::Approx(2.0).margin(1e-3));
}

TEST_CASE("Disabled bodies are skipped and bodyPose returns rest", "[sim]")
{
    RigidBodySimulation::Definition def = weldDefinition();
    def.bodies[1].enabled = false; // dynamic body disabled

    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));

    // The disabled body is not simulated (still rest pose), and the joint
    // referencing it is skipped without error.
    RigidBodySimulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y == Catch::Approx(1.0));
    sim.step(RigidBodySimulation::kFixedDt); // must not crash
}

TEST_CASE("clear() tears the world down and resets state", "[sim]")
{
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(gravityDefinition(-9.8)));
    sim.clear();
    REQUIRE_FALSE(sim.initialized());

    // bodyPose falls back to the default rest pose (no bodies).
    RigidBodySimulation::Pose p = sim.bodyPose(0);
    REQUIRE(p.pos.y == Catch::Approx(0.0));
}

TEST_CASE("rideDynamicBodiesAlong moves dynamic bodies by a rigid world move without stepping",
          "[sim]")
{
    // weldDefinition: kinematic anchor at origin + dynamic body at y=1, welded.
    // A whole-skeleton drag at a paused frame must ride the dynamic chain
    // along by the same rigid move — no physics step, no velocity impulse.
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    // Anchor teleported to y=2 (the kinematic body follows its bone).
    RigidBodySimulation::Pose anchor;
    anchor.pos = Double3(0.0, 2.0, 0.0);
    REQUIRE(sim.setKinematicPose(0, anchor));

    // The dynamic body is still at rest (y=1) until a step happens.
    RigidBodySimulation::Pose before = sim.bodyPose(1);
    REQUIRE(before.pos.y == Catch::Approx(1.0).margin(1e-3));

    // Whole-skeleton ride-along: translate everything by +3 on Y.
    RigidBodySimulation::Pose move;
    move.pos = Double3(0.0, 3.0, 0.0);
    sim.rideDynamicBodiesAlong(move);

    // The dynamic body rode along to y=4 (1 + 3), WITHOUT a physics step.
    RigidBodySimulation::Pose after = sim.bodyPose(1);
    REQUIRE(after.pos.y == Catch::Approx(4.0).margin(1e-3));
    REQUIRE(after.pos.x == Catch::Approx(0.0).margin(1e-3));
    REQUIRE(after.pos.z == Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("rideDynamicBodiesAlong also carries rotation", "[sim]")
{
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    // 90-degree rotation about Z: the welded body at (0,1,0) lands at
    // (-1,0,0) (right-handed rotation: +Z turns +Y toward -X).
    RigidBodySimulation::Pose move;
    move.quat = eulerDegreesToQuat(0.0, 0.0, 90.0);
    sim.rideDynamicBodiesAlong(move);

    RigidBodySimulation::Pose after = sim.bodyPose(1);
    REQUIRE(after.pos.x == Catch::Approx(-1.0).margin(1e-2));
    REQUIRE(after.pos.y == Catch::Approx(0.0).margin(1e-2));
}

TEST_CASE("rideDynamicBodiesAlong leaves velocities at zero for a clean resume", "[sim]")
{
    // A coherent whole-skeleton move: anchor 0 -> 3 AND body 1 -> 4 by the
    // SAME +3 translate (weld keeps body 1 unit above the anchor).  After the
    // ride-along, subsequent REAL steps must not inherit a teleport velocity:
    // the body should stay at 4, not fly off or sag because its anchor jumped.
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    RigidBodySimulation::Pose anchor;
    anchor.pos = Double3(0.0, 3.0, 0.0);
    (void) sim.setKinematicPose(0, anchor);

    RigidBodySimulation::Pose move;
    move.pos = Double3(0.0, 3.0, 0.0);
    sim.rideDynamicBodiesAlong(move);

    for (int i = 0; i < 5; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }
    RigidBodySimulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y == Catch::Approx(4.0).margin(0.25));
}

// A static (mass 0) ground sphere at the origin + a dynamic ball dropped from
// above.  The ground is in group 0 (bit 0), the ball in group 1 (bit 1); the
// masks decide whether the pair is allowed to collide.
RigidBodySimulation::Definition ballDropDefinition(long groundMask, long ballMask)
{
    RigidBodySimulation::Definition def;
    def.gravity = Double3(0.0, -9.8, 0.0);

    RigidBodySimulation::BodyDefinition ground;
    ground.mass = 0.0; // static
    ground.colliderType = RigidBodySimulation::ColliderType::eSphere;
    ground.radius = 1.0;
    ground.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    ground.groupId = 0; // group bit 0
    ground.mask = groundMask;
    def.bodies.push_back(ground);

    RigidBodySimulation::BodyDefinition ball;
    ball.mass = 1.0;
    ball.colliderType = RigidBodySimulation::ColliderType::eSphere;
    ball.radius = 1.0;
    ball.restPos = Double3(0.0, 5.0, 0.0);
    ball.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    ball.groupId = 1; // group bit 1
    ball.mask = ballMask;
    def.bodies.push_back(ball);

    return def;
}

TEST_CASE("Collision filtering allows bodies whose filter masks overlap", "[sim]")
{
    // Both masks match everything -> the ball lands on the ground sphere.
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(ballDropDefinition(0xFFFF, 0xFFFF)));
    for (int i = 0; i < 120; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    // Resting ball center = sum of radii = 2.0 (definitely not fallen through).
    RigidBodySimulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y > 1.5);
    REQUIRE(p.pos.y < 2.5);
}

TEST_CASE("Collision filtering lets disjoint-mask bodies pass through", "[sim]")
{
    // The ball's mask excludes the ground's group bit -> no collision pair is
    // created, so the ball falls straight through the ground sphere.
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(ballDropDefinition(0xFFFF, 0xFFFE)));
    for (int i = 0; i < 120; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    RigidBodySimulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y < -5.0); // fell through
}

TEST_CASE("Kinematic anchor rotation is detected and drives a welded body", "[sim]")
{
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    // Identity pose == stored rest -> no movement.
    RigidBodySimulation::Pose identity;
    REQUIRE_FALSE(sim.setKinematicPose(0, identity));

    // A pure rotation must also be reported as movement (column-dot check).
    RigidBodySimulation::Pose rotated;
    {
        const Double4 q = eulerDegreesToQuat(30.0, 0.0, 0.0);
        rotated.quat = q;
    }
    REQUIRE(sim.setKinematicPose(0, rotated));

    // Repeating the same pose reports no movement.
    REQUIRE_FALSE(sim.setKinematicPose(0, rotated));

    for (int i = 0; i < 60; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    // The rigid weld carries the anchor's 30 deg X rotation onto the body.
    RigidBodySimulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.quat.x == Catch::Approx(std::sin(deg2rad(15.0))).margin(1e-3));
    REQUIRE(p.quat.w == Catch::Approx(std::cos(deg2rad(15.0))).margin(1e-3));
    REQUIRE(std::fabs(p.quat.y) < 1e-3);
    REQUIRE(std::fabs(p.quat.z) < 1e-3);
}

TEST_CASE("Disabled kinematic bodies are excluded from the anchor order", "[sim]")
{
    RigidBodySimulation::Definition def;
    def.gravity = Double3(0.0, 0.0, 0.0);

    // Body 0: plain dynamic body (mask 0 -> no interactions).
    RigidBodySimulation::BodyDefinition freeBody;
    freeBody.colliderType = RigidBodySimulation::ColliderType::eSphere;
    freeBody.radius = 0.5;
    freeBody.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    def.bodies.push_back(freeBody);

    // Body 1: kinematic but DISABLED â€” must NOT become anchor index 0.
    RigidBodySimulation::BodyDefinition disabledAnchor;
    disabledAnchor.colliderType = RigidBodySimulation::ColliderType::eSphere;
    disabledAnchor.radius = 0.5;
    disabledAnchor.physicsMode = RigidBodySimulation::PhysicsMode::eFollowBone;
    disabledAnchor.enabled = false;
    def.bodies.push_back(disabledAnchor);

    // Body 2: enabled kinematic anchor -> anchor index 0.
    RigidBodySimulation::BodyDefinition anchor;
    anchor.colliderType = RigidBodySimulation::ColliderType::eSphere;
    anchor.radius = 0.5;
    anchor.physicsMode = RigidBodySimulation::PhysicsMode::eFollowBone;
    def.bodies.push_back(anchor);

    // Body 3: dynamic welded to body 2, reset anchor = anchor index 0.
    RigidBodySimulation::BodyDefinition welded;
    welded.colliderType = RigidBodySimulation::ColliderType::eSphere;
    welded.radius = 0.5;
    welded.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    welded.restPos = Double3(0.0, 1.0, 0.0);
    welded.resetAnchorIndex = 0;
    def.bodies.push_back(welded);

    RigidBodySimulation::JointDefinition weld;
    weld.bodyA = 2;
    weld.bodyB = 3;
    weld.type = 0; // SPRING_6DOF
    weld.frameT = Double3(0.0, 0.5, 0.0);
    def.joints.push_back(weld);

    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));

    // Exactly ONE anchor exists (body 2) â€” index 1 must be out of range.
    RigidBodySimulation::Pose moved;
    moved.pos.y = 1.0;
    REQUIRE(sim.setKinematicPose(0, moved));
    REQUIRE_FALSE(sim.setKinematicPose(1, moved));

    for (int i = 0; i < 60; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    // The welded body follows body 2 (the enabled anchor), not the disabled one.
    RigidBodySimulation::Pose p = sim.bodyPose(3);
    REQUIRE(p.pos.y > 1.2); // rest offset +1 kept -> tracks ~y=2
}

TEST_CASE("Box and capsule bodies simulate under gravity", "[sim]")
{
    RigidBodySimulation::Definition def;
    def.gravity = Double3(0.0, -9.8, 0.0);

    RigidBodySimulation::BodyDefinition box;
    box.mass = 1.0;
    box.colliderType = RigidBodySimulation::ColliderType::eBox;
    box.extents = Double3(0.5, 0.5, 0.5);
    box.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    def.bodies.push_back(box);

    RigidBodySimulation::BodyDefinition capsule;
    capsule.mass = 1.0;
    capsule.colliderType = RigidBodySimulation::ColliderType::eCapsule;
    capsule.radius = 0.5;
    capsule.length = 1.0;
    capsule.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
    def.bodies.push_back(capsule);

    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));

    for (int i = 0; i < 60; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    // Both shapes get valid inertia and fall freely (mask 0 -> no inter-body
    // collision, so they do not push each other around).
    RigidBodySimulation::Pose pb = sim.bodyPose(0);
    RigidBodySimulation::Pose pc = sim.bodyPose(1);
    REQUIRE(pb.pos.y < -1.0);
    REQUIRE(pc.pos.y < -1.0);
}

namespace
{
// The weld fixture under gravity, with the dynamic body hanging from the
// kinematic anchor on a joint configured by `configure`.  Returns the body's
// Y displacement from rest after one simulated second.
template <typename Configure> double sagUnderGravity(Configure configure)
{
    auto def = weldDefinition();
    def.gravity = Double3(0.0, -9.8, 0.0);
    configure(def.joints[0]);
    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));
    for (int i = 0; i < 60; ++i)
        sim.step(RigidBodySimulation::kFixedDt);
    return sim.bodyPose(1).pos.y - 1.0;
}
} // namespace

// These pin the Bullet behaviour the joint display model relies on
// (joint_display_ranges.hpp): equal endpoints lock, tiny ranges move,
// inverted endpoints are free, and stiffness on a locked axis does nothing.
TEST_CASE("Spring6Dof freedom matches the joint display model", "[sim][joint-display]")
{
    const double locked = sagUnderGravity([](auto&) {});
    REQUIRE(std::abs(locked) < 1e-3);

    SECTION("stiffness on a locked axis has no effect")
    {
        const double sprungLocked =
            sagUnderGravity([](auto& j) { j.linearStiffness = Double3(0.0, 50.0, 0.0); });
        REQUIRE(sprungLocked == Catch::Approx(locked).margin(1e-4));
    }
    SECTION("a tiny linear range is real freedom")
    {
        const double tiny = sagUnderGravity(
            [](auto& j)
            {
                j.linearMin.y = -0.008;
                j.linearMax.y = 0.008;
            });
        REQUIRE(tiny < -0.005);
        REQUIRE(tiny > -0.02);
    }
    SECTION("inverted linear endpoints are free")
    {
        const double free = sagUnderGravity(
            [](auto& j)
            {
                j.linearMin.y = 1.0;
                j.linearMax.y = -1.0;
            });
        REQUIRE(free < -1.0);
    }
    SECTION("a spring acts on a movable axis")
    {
        const auto range = [](auto& j)
        {
            j.linearMin.y = -2.0;
            j.linearMax.y = 2.0;
        };
        const double unsprung = sagUnderGravity(range);
        const double sprung = sagUnderGravity(
            [&](auto& j)
            {
                range(j);
                j.linearStiffness = Double3(0.0, 50.0, 0.0);
            });
        REQUIRE(unsprung < -1.5); // falls to the stop
        REQUIRE(sprung > unsprung + 0.5);
    }
}

TEST_CASE("PMX 2.1 joint types are not simulated", "[sim]")
{
    // Only Spring6Dof (PMX 2.0) builds a constraint: with any other type the
    // dynamic body is NOT carried by the moving anchor.
    for (long type : {1L, 2L, 3L, 4L, 5L})
    {
        RigidBodySimulation::Definition def = weldDefinition();
        def.joints[0].type = type;

        RigidBodySimulation sim;
        REQUIRE(sim.initialize(def));

        // Move the anchor sideways, clear of the dynamic body (no contact).
        RigidBodySimulation::Pose moved;
        moved.pos.x = 5.0;
        REQUIRE(sim.setKinematicPose(0, moved));
        for (int i = 0; i < 60; ++i)
            sim.step(RigidBodySimulation::kFixedDt);

        REQUIRE(sim.bodyPose(1).pos.x == Catch::Approx(0.0).margin(1e-6));
    }
}

TEST_CASE("initialize returns false for an empty body list", "[sim]")
{
    RigidBodySimulation sim;
    RigidBodySimulation::Definition def;
    REQUIRE_FALSE(sim.initialize(def));
    REQUIRE_FALSE(sim.initialized());
}

TEST_CASE("Spring6Dof with finite limits builds and steps without error", "[sim]")
{
    {
        RigidBodySimulation::Definition def;
        def.gravity = Double3(0.0, 0.0, 0.0);

        RigidBodySimulation::BodyDefinition anchor;
        anchor.colliderType = RigidBodySimulation::ColliderType::eSphere;
        anchor.radius = 0.5;
        anchor.physicsMode = RigidBodySimulation::PhysicsMode::eFollowBone;
        def.bodies.push_back(anchor);

        RigidBodySimulation::BodyDefinition dynamic;
        dynamic.colliderType = RigidBodySimulation::ColliderType::eSphere;
        dynamic.radius = 0.5;
        dynamic.physicsMode = RigidBodySimulation::PhysicsMode::ePhysics;
        dynamic.restPos = Double3(0.0, 1.0, 0.0);
        def.bodies.push_back(dynamic);

        RigidBodySimulation::JointDefinition joint;
        joint.bodyA = 0;
        joint.bodyB = 1;
        joint.type = 0;
        joint.frameT = Double3(0.0, 0.5, 0.0);
        joint.linearMin = Double3(-1.0, -1.0, -1.0);
        joint.linearMax = Double3(1.0, 1.0, 1.0);
        joint.angularMin = Double3(-0.5, -0.5, -0.5);
        joint.angularMax = Double3(0.5, 0.5, 0.5);
        def.joints.push_back(joint);

        RigidBodySimulation sim;
        REQUIRE(sim.initialize(def));

        for (int i = 0; i < 60; ++i)
        {
            sim.step(RigidBodySimulation::kFixedDt);
        }

        // No crash, and the solved pose stays finite.
        RigidBodySimulation::Pose p = sim.bodyPose(1);
        REQUIRE(std::isfinite(p.pos.x));
        REQUIRE(std::isfinite(p.pos.y));
        REQUIRE(std::isfinite(p.pos.z));
        REQUIRE(std::isfinite(p.quat.x));
        REQUIRE(std::isfinite(p.quat.y));
        REQUIRE(std::isfinite(p.quat.z));
        REQUIRE(std::isfinite(p.quat.w));
    }
}

// â”€â”€ PMX shape_size mapping (applyShapeSize) â”€â”€
// These lock in the full-size-vs-half-extents convention: the PMX shape_size is
// a FULL size (box extents are full), while the engine stores box extents as
// HALF-extents because btBoxShape expects half extents.

TEST_CASE("applyShapeSize maps PMX shape_size onto the engine fields", "[sim][shape-size]")
{
    // Sphere: shape_size[0] is the radius (shape_size[1]/[2] are unused).
    {
        RigidBodySimulation::BodyDefinition b;
        b.colliderType = RigidBodySimulation::ColliderType::eSphere;
        applyShapeSize(b, Double3(0.5, 0.5, 0.0));
        REQUIRE(b.radius == Catch::Approx(0.5));
        // extents/length are not touched by the sphere branch.
        REQUIRE(b.extents.x == Catch::Approx(1.0));
        REQUIRE(b.length == Catch::Approx(1.0));
    }

    // Box: PMX shape_size IS the half-extent (MMD feeds it verbatim to
    // btBoxShape) — the engine stores it VERBATIM, full span = 2 × shape_size.
    {
        RigidBodySimulation::BodyDefinition b;
        b.colliderType = RigidBodySimulation::ColliderType::eBox;
        applyShapeSize(b, Double3(0.15, 0.5, 1.0));
        requireDouble3(b.extents, Double3(0.15, 0.5, 1.0));
    }

    // Capsule: shape_size[0] = radius, shape_size[1] = cylinder length.
    {
        RigidBodySimulation::BodyDefinition b;
        b.colliderType = RigidBodySimulation::ColliderType::eCapsule;
        applyShapeSize(b, Double3(0.2, 0.4, 0.0));
        REQUIRE(b.radius == Catch::Approx(0.2));
        REQUIRE(b.length == Catch::Approx(0.4));
    }
}
