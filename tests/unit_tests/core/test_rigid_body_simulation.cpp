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

#include "physics_math.hpp"
#include "rigid_body_simulation.hpp"

#include <cmath>

using mmd::core::applyShapeSize;
using mmd::core::Double3;
using mmd::core::Double4;
using mmd::core::RigidBodySimulation;
using namespace mmd::core::physics_math;

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

TEST_CASE("rideDynamicBodiesAlong moves dynamic bodies by a rigid world move without stepping", "[sim]")
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

TEST_CASE("Point-to-point joint keeps the body at the anchor pivot", "[sim]")
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
    dynamic.restPos = Double3(1.0, 1.0, 0.0);
    def.bodies.push_back(dynamic);

    RigidBodySimulation::JointDefinition p2p;
    p2p.bodyA = 0;
    p2p.bodyB = 1;
    p2p.type = 2; // P2P
    // Joint frame at the body's rest position: the pivot coincides with the
    // body's COM, so the constraint force passes through it (no torque -> the
    // body translates without spinning, keeping the test deterministic).
    p2p.frameT = Double3(1.0, 1.0, 0.0);
    def.joints.push_back(p2p);

    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));

    // Anchor up by 2 -> the body keeps its local pivot offset and lands at
    // (1, 3, 0).
    RigidBodySimulation::Pose moved;
    moved.pos.y = 2.0;
    REQUIRE(sim.setKinematicPose(0, moved));

    for (int i = 0; i < 60; ++i)
    {
        sim.step(RigidBodySimulation::kFixedDt);
    }

    RigidBodySimulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.x == Catch::Approx(1.0).margin(0.05));
    REQUIRE(p.pos.y == Catch::Approx(3.0).margin(0.05));
    REQUIRE(p.pos.z == Catch::Approx(0.0).margin(0.05));
}

TEST_CASE("initialize returns false for an empty body list", "[sim]")
{
    RigidBodySimulation sim;
    RigidBodySimulation::Definition def;
    REQUIRE_FALSE(sim.initialize(def));
    REQUIRE_FALSE(sim.initialized());
}

TEST_CASE("6DOF with zero limits behaves like a rigid weld", "[sim]")
{
    // SIX_DOF (type 1) with zero springs + zero limits locks exactly like the
    // SPRING_6DOF weld â€” the anchor must carry the dynamic body.
    RigidBodySimulation::Definition def = weldDefinition();
    def.joints[0].type = 1; // SIX_DOF

    RigidBodySimulation sim;
    REQUIRE(sim.initialize(def));

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

TEST_CASE("Every joint type builds and steps without error", "[sim]")
{
    for (long type : {0L, 1L, 2L, 3L, 4L, 5L})
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
        joint.type = type;
        joint.frameT = Double3(0.0, 0.5, 0.0);
        // Finite limits so no constraint type degenerates.
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

        // No crash, and the solved pose stays finite for every joint type.
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

    // Box: PMX shape_size is FULL size â€” the engine stores HALF extents.
    {
        RigidBodySimulation::BodyDefinition b;
        b.colliderType = RigidBodySimulation::ColliderType::eBox;
        applyShapeSize(b, Double3(0.15, 0.5, 1.0));
        requireDouble3(b.extents, Double3(0.075, 0.25, 0.5));
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
