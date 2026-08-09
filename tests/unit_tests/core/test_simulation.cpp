/*
 * SPDX-License-Identifier: MIT
 *
 * test_simulation.cpp
 *
 * Unit tests for the Maya-free physics engine (Simulation).  This target
 * links ONLY Bullet + Catch2 (no Maya SDK) — compiling and passing here is
 * what proves the engine is truly Maya-free.
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "physics_math.hpp"
#include "simulation.hpp"

#include <cmath>

using mmd::core::Double3;
using mmd::core::Simulation;
using namespace mmd::core::physics_math;

namespace
{

// A single dynamic sphere at the origin with gravity.
Simulation::Definition gravityDefinition(double gy)
{
    Simulation::Definition def;
    def.gravity = Double3(0.0, gy, 0.0);

    Simulation::BodyDefinition bd;
    bd.mass = 1.0;
    bd.colliderType = Simulation::ColliderType::eSphere;
    bd.radius = 0.5;
    bd.physicsMode = Simulation::PhysicsMode::ePhysics;
    def.bodies.push_back(bd);
    return def;
}

// A kinematic anchor at the origin + a dynamic body 1 unit above it, rigidly
// welded (SPRING_6DOF with zero springs + zero limits = locked).
Simulation::Definition weldDefinition()
{
    Simulation::Definition def;
    def.gravity[0] = 0.0;
    def.gravity[1] = 0.0;
    def.gravity[2] = 0.0;

    Simulation::BodyDefinition anchor;
    anchor.colliderType = Simulation::ColliderType::eSphere;
    anchor.radius = 0.5;
    anchor.physicsMode = Simulation::PhysicsMode::eFollowBone;
    def.bodies.push_back(anchor);

    Simulation::BodyDefinition dynamic;
    dynamic.colliderType = Simulation::ColliderType::eSphere;
    dynamic.radius = 0.5;
    dynamic.physicsMode = Simulation::PhysicsMode::ePhysics;
    dynamic.restPos = Double3(0.0, 1.0, 0.0);
    dynamic.resetAnchorIndex = 0; // anchored to the kinematic body
    def.bodies.push_back(dynamic);

    Simulation::JointDefinition weld;
    weld.bodyA = 0;
    weld.bodyB = 1;
    weld.type = 0; // SPRING_6DOF
    weld.frameT = Double3(0.0, 0.5, 0.0);
    def.joints.push_back(weld);
    return def;
}

} // namespace

TEST_CASE("Simulation initializes and returns rest pose", "[sim]")
{
    Simulation sim;
    REQUIRE_FALSE(sim.initialized());

    REQUIRE(sim.initialize(gravityDefinition(-9.8)));
    REQUIRE(sim.initialized());

    // Before stepping, the solved pose is the rest pose.
    Simulation::Pose p0 = sim.bodyPose(0);
    REQUIRE(p0.pos.x == Catch::Approx(0.0));
    REQUIRE(p0.pos.y == Catch::Approx(0.0));
    REQUIRE(p0.pos.z == Catch::Approx(0.0));
}

TEST_CASE("Gravity pulls a dynamic body down", "[sim]")
{
    Simulation sim;
    REQUIRE(sim.initialize(gravityDefinition(-9.8)));

    // 1 second of simulation (60 ticks) at -9.8 -> y ~ -4.9.
    for (int i = 0; i < 60; ++i)
        sim.step(Simulation::kFixedDt);

    Simulation::Pose p = sim.bodyPose(0);
    REQUIRE(p.pos.y < -1.0);
}

TEST_CASE("Kinematic anchor drives a welded dynamic body", "[sim]")
{
    Simulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    // Setting the anchor to its current (rest) pose reports no movement.
    Simulation::Pose rest;
    rest.pos.y = 0.0;
    REQUIRE_FALSE(sim.setKinematicPose(0, rest));

    // Move the anchor up by 1 unit -> the welded dynamic body must follow.
    Simulation::Pose moved;
    moved.pos.y = 1.0;
    REQUIRE(sim.setKinematicPose(0, moved));

    for (int i = 0; i < 60; ++i)
        sim.step(Simulation::kFixedDt);

    Simulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y > 1.2); // rest offset +1 kept, so it tracks ~y=2
}

TEST_CASE("resetDynamicBodies places bodies at the current skeleton pose", "[sim]")
{
    Simulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    Simulation::Pose moved;
    moved.pos[1] = 1.0;
    sim.setKinematicPose(0, moved);
    for (int i = 0; i < 30; ++i)
        sim.step(Simulation::kFixedDt);

    // Reset teleports the dynamic body to anchorCurrent * (anchorRest^-1 * bodyRest)
    // = (0,1,0) * (0,1,0) = (0,2,0) exactly, zeroing velocities.
    sim.resetDynamicBodies();
    Simulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y == Catch::Approx(2.0).margin(1e-3));
}

TEST_CASE("Disabled bodies are skipped and bodyPose returns rest", "[sim]")
{
    Simulation::Definition def = weldDefinition();
    def.bodies[1].enabled = false; // dynamic body disabled

    Simulation sim;
    REQUIRE(sim.initialize(def));

    // The disabled body is not simulated (still rest pose), and the joint
    // referencing it is skipped without error.
    Simulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y == Catch::Approx(1.0));
    sim.step(Simulation::kFixedDt); // must not crash
}

TEST_CASE("clear() tears the world down and resets state", "[sim]")
{
    Simulation sim;
    REQUIRE(sim.initialize(gravityDefinition(-9.8)));
    sim.clear();
    REQUIRE_FALSE(sim.initialized());

    // bodyPose falls back to the default rest pose (no bodies).
    Simulation::Pose p = sim.bodyPose(0);
    REQUIRE(p.pos.y == Catch::Approx(0.0));
}

// A static (mass 0) ground sphere at the origin + a dynamic ball dropped from
// above.  The ground is in group 0 (bit 0), the ball in group 1 (bit 1); the
// masks decide whether the pair is allowed to collide.
Simulation::Definition ballDropDefinition(long groundMask, long ballMask)
{
    Simulation::Definition def;
    def.gravity = Double3(0.0, -9.8, 0.0);

    Simulation::BodyDefinition ground;
    ground.mass = 0.0; // static
    ground.colliderType = Simulation::ColliderType::eSphere;
    ground.radius = 1.0;
    ground.physicsMode = Simulation::PhysicsMode::ePhysics;
    ground.groupId = 0; // group bit 0
    ground.mask = groundMask;
    def.bodies.push_back(ground);

    Simulation::BodyDefinition ball;
    ball.mass = 1.0;
    ball.colliderType = Simulation::ColliderType::eSphere;
    ball.radius = 1.0;
    ball.restPos = Double3(0.0, 5.0, 0.0);
    ball.physicsMode = Simulation::PhysicsMode::ePhysics;
    ball.groupId = 1; // group bit 1
    ball.mask = ballMask;
    def.bodies.push_back(ball);

    return def;
}

TEST_CASE("Collision filtering allows bodies whose filter masks overlap", "[sim]")
{
    // Both masks match everything -> the ball lands on the ground sphere.
    Simulation sim;
    REQUIRE(sim.initialize(ballDropDefinition(0xFFFF, 0xFFFF)));
    for (int i = 0; i < 120; ++i)
        sim.step(Simulation::kFixedDt);

    // Resting ball center = sum of radii = 2.0 (definitely not fallen through).
    Simulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y > 1.5);
    REQUIRE(p.pos.y < 2.5);
}

TEST_CASE("Collision filtering lets disjoint-mask bodies pass through", "[sim]")
{
    // The ball's mask excludes the ground's group bit -> no collision pair is
    // created, so the ball falls straight through the ground sphere.
    Simulation sim;
    REQUIRE(sim.initialize(ballDropDefinition(0xFFFF, 0xFFFE)));
    for (int i = 0; i < 120; ++i)
        sim.step(Simulation::kFixedDt);

    Simulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.y < -5.0); // fell through
}

TEST_CASE("Kinematic anchor rotation is detected and drives a welded body", "[sim]")
{
    Simulation sim;
    REQUIRE(sim.initialize(weldDefinition()));

    // Identity pose == stored rest -> no movement.
    Simulation::Pose identity;
    REQUIRE_FALSE(sim.setKinematicPose(0, identity));

    // A pure rotation must also be reported as movement (column-dot check).
    Simulation::Pose rotated;
    {
        const btQuaternion q = eulerDegreesToQuat(30.0, 0.0, 0.0);
        rotated.quat.x = q.x();
        rotated.quat.y = q.y();
        rotated.quat.z = q.z();
        rotated.quat.w = q.w();
    }
    REQUIRE(sim.setKinematicPose(0, rotated));

    // Repeating the same pose reports no movement.
    REQUIRE_FALSE(sim.setKinematicPose(0, rotated));

    for (int i = 0; i < 60; ++i)
        sim.step(Simulation::kFixedDt);

    // The rigid weld carries the anchor's 30 deg X rotation onto the body.
    Simulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.quat.x == Catch::Approx(std::sin(deg2rad(15.0))).margin(1e-3));
    REQUIRE(p.quat.w == Catch::Approx(std::cos(deg2rad(15.0))).margin(1e-3));
    REQUIRE(std::fabs(p.quat.y) < 1e-3);
    REQUIRE(std::fabs(p.quat.z) < 1e-3);
}

TEST_CASE("Disabled kinematic bodies are excluded from the anchor order", "[sim]")
{
    Simulation::Definition def;
    def.gravity = Double3(0.0, 0.0, 0.0);

    // Body 0: plain dynamic body (mask 0 -> no interactions).
    Simulation::BodyDefinition freeBody;
    freeBody.colliderType = Simulation::ColliderType::eSphere;
    freeBody.radius = 0.5;
    freeBody.physicsMode = Simulation::PhysicsMode::ePhysics;
    def.bodies.push_back(freeBody);

    // Body 1: kinematic but DISABLED — must NOT become anchor index 0.
    Simulation::BodyDefinition disabledAnchor;
    disabledAnchor.colliderType = Simulation::ColliderType::eSphere;
    disabledAnchor.radius = 0.5;
    disabledAnchor.physicsMode = Simulation::PhysicsMode::eFollowBone;
    disabledAnchor.enabled = false;
    def.bodies.push_back(disabledAnchor);

    // Body 2: enabled kinematic anchor -> anchor index 0.
    Simulation::BodyDefinition anchor;
    anchor.colliderType = Simulation::ColliderType::eSphere;
    anchor.radius = 0.5;
    anchor.physicsMode = Simulation::PhysicsMode::eFollowBone;
    def.bodies.push_back(anchor);

    // Body 3: dynamic welded to body 2, reset anchor = anchor index 0.
    Simulation::BodyDefinition welded;
    welded.colliderType = Simulation::ColliderType::eSphere;
    welded.radius = 0.5;
    welded.physicsMode = Simulation::PhysicsMode::ePhysics;
    welded.restPos = Double3(0.0, 1.0, 0.0);
    welded.resetAnchorIndex = 0;
    def.bodies.push_back(welded);

    Simulation::JointDefinition weld;
    weld.bodyA = 2;
    weld.bodyB = 3;
    weld.type = 0; // SPRING_6DOF
    weld.frameT = Double3(0.0, 0.5, 0.0);
    def.joints.push_back(weld);

    Simulation sim;
    REQUIRE(sim.initialize(def));

    // Exactly ONE anchor exists (body 2) — index 1 must be out of range.
    Simulation::Pose moved;
    moved.pos.y = 1.0;
    REQUIRE(sim.setKinematicPose(0, moved));
    REQUIRE_FALSE(sim.setKinematicPose(1, moved));

    for (int i = 0; i < 60; ++i)
        sim.step(Simulation::kFixedDt);

    // The welded body follows body 2 (the enabled anchor), not the disabled one.
    Simulation::Pose p = sim.bodyPose(3);
    REQUIRE(p.pos.y > 1.2); // rest offset +1 kept -> tracks ~y=2
}

TEST_CASE("Box and capsule bodies simulate under gravity", "[sim]")
{
    Simulation::Definition def;
    def.gravity = Double3(0.0, -9.8, 0.0);

    Simulation::BodyDefinition box;
    box.mass = 1.0;
    box.colliderType = Simulation::ColliderType::eBox;
    box.extents = Double3(0.5, 0.5, 0.5);
    box.physicsMode = Simulation::PhysicsMode::ePhysics;
    def.bodies.push_back(box);

    Simulation::BodyDefinition capsule;
    capsule.mass = 1.0;
    capsule.colliderType = Simulation::ColliderType::eCapsule;
    capsule.radius = 0.5;
    capsule.length = 1.0;
    capsule.physicsMode = Simulation::PhysicsMode::ePhysics;
    def.bodies.push_back(capsule);

    Simulation sim;
    REQUIRE(sim.initialize(def));

    for (int i = 0; i < 60; ++i)
        sim.step(Simulation::kFixedDt);

    // Both shapes get valid inertia and fall freely (mask 0 -> no inter-body
    // collision, so they do not push each other around).
    Simulation::Pose pb = sim.bodyPose(0);
    Simulation::Pose pc = sim.bodyPose(1);
    REQUIRE(pb.pos.y < -1.0);
    REQUIRE(pc.pos.y < -1.0);
}

TEST_CASE("Point-to-point joint keeps the body at the anchor pivot", "[sim]")
{
    Simulation::Definition def;
    def.gravity = Double3(0.0, 0.0, 0.0);

    Simulation::BodyDefinition anchor;
    anchor.colliderType = Simulation::ColliderType::eSphere;
    anchor.radius = 0.5;
    anchor.physicsMode = Simulation::PhysicsMode::eFollowBone;
    def.bodies.push_back(anchor);

    Simulation::BodyDefinition dynamic;
    dynamic.colliderType = Simulation::ColliderType::eSphere;
    dynamic.radius = 0.5;
    dynamic.physicsMode = Simulation::PhysicsMode::ePhysics;
    dynamic.restPos = Double3(1.0, 1.0, 0.0);
    def.bodies.push_back(dynamic);

    Simulation::JointDefinition p2p;
    p2p.bodyA = 0;
    p2p.bodyB = 1;
    p2p.type = 2; // P2P
    // Joint frame at the body's rest position: the pivot coincides with the
    // body's COM, so the constraint force passes through it (no torque -> the
    // body translates without spinning, keeping the test deterministic).
    p2p.frameT = Double3(1.0, 1.0, 0.0);
    def.joints.push_back(p2p);

    Simulation sim;
    REQUIRE(sim.initialize(def));

    // Anchor up by 2 -> the body keeps its local pivot offset and lands at
    // (1, 3, 0).
    Simulation::Pose moved;
    moved.pos.y = 2.0;
    REQUIRE(sim.setKinematicPose(0, moved));

    for (int i = 0; i < 60; ++i)
        sim.step(Simulation::kFixedDt);

    Simulation::Pose p = sim.bodyPose(1);
    REQUIRE(p.pos.x == Catch::Approx(1.0).margin(0.05));
    REQUIRE(p.pos.y == Catch::Approx(3.0).margin(0.05));
    REQUIRE(p.pos.z == Catch::Approx(0.0).margin(0.05));
}
