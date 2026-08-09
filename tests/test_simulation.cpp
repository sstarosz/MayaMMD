/*
 * SPDX-License-Identifier: MIT
 *
 * test_mmd_simulation.cpp
 *
 * Unit tests for the Maya-free physics engine (Simulation).  This target
 * links ONLY Bullet + Catch2 (no Maya SDK) — compiling and passing here is
 * what proves the engine is truly Maya-free.
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "simulation.hpp"

using mmd::core::Simulation;
using mmd::core::Double3;
using mmd::core::Double4;

namespace
{

// A single dynamic sphere at the origin with gravity.
Simulation::Definition gravityDefinition(double gy)
{
    Simulation::Definition def;
    def.gravity = Double3(0.0, gy, 0.0);

    Simulation::BodyDefinition bd;
    bd.mass = 1.0;
    bd.colliderType = Simulation::ColliderType::Sphere;
    bd.radius = 0.5;
    bd.physicsMode = Simulation::PhysicsMode::Physics;
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
    anchor.colliderType = Simulation::ColliderType::Sphere;
    anchor.radius = 0.5;
    anchor.physicsMode = Simulation::PhysicsMode::FollowBone;
    def.bodies.push_back(anchor);

    Simulation::BodyDefinition dynamic;
    dynamic.colliderType = Simulation::ColliderType::Sphere;
    dynamic.radius = 0.5;
    dynamic.physicsMode = Simulation::PhysicsMode::Physics;
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
