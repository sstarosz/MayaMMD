/*
 * SPDX-License-Identifier: MIT
 *
 * simulation.hpp
 *
 * mmd::core::Simulation — the Maya-free Bullet physics engine behind MMDPhysicsNode.
 *
 * WHY SEPARATE: mmd_physics_node.cpp is a Maya adapter (it reads attributes,
 * converts Maya<->Bullet transforms, and owns the timeline/state machine).
 * Everything that actually IS the simulation — the Bullet world, rigid
 * bodies, constraints, collision filtering, kinematic anchors, scrub-back
 * reset — lives here, so it can be unit-tested WITHOUT the Maya SDK (see
 * tests/test_mmd_simulation.cpp) and reasoned about in isolation.
 *
 * This class knows nothing about Maya: it consumes a plain Definition
 * (gravity + body/joint data), receives kinematic anchor poses as pos+quat,
 * steps the world, and returns solved poses as pos+quat.  All transforms are
 * in the physics group's LOCAL space (the Bullet world frame); the node owns
 * every Maya matrix conversion.
 *
 * Bullet usage mirrors the previous in-node implementation — see
 * docs/PhysicsImplementation.md: btDiscreteDynamicsWorld, the MMD joint-type
 * -> constraint mapping (SPRING_6DOF -> btGeneric6DofSpring2Constraint rigid
 * welds, 6DOF / P2P / cone-twist / slider / hinge), 30 solver iterations for
 * long chains, substepping at kFixedDt.
 */

#pragma once

#include "common.hpp"

#include <cstddef>
#include <memory>
#include <vector>

class btCollisionShape;
class btCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btConstraintSolver;
class btDiscreteDynamicsWorld;
class btRigidBody;
class btTypedConstraint;

namespace mmd
{
namespace core
{

class Simulation
{
  public:
    // PMX rigid-body physics mode (mirrors MMDPhysicsNode::PhysicsMode — the
    // node maps its attribute enum onto these values).
    enum class PhysicsMode : short
    {
        FollowBone = 0, // kinematic anchor — driven by the related joint
        Physics = 1,    // full dynamic body
        PhysicsBone = 2 // dynamic, rotation-only write-back
    };

    // PMX collider shape (mirrors MMDPhysicsNode::ColliderType; PMX ShapeType
    // values 1/2/3).
    enum class ColliderType : short
    {
        Box = 1,
        Sphere = 2,
        Capsule = 3,
    };

    // One PMX rigid body.  Fields mirror the node's bodies[i] compound
    // attributes.  `parentBodyIndex` is node write-back wiring (the engine
    // ignores it); `resetAnchorIndex` drives the scrub-back reset.
    struct BodyDefinition
    {
        Double3 restPos; // PMX rest position (group space)
        Double3 restRot; // degrees
        double mass = 1.0;
        double linearDamping = 0.0;
        double angularDamping = 0.0;
        double friction = 0.5;
        double restitution = 0.0;
        ColliderType colliderType = ColliderType::Box;
        double radius = 0.5;
        Double3 extents = Double3(1.0, 1.0, 1.0);
        double length = 1.0;
        long mask = 0;     // collision mask (PMX non_collision_group, verbatim)
        short groupId = 0; // raw PMX group id 0..15
        PhysicsMode physicsMode = PhysicsMode::Physics;
        bool enabled = true;
        int parentBodyIndex = -1;  // node write-back wiring (ignored by the engine)
        int resetAnchorIndex = -1; // kinematic anchor whose pose drives scrub-back reset

        // FOLLOW_BONE bodies are kinematic anchors driven by their joints.
        bool isKinematic() const { return physicsMode == PhysicsMode::FollowBone; }
    };

    struct JointDefinition
    {
        long bodyA = -1;
        long bodyB = -1;
        long type = 0; // PMX JointType value
        Double3 frameT;
        Double3 frameR; // degrees
        Double3 linearMin;
        Double3 linearMax;
        Double3 angularMin; // radians
        Double3 angularMax;
        Double3 linearSpring;
        Double3 angularSpring;
    };

    struct Definition
    {
        Double3 gravity = Double3(0.0, -9.8, 0.0);
        std::vector<BodyDefinition> bodies;
        std::vector<JointDefinition> joints;
    };

    // A group-space pose (quat is a unit quaternion {x, y, z, w}).
    struct Pose
    {
        Double3 pos;
        Double4 quat = Double4(0.0, 0.0, 0.0, 1.0);
    };

    Simulation();
    ~Simulation();
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    // Build the Bullet world from `definition` (calling clear() first).
    // Returns false when there are no bodies.  An edit requires a fresh
    // initialize() (the node calls clear() + initialize() on config change).
    bool initialize(const Definition& definition);
    bool initialized() const;
    // Destroy the world + all Bullet state (teardown order: world first).
    void clear();

    // Kinematic anchors ----------------------------------------------------
    // Set the group-local pose of kinematic anchor `anchorIndex` (kinematic
    // order = FOLLOW_BONE bodies in body order).  Returns true when the pose
    // MOVED since the previous call — the node steps the sim in that case so
    // a bone dragged at a fixed time is followed immediately.
    bool setKinematicPose(size_t anchorIndex, const Pose& pose);

    // Advance the simulation by `dt` seconds (substepped at kFixedDt).
    void step(double dt);

    // Scrub-back reset: teleport every dynamic body to its rest pose
    // transformed by the CURRENT skeleton pose (anchorCurrent * resetOffset),
    // zeroing velocities.  Called when time is scrubbed backwards.
    void resetDynamicBodies();

    // Solved world (group-space) pose of `bodyIndex` — body-indexed, matching
    // Definition::bodies.  Falls back to the body's REST pose when the body is
    // disabled/missing or the world is not initialized.
    Pose bodyPose(size_t bodyIndex) const;

    // MMD physics tick — the node steps with this fixed dt.
    static constexpr double kFixedDt = 1.0 / 60.0;

  private:
    // Runtime body state (rest pose + scrub-back reset data captured at
    // initialize()).  `parentBodyIndex` lives only in BodyDefinition (node
    // write-back wiring) — the engine never needs it.
    struct Body
    {
        Double3 restPos;
        Double3 restRot;
        double mass = 1.0;
        double linearDamping = 0.0;
        double angularDamping = 0.0;
        double friction = 0.5;
        double restitution = 0.0;
        ColliderType colliderType = ColliderType::Box;
        double radius = 0.5;
        Double3 extents = Double3(1.0, 1.0, 1.0);
        double length = 1.0;
        long mask = 0;
        short groupId = 0;
        bool kinematic = false;
        bool enabled = true;
        // Scrub-back reset (captured at initialize): bodyRest = anchorRest *
        // resetOffset; on rewind the body is placed at anchorCurrent * resetOffset.
        int resetAnchorIndex = -1;
        bool hasBoneReset = false;
        Double3 resetOffsetPos;
        Double4 resetOffsetQuat = Double4(0.0, 0.0, 0.0, 1.0);
    };

    // Anchor poses in kinematic order, kept as pos+quat (no Bullet type).
    struct AnchorPose
    {
        Double3 pos;
        Double4 quat = Double4(0.0, 0.0, 0.0, 1.0);
    };

    // Bullet world + state.  The world does NOT own its dispatcher / broadphase
    // / collision config / solver, and rigid bodies do NOT own their collision
    // shapes — keep them here so they are freed exactly once in clear()
    // (world before bodies, which btCollisionWorld's destructor needs).
    std::unique_ptr<btDiscreteDynamicsWorld> mWorld;
    std::vector<std::unique_ptr<btRigidBody>> mRigidBodies; // body-indexed (null = disabled)
    std::vector<std::unique_ptr<btTypedConstraint>> mConstraints;
    std::unique_ptr<btCollisionConfiguration> mCollisionConfig;
    std::unique_ptr<btCollisionDispatcher> mDispatcher;
    std::unique_ptr<btBroadphaseInterface> mBroadphase;
    std::unique_ptr<btConstraintSolver> mConstraintSolver;
    std::vector<std::unique_ptr<btCollisionShape>> mShapes;
    std::vector<Body> mBodies;                 // body-indexed runtime state
    std::vector<size_t> mKinematicBodyIndices; // kinematic body indices, in anchor order
    std::vector<AnchorPose> mAnchorRest;       // anchor rest poses (kinematic order)
    std::vector<AnchorPose> mAnchorCurrent;    // anchor current poses (kinematic order)
    bool mWorldBuilt = false;
};

} // namespace core
} // namespace mmd
