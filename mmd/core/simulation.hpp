/**
 * @file simulation.hpp
 * @brief Maya-free Bullet physics engine for MMD rigid bodies and joints.
 *
 * DESIGN: everything that actually IS the simulation — the Bullet world,
 * rigid bodies, constraints, collision filtering, kinematic anchors and the
 * scrub-back reset — lives in this class, so it can be unit-tested WITHOUT
 * the Maya SDK (see tests/core/test_simulation.cpp) and reasoned about in
 * isolation.  A thin Maya node (a later PR) adapts it: it reads attributes,
 * converts Maya<->Bullet transforms and owns the timeline/state machine.
 *
 * This class knows nothing about Maya: it consumes a plain Definition
 * (gravity + body/joint data), receives kinematic anchor poses as pos+quat,
 * steps the world and returns solved poses as pos+quat.  All transforms are
 * in the physics group's LOCAL space (the Bullet world frame); the adapter
 * owns every Maya matrix conversion.
 *
 * Bullet usage: btDiscreteDynamicsWorld, the MMD joint-type -> constraint
 * mapping (SPRING_6DOF -> btGeneric6DofSpring2Constraint rigid welds, 6DOF /
 * P2P / cone-twist / slider / hinge), 30 solver iterations for long chains,
 * substepping at kFixedDt.  See docs/PhysicsImplementation.md.
 */

#pragma once

#include "common.hpp"

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

// The Bullet types held BY VALUE (through std::optional) are included above.
// These three stay forward-declared — they are only ever held through
// std::unique_ptr / raw pointers.
class btCollisionShape;
class btRigidBody;
class btTypedConstraint;

namespace mmd
{
namespace core
{

/** @brief Simulates a PMX rigid-body world with embedded Bullet. */
class Simulation
{
  public:
    /// PMX rigid-body physics mode (PMX PhysicsMode: 0 = follow bone,
    /// 1 = physics, 2 = physics + bone).
    enum class PhysicsMode : short
    {
        eFollowBone = 0, // kinematic anchor — driven by the related joint
        ePhysics = 1,    // full dynamic body
        ePhysicsBone = 2 // dynamic, rotation-only write-back
    };

    /// PMX collider shape (PMX ShapeType: 1 = box, 2 = sphere, 3 = capsule).
    enum class ColliderType : short
    {
        eBox = 1,
        eSphere = 2,
        eCapsule = 3,
    };

    /// One PMX rigid body.  `parentBodyIndex` is write-back wiring owned by
    /// the adapter (the engine ignores it); `resetAnchorIndex` drives the
    /// scrub-back reset.
    struct BodyDefinition
    {
        Double3 restPos; // PMX rest position (group space)
        Double3 restRot; // degrees
        double mass = 1.0;
        double linearDamping = 0.0;
        double angularDamping = 0.0;
        double friction = 0.5;
        double restitution = 0.0;
        ColliderType colliderType = ColliderType::eBox;
        double radius = 0.5;
        Double3 extents = Double3(1.0, 1.0, 1.0);
        double length = 1.0;
        long mask = 0;     // collision mask (PMX non_collision_group, verbatim)
        short groupId = 0; // raw PMX group id 0..15
        PhysicsMode physicsMode = PhysicsMode::ePhysics;
        bool enabled = true;
        int parentBodyIndex = -1;  // adapter write-back wiring (ignored by the engine)
        int resetAnchorIndex = -1; // kinematic anchor whose pose drives scrub-back reset

        /// FOLLOW_BONE bodies are kinematic anchors driven by their joints.
        bool isKinematic() const { return physicsMode == PhysicsMode::eFollowBone; }
    };

    /// One PMX joint (a rigid-body constraint between two bodies).
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

    /// Full simulation input: gravity + bodies + joints.
    struct Definition
    {
        Double3 gravity = Double3(0.0, -9.8, 0.0);
        std::vector<BodyDefinition> bodies;
        std::vector<JointDefinition> joints;
    };

    /// A group-space pose (quat is a unit quaternion {x, y, z, w}).
    struct Pose
    {
        Double3 pos;
        Double4 quat = Double4(0.0, 0.0, 0.0, 1.0);
    };

    Simulation();
    ~Simulation();
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) noexcept = default;
    Simulation& operator=(Simulation&&) noexcept = default;

    /// Build the Bullet world from `definition` (calling clear() first).
    /// Returns false when there are no bodies.  An edit requires a fresh
    /// initialize() (the adapter calls clear() + initialize() on config change).
    bool initialize(const Definition& definition);

    /// True after a successful initialize() (world exists).
    bool initialized() const;

    /// Destroy the world + all Bullet state (teardown order: world first).
    void clear();

    // ── Kinematic anchors ─────────────────────────────────────────────────
    /// Set the group-local pose of kinematic anchor `anchorIndex` (kinematic
    /// order = FOLLOW_BONE bodies in body order).  Returns true when the pose
    /// MOVED since the previous call — the adapter steps the sim in that case
    /// so a bone dragged at a fixed time is followed immediately.
    bool setKinematicPose(size_t anchorIndex, const Pose& pose);

    /// Advance the simulation by `dt` seconds (substepped at kFixedDt).
    void step(double dt);

    /// Scrub-back reset: teleport every dynamic body to its rest pose
    /// transformed by the CURRENT skeleton pose (anchorCurrent * resetOffset),
    /// zeroing velocities.  Called when time is scrubbed backwards.
    void resetDynamicBodies();

    /// Solved world (group-space) pose of `bodyIndex` — body-indexed, matching
    /// Definition::bodies.  Falls back to the body's REST pose when the body
    /// is disabled/missing or the world is not initialized.
    Pose bodyPose(size_t bodyIndex) const;

    /// MMD physics tick — the adapter steps with this fixed dt.
    static constexpr double kFixedDt = 1.0 / 60.0;

  private:
    /// Runtime body state (rest pose + scrub-back reset data captured at
    /// initialize()).
    struct Body
    {
        Double3 restPos;
        Double3 restRot;
        double mass = 1.0;
        double linearDamping = 0.0;
        double angularDamping = 0.0;
        double friction = 0.5;
        double restitution = 0.0;
        ColliderType colliderType = ColliderType::eBox;
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

    /// Anchor poses in kinematic order, kept as pos+quat (no Bullet type).
    struct AnchorPose
    {
        Double3 pos;
        Double4 quat = Double4(0.0, 0.0, 0.0, 1.0);
    };

    /// Create the Bullet world + support objects and configure gravity/solver.
    void createWorld(const Double3& gravity);

    /// Create every body's Bullet shape + rigid body and add it to the world.
    void createBodies();

    /// Create one joint (constraint) and add it to the world.
    void createJoint(const JointDefinition& joint);

    // Bullet world-level objects.  The world does NOT own its dispatcher /
    // broadphase / collision config / solver, and rigid bodies do NOT own
    // their collision shapes — the Simulation keeps them alive and tears them
    // down in clear().  The world + its support objects are held BY VALUE in
    // std::optional (no heap allocation) and mWorld is declared LAST so that
    // when members are destroyed (reverse declaration order) the world goes
    // down FIRST, while every body / shape / constraint it references is still
    // alive — exactly what btCollisionWorld's destructor needs (it walks
    // m_collisionObjects and calls destroyProxy() on each live body).
    std::optional<btDefaultCollisionConfiguration> mCollisionConfig;
    std::optional<btCollisionDispatcher> mDispatcher;
    std::optional<btDbvtBroadphase> mBroadphase;
    std::optional<btSequentialImpulseConstraintSolver> mConstraintSolver;
    // Bodies / shapes / constraints are polymorphic Bullet types, so they stay
    // heap-allocated (std::unique_ptr); the explicit reset order in clear()
    // keeps them alive until after the world is gone.
    std::vector<std::unique_ptr<btRigidBody>> mRigidBodies; // body-indexed (null = disabled)
    std::vector<std::unique_ptr<btTypedConstraint>> mConstraints;
    std::vector<std::unique_ptr<btCollisionShape>> mShapes;
    std::vector<Body> mBodies;                 // body-indexed runtime state
    std::vector<size_t> mKinematicBodyIndices; // kinematic body indices, in anchor order
    std::vector<AnchorPose> mAnchorRest;       // anchor rest poses (kinematic order)
    std::vector<AnchorPose> mAnchorCurrent;    // anchor current poses (kinematic order)
    bool mWorldBuilt = false;
    std::optional<btDiscreteDynamicsWorld> mWorld; // declared last → destroyed first
};

} // namespace core
} // namespace mmd
