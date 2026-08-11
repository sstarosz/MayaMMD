/**
 * @file simulation.hpp
 * @brief Maya-free Bullet physics engine for MMD rigid bodies and joints.
 *
 * DESIGN: everything that actually IS the simulation — the Bullet world,
 * rigid bodies, constraints, collision filtering, kinematic anchors and the
 * scrub-back reset — lives in the private SimulationImpl (PIMPL), so THIS
 * header is Bullet-free: consumers only see the core value types, and Bullet
 * is an implementation detail (the Bullet-facing math lives in
 * bullet_bridge.hpp).
 * The engine can be unit-tested WITHOUT the Maya SDK and WITHOUT Bullet
 * headers (see tests/unit_tests/core/test_simulation.cpp).  A thin Maya node
 * (a later PR) adapts it: it reads attributes, converts Maya<->Bullet
 * transforms and owns the timeline/state machine.
 *
 * This class knows nothing about Maya: it consumes a plain Definition
 * (gravity + body/joint data), receives kinematic anchor poses as pos+quat,
 * steps the world and returns solved poses as pos+quat.  All transforms are
 * in the physics group's LOCAL space (the Bullet world frame); the adapter
 * owns every Maya matrix conversion.
 */

#pragma once

#include "common.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace mmd::core
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

    /// PMX collider shape.  Values match the PMX ShapeType byte and the
    /// Python ShapeType enum (data_types.py): 0 = sphere, 1 = box, 2 = capsule.
    enum class ColliderType : short
    {
        eSphere = 0,
        eBox = 1,
        eCapsule = 2,
    };

    /// One PMX rigid body.  `parentBodyIndex` is write-back wiring owned by
    /// the adapter (the engine ignores it); `resetAnchorIndex` drives the
    /// scrub-back reset.
    struct BodyDefinition
    {
        Double3 restPos; // PMX rest position (world space)
        Double3 restRot; // degrees
        double mass = 1.0;
        double linearDamping = 0.0;
        double angularDamping = 0.0;
        double friction = 0.5;
        double restitution = 0.0;
        ColliderType colliderType = ColliderType::eBox;
        double radius = 0.5;
        // HALF-extents, as btBoxShape expects — the adapter halves the PMX
        // full size before filling this.
        Double3 extents = Double3(1.0, 1.0, 1.0);
        double length = 1.0;
        // Bullet collision filter mask, passed VERBATIM to addRigidBody (bit
        // set = collides with that group).  The adapter must invert the PMX
        // non_collision_group bits (set = excluded) when filling this.
        long mask = 0;
        short groupId = 0; // raw PMX group id 0..15
        PhysicsMode physicsMode = PhysicsMode::ePhysics;
        bool enabled = true;
        int parentBodyIndex = -1;  // adapter write-back wiring (ignored by the engine)
        int resetAnchorIndex = -1; // kinematic anchor whose pose drives scrub-back reset

        /// FOLLOW_BONE bodies are kinematic anchors driven by their joints.
        bool isKinematic() const { return physicsMode == PhysicsMode::eFollowBone; }

        /// Field-wise equality — the Maya node uses it to detect config edits
        /// (a changed body definition must rebuild the world).
        bool operator==(const BodyDefinition& o) const
        {
            return restPos.x == o.restPos.x && restPos.y == o.restPos.y &&
                   restPos.z == o.restPos.z && restRot.x == o.restRot.x &&
                   restRot.y == o.restRot.y && restRot.z == o.restRot.z && mass == o.mass &&
                   linearDamping == o.linearDamping && angularDamping == o.angularDamping &&
                   friction == o.friction && restitution == o.restitution &&
                   colliderType == o.colliderType && radius == o.radius &&
                   extents.x == o.extents.x && extents.y == o.extents.y &&
                   extents.z == o.extents.z && length == o.length && mask == o.mask &&
                   groupId == o.groupId && physicsMode == o.physicsMode && enabled == o.enabled &&
                   parentBodyIndex == o.parentBodyIndex && resetAnchorIndex == o.resetAnchorIndex;
        }
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

        /// Field-wise equality — the Maya node uses it to detect config edits.
        bool operator==(const JointDefinition& o) const
        {
            return bodyA == o.bodyA && bodyB == o.bodyB && type == o.type &&
                   frameT.x == o.frameT.x && frameT.y == o.frameT.y && frameT.z == o.frameT.z &&
                   frameR.x == o.frameR.x && frameR.y == o.frameR.y && frameR.z == o.frameR.z &&
                   linearMin.x == o.linearMin.x && linearMin.y == o.linearMin.y &&
                   linearMin.z == o.linearMin.z && linearMax.x == o.linearMax.x &&
                   linearMax.y == o.linearMax.y && linearMax.z == o.linearMax.z &&
                   angularMin.x == o.angularMin.x && angularMin.y == o.angularMin.y &&
                   angularMin.z == o.angularMin.z && angularMax.x == o.angularMax.x &&
                   angularMax.y == o.angularMax.y && angularMax.z == o.angularMax.z &&
                   linearSpring.x == o.linearSpring.x && linearSpring.y == o.linearSpring.y &&
                   linearSpring.z == o.linearSpring.z && angularSpring.x == o.angularSpring.x &&
                   angularSpring.y == o.angularSpring.y && angularSpring.z == o.angularSpring.z;
        }
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
    // The Bullet world lives on the heap inside SimulationImpl, so MOVING is
    // safe (the unique_ptr transfers ownership without copying the world).
    // Copying is impossible — a Bullet world cannot be copied.
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) noexcept = default;
    Simulation& operator=(Simulation&&) noexcept = default;

    /// Build the Bullet world from `definition` (calling clear() first).
    /// Returns false when there are no bodies.  An edit requires a fresh
    /// initialize() (the adapter calls clear() + initialize() on config change).
    [[nodiscard]] bool initialize(const Definition& definition);

    /// True after a successful initialize() (world exists).
    [[nodiscard]] bool initialized() const;

    /// Destroy the world + all Bullet state (teardown order: world first).
    void clear();

    // ── Kinematic anchors ─────────────────────────────────────────────────
    /// Set the group-local pose of kinematic anchor `anchorIndex` (kinematic
    /// order = FOLLOW_BONE bodies in body order).  Returns true when the pose
    /// MOVED since the previous call — the adapter steps the sim in that case
    /// so a bone dragged at a fixed time is followed immediately.
    [[nodiscard]] bool setKinematicPose(size_t anchorIndex, const Pose& pose);

    /// Advance the simulation by `dt` seconds (substepped at kFixedDt).
    void step(double dt);

    /// Scrub-back reset: teleport every dynamic body to its rest pose
    /// transformed by the CURRENT skeleton pose (anchorCurrent * resetOffset),
    /// zeroing velocities.  Called when time is scrubbed backwards.
    void resetDynamicBodies();

    /// Solved world (group-space) pose of `bodyIndex` — body-indexed, matching
    /// Definition::bodies.  Falls back to the body's REST pose when the body
    /// is disabled/missing or the world is not initialized.
    [[nodiscard]] Pose bodyPose(size_t bodyIndex) const;

    /// MMD physics tick — the adapter steps with this fixed dt.
    static constexpr double kFixedDt = 1.0 / 60.0;

  private:
    /// Runtime body state: the (copied) input definition plus the scrub-back
    /// reset data captured at initialize().  BodyDefinition already carries
    /// every rest/physics field, so Body embeds it instead of duplicating the
    /// members — the only extra state is what the engine derives at runtime.
    struct Body
    {
        BodyDefinition def; // copied from Definition::bodies at initialize()

        // Scrub-back reset (captured at initialize): bodyRest = anchorRest *
        // resetOffset; on rewind the body is placed at anchorCurrent * resetOffset.
        bool hasBoneReset = false;
        Double3 resetOffsetPos;
        Double4 resetOffsetQuat = Double4(0.0, 0.0, 0.0, 1.0);
    };

    struct SimulationImpl;                 // all Bullet state lives here (PIMPL)
    std::unique_ptr<SimulationImpl> mImpl; // Bullet is an implementation detail
};

/// Map a PMX `shape_size` (3 doubles, FULL size — box extents are full, not
/// half) onto a BodyDefinition's radius/extents/length according to its
/// collider type.  The engine stores box extents as HALF-extents (as
/// btBoxShape expects), so boxes are halved here.  PMX verbatim in,
/// engine-ready out.
inline void applyShapeSize(Simulation::BodyDefinition& b, const Double3& size)
{
    switch (b.colliderType)
    {
    case Simulation::ColliderType::eSphere:
        b.radius = size.x; // sphere uses shape_size[0] as its radius
        break;
    case Simulation::ColliderType::eBox:
        b.extents = Double3(size.x * 0.5, size.y * 0.5, size.z * 0.5); // FULL -> half
        break;
    case Simulation::ColliderType::eCapsule:
        b.radius = size.x; // capsule: shape_size[0] = radius
        b.length = size.y; //          shape_size[1] = cylinder length
        break;
    }
}

/// Inverse of applyShapeSize — reconstruct the PMX `shape_size` (FULL size,
/// box extents doubled) from a BodyDefinition.  Used to build the draw-data
/// contract (DrawBody.shapeSize) from a solved/rest BodyDefinition.
inline Double3 shapeSizeFromBodyDefinition(const Simulation::BodyDefinition& b)
{
    switch (b.colliderType)
    {
    case Simulation::ColliderType::eSphere:
        return Double3(b.radius, 0.0, 0.0);
    case Simulation::ColliderType::eBox:
        return Double3(b.extents.x * 2.0, b.extents.y * 2.0, b.extents.z * 2.0);
    case Simulation::ColliderType::eCapsule:
        return Double3(b.radius, b.length, 0.0);
    }
    return Double3(0.0, 0.0, 0.0);
}

} // namespace mmd::core
