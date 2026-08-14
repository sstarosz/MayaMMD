/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_node.hpp
 *
 * RigidBodyNode — native rigid-body physics node for MMD secondary movement.
 *
 * WHY A NATIVE NODE: the Bullet world lives inside this node and advances in
 * compute() whenever `time1.outTime` changes — the same evaluation path as a
 * parentConstraint, so it runs (and is re-stepped) under Cached Playback.  The
 * mayaBullet solver it replaces is a stateful node the evaluation cache does
 * not re-step, which froze dynamic bodies at rest.
 *
 * The node is an adapter over the Maya-free mmd::core::RigidBodySimulation engine: it
 * reads the scene attributes into a RigidBodySimulation::Definition, rebuilds the world
 * when those inputs change (or time is scrubbed backwards), steps it when time
 * advances or a kinematic anchor moves, and writes each dynamic body's solved
 * local pose to outTranslate[i]/outRotate[i] (which pmxRigidBody connects
 * directly into the related joints at create).  Registered by MayaMMD.mll.
 */

#pragma once

#include <maya/MEvaluationNode.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MTime.h>
#include <maya/MTypeId.h>

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

// The engine header is Bullet-free (the engine itself is a PIMPL), so pulling
// it in here stays cheap and keeps Bullet an implementation detail.
#include "rigid_body_simulation.hpp"

// ===========================================================================
// RigidBodyNode
// ===========================================================================
class RigidBodyNode : public MPxLocatorNode
{
  public:
    static const MTypeId kTypeId;
    static constexpr const char* kNodeName = "pmxRigidBodyNode";
    // VP2 draw-database classification.  A locator node must be registered
    // with a "drawdb/geometry/..." classification so VP2 renders it in the
    // viewport.
    static constexpr const char* kNodeClassify = "drawdb/geometry/pmxRigidBodyNode";

    // PMX rigid-body physics mode — stored in the bodyPhysicsMode enum
    // attribute.  The attribute VALUES are mmd::core::RigidBodySimulation::PhysicsMode
    // (eFollowBone=0 / ePhysics=1 / ePhysicsBone=2) so readBodyData can cast
    // directly — there is deliberately NO separate node-side enum (the PMX
    // mode is KEPT on every body, never collapsed to a bool, so follow-bone /
    // full-physics / rotation-only stay distinguishable downstream).
    // The bodyColliderType enum below DOES differ from the engine's (attribute
    // values are persisted in scenes), hence the explicit collider mapping.
    enum ColliderType : short
    {
        kColliderBox = 1,
        kColliderSphere = 2,
        kColliderCapsule = 3,
    };

    RigidBodyNode();
    // Defaulted out-of-line: the node is destroyed polymorphically through its
    // MPxNode base (Maya deletes it via the base pointer), and the default
    // teardown is exactly what we want — see the lifecycle notes in the cpp.
    ~RigidBodyNode() override;

    // MPxNode overrides
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;
    // This node owns a STATEFUL Bullet world that advances in compute().
    // Cached Playback must NOT treat its outputs as pure functions of its
    // inputs — disable caching so the node is re-evaluated every frame.
    // (This is exactly what mayaBullet's built-in solver could not declare,
    // which is why dynamic bodies froze under Cached Playback.)
    void getCacheSetup(const MEvaluationNode&, MNodeCacheDisablingInfo&, MNodeCacheSetupInfo&,
                       MObjectArray&) const override;

    // Registration helpers
    static void* creator();
    static MStatus initialize();

    // ------------------------------------------------------------------
    // Attributes
    // ------------------------------------------------------------------
    static MObject aTime;
    static MObject aGravity;

    // Per-body compound array: aBodies[i] — children are declared to mirror
    // the PMX rigid_bodies.json fields; aBodyEnabled (a Maya-only custom
    // attribute) sits first.
    static MObject aBodies;
    static MObject aBodyEnabled;       // bool (custom) — disabled bodies are skipped by buildWorld
    static MObject aBodyNameLocal;     // string — PMX name_local; "" = none
    static MObject aBodyNameUniversal; // string — PMX name_universal; "" = none
    static MObject aBodyGroupId;       // enum — PMX group_id 0..15 ("Group 0".."Group 15")
    // THE collision mask — one bool per collision group (0..15), True = the
    // body collides with that group.  This is the PMX non_collision_group
    // field stored VERBATIM (bit i set = collides with group i — MMD feeds it
    // to Bullet directly, no inversion); the node uses it exactly as read.
    // Declared as std::array so the attribute loops can use bounds-checked
    // .at() (the cppcoreguidelines constant-array-index check rejects `[]`
    // with a loop counter on a C array).
    static std::array<MObject, 16> aBodyMaskGroup;
    static MObject aBodyColliderType; // enum — PMX shape (kColliderBox/Sphere/Capsule)
    // PMX shape_size VERBATIM (3 doubles, full size).  The node derives the
    // engine's radius / box half-extents / capsule length by collider type
    // (mmd::core::applyShapeSize) in readBodyData.
    static MObject aBodyShapeSize;      // float3 — PMX shape_size verbatim
    static MObject aBodyRestTranslate;  // float3 — PMX shape_position (rest, world space)
    static MObject aBodyRestRotate;     // float3 — PMX shape_rotation (degrees)
    static MObject aBodyMass;           // double — PMX mass
    static MObject aBodyLinearDamping;  // double — PMX move_attenuation
    static MObject aBodyAngularDamping; // double — PMX rotation_damping
    static MObject aBodyRestitution;    // double — PMX repulsion
    static MObject aBodyFriction;       // double — PMX friction_force
    static MObject aBodyPhysicsMode;    // enum — PMX physics_mode (PhysicsMode)
    static MObject aBodyJoint;          // message child — the body's related joint (its
                                        // bone); the node resolves the bone index + the
                                        // hierarchy from it + the joint DAG.  Unconnected
                                        // = a static collider (no write-back).
    // The body's kinematic-anchor INPUT — a MATRIX child of the body compound
    // (the parentConstraint target[i].targetParentMatrix pattern).
    // pmxRigidBody connects joint.worldMatrix[0] into it for every FOLLOW_BONE
    // body with a related joint, so each body declares the bone world it
    // follows; a boneless FOLLOW_BONE body pins its own rest world instead.
    // The node applies the body<->joint rest offset (K^-1) on top.  The
    // Bullet world runs in WORLD space, so the solver's own location never
    // matters.  Unconnected = identity (dynamic bodies never read it).
    static MObject aBodyAnchorWorld;

    // Per-joint compound array: aJoints[j].
    static MObject aJoints;
    static MObject aJointNameLocal;      // string — PMX name_local; "" = none
    static MObject aJointNameUniversal;  // string — PMX name_universal; "" = none
    static MObject aJointBodyA;          // long
    static MObject aJointBodyB;          // long
    static MObject aJointType;           // enum 0..5 (PMX JointType — dropdown)
    static MObject aJointFrameTranslate; // float3
    static MObject aJointFrameRotate;    // float3 degrees
    static MObject aJointLinearMin;      // float3
    static MObject aJointLinearMax;      // float3
    static MObject aJointAngularMin;     // float3
    static MObject aJointAngularMax;     // float3
    static MObject aJointLinearSpring;   // float3
    static MObject aJointAngularSpring;  // float3

    // Outputs: solved local translate/rotate per body (compound arrays).
    // The children are UNIT-TYPED (MFnUnitAttribute kDistance / kAngle) —
    // exactly like transform.translate/rotate — so the write-back
    // connections to joint.translate / joint.rotate are DIRECT.  A unitless
    // k3Double forced Maya to auto-insert a unitConversion between the float3
    // and the joint's angle/linear attributes.
    static MObject aOutTranslate;
    static MObject aOutTranslateX; // kDistance child
    static MObject aOutTranslateY; // kDistance child
    static MObject aOutTranslateZ; // kDistance child
    static MObject aOutRotate;
    static MObject aOutRotateX; // kAngle child
    static MObject aOutRotateY; // kAngle child
    static MObject aOutRotateZ; // kAngle child

    // The full per-node simulation state: the Bullet world plus the config,
    // wiring, and derived write-back offsets it was built with, bundled and
    // replaced ATOMICALLY on rebuild.  A plain data record — public only so
    // the translation-unit frame functions can build / advance it; every
    // field is touched exclusively from rigid_body_node.cpp.
    struct World
    {
        mmd::core::RigidBodySimulation sim;
        std::vector<mmd::core::RigidBodySimulation::BodyDefinition> bodies; // wired config
        std::vector<mmd::core::RigidBodySimulation::JointDefinition> joints;
        mmd::core::Double3 gravity;
        std::vector<MMatrix> k; // write-back offsets (body-indexed; identity for no joint)
        // The kinematic anchors' RAW world matrices (bodyAnchorWorld, BEFORE the
        // K^-1 rest offset) from the previous evaluation, in kinematic order.
        // Used to detect a whole-skeleton rigid drag at a paused frame: when
        // every moved anchor shares the same world move, the character was
        // repositioned (not animated) and the dynamic chains ride along.
        std::vector<MMatrix> lastAnchorWorld;
        double lastTime = -1.0;
        MTime::Unit lastTimeUnit = MTime::kFilm;
    };

  private:
    // The simulation state; empty = no bodies (a valid no-op).  compute()
    // runs a pure frame transition (build or advance) that produces a NEW
    // World, which atomically replaces this one — the node never mutates
    // simulation state piecemeal.
    std::optional<World> mWorld;
};
