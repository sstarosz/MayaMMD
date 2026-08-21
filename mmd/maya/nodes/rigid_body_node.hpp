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

#include <maya/MDagPath.h>
#include <maya/MEvaluationNode.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MTime.h>
#include <maya/MTypeId.h>

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

    // Per-body message array: aBodyShapes[i] -> the pmxRigidBodyShape node
    // for PMX body i (PMX order — one selectable/movable locator per body).
    // The solver pulls every body's PMX-verbatim data from the connected
    // shape node via RigidBodyShape::readBodyDefinition (including its rest
    // pose, which is the shape's transform — moving the guide in the viewport
    // therefore changes the sim config and triggers a rebuild).  Unconnected
    // slots are skipped.
    static MObject aBodyShapes;

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

    // Guide outputs: each body's CURRENT world pose (in the RigidBodies
    // group's space) — written for EVERY body index.  The pmxRigidBody
    // command connects them to the per-body guide transforms, so the guide
    // (and its collider) follows the animation.  Same unit-typed compound
    // pattern as outTranslate/outRotate, so the connections to the guide's
    // translate/rotate are direct.
    static MObject aOutGuideTranslate;
    static MObject aOutGuideTranslateX; // kDistance child
    static MObject aOutGuideTranslateY; // kDistance child
    static MObject aOutGuideTranslateZ; // kDistance child
    static MObject aOutGuideRotate;
    static MObject aOutGuideRotateX; // kAngle child
    static MObject aOutGuideRotateY; // kAngle child
    static MObject aOutGuideRotateZ; // kAngle child

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
        // The kinematic anchors' REST worlds (the joint's composed rest world
        // from the stamped pmxRest* attributes, or the body's own rest world
        // for a boneless pin), in kinematic order — the model-constant "rest"
        // reference for the whole-skeleton-move detector on a REBUILD and for
        // the raw reset / raw kinematic placement.  Persisted across
        // scrub-back rebuilds (see frame()); re-captured on a config change or
        // first build.
        std::vector<MMatrix> originalAnchorWorld;
        // The related joint's DAG path per body (resolved at build), used by
        // the write-back fallback: when a dynamic body's parent bone has no
        // body, the parent joint's CURRENT world is needed to express the
        // solved bone world as a joint-local pose.  Invalid for bodies with no
        // connected joint (static colliders).
        std::vector<MDagPath> jointPaths;
        // The RigidBodies group's inclusive matrix (the solver's DAG parent —
        // also the parent of every per-body guide transform).  The guide
        // outputs express each body's WORLD pose in this space, so the driven
        // guide lands where the body actually is even if the group was moved.
        MMatrix groupWorld;
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
