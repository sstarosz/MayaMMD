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
#include <maya/MObjectHandle.h>
#include <maya/MPlug.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MTime.h>
#include <maya/MTypeId.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <array>
#include <atomic>

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
    // Marks the config inputs stale, and flags a bodies[] dirty so compute()
    // re-reads the body records from the data block.  The bodies are a real
    // input, so this never needs a scene-wide serial.
    MStatus setDependentsDirty(const MPlug& plug, MPlugArray& affected) override;
    // The Evaluation Manager can evaluate without calling setDependentsDirty,
    // so the bodies[] dirt is also picked up here.
    MStatus preEvaluation(const MDGContext& context,
                          const MEvaluationNode& evaluationNode) override;
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

    // Read-only viewport snapshot of one joint's bodies and attachment frames
    // (world space).  Read the joint's solverStep first: that evaluates this
    // solver for the current time.
    bool jointViewportFrames(const mmd::core::RigidBodySimulation::JointDefinition& joint,
                             MMatrix& bodyA, MMatrix& bodyB, MMatrix& attachmentA,
                             MMatrix& attachmentB) const;
    // The simulation's body index of a pmxRigidBodyShape (what a joint's
    // jointBodyA/B connection resolves to); -1 when the shape is not one of
    // this solver's bodies or there is no world.
    long bodyIndex(const MObject& shape) const
    {
        return mWorld ? mWorld->bodyShapes.indexOf(shape) : -1;
    }
    // The pmxRigidBodyShape connected to one end of a pmxRigidBodyJoint
    // (`end` = RigidBodyJoint::aJointBodyA / aJointBodyB); null when that end
    // is unconnected or does not point at a rigid body.  Static: the body
    // links are identity, independent of which solver owns the joint.
    static MObject jointBodyShape(const MObject& joint, const MObject& end);
    // World matrix of the solver's DAG parent (the {model}_RigidBodies
    // group), identity when it has none.  Body and joint guide outputs are
    // expressed in this space (the bodies' rest poses live there).
    static MMatrix groupWorldMatrix(const MObject& solver);

    // ------------------------------------------------------------------
    // Attributes
    // ------------------------------------------------------------------
    static MObject aTime;
    static MObject aGravity;

    // Viewport-only hub size multiplier, consumed by outJointDisplay. It
    // never dirties the simulation outputs or rebuilds the Bullet world.
    static MObject aJointDisplayScale;

    // Output -> every joint's solverStep: the joints' viewport dependency on
    // the simulation (see RigidBodyJoint::aSolverStep).  Holds the evaluated
    // time in seconds.
    static MObject aOutStep;
    // Configuration-only display output -> joint.jointDisplay. Authored
    // changes dirty this output; simulation time and anchors do not.
    static MObject aOutJointDisplay;
    static MObject aOutJointHubRadius;
    static MObject aOutJointActive;
    static MObject aOutJointBoundsMin;
    static MObject aOutJointBoundsMax;

    // Per-body message array: aBodyShapes[i] -> the pmxRigidBodyShape node
    // for PMX body i (PMX order — one selectable/movable locator per body).
    // This edge is IDENTITY, not data: it fixes the shape -> body-index
    // mapping the joints and the output slots resolve through, and its dirt
    // reports a re-bind / deleted body (see inputsStale).  The body's
    // PMX-verbatim VALUES arrive through aBodyBodies below — a real input,
    // read from the data block, with the rest pose in the shape's
    // bodyRestTranslate/bodyRestRotate (the guide TRANSFORM is the CURRENT
    // pose, driven by the solver; it is not read).
    // Unconnected slots remain disabled placeholders to preserve output indices.
    static MObject aBodyShapes;

    // Per-body simulation INPUT: aBodyBodies[i] is connected from a
    // pmxRigidBodyShape's `body` compound by pmxRigidBody (one connectAttr per
    // body, expanded to one edge per child).  The element is decoded by
    // decodeBody() from the data block; the shape's long names are repeated
    // here because a compound-to-compound connection pairs children by name.
    static MObject aBodyBodies;
    static MObject aInEnabled;
    static MObject aInGroupId;
    static std::array<MObject, 16> aInMaskGroup;
    static MObject aInColliderType;
    static MObject aInShapeSize;
    static MObject aInRestTranslate;
    static MObject aInRestRotate;
    static MObject aInMass;
    static MObject aInLinearDamping;
    static MObject aInAngularDamping;
    static MObject aInRestitution;
    static MObject aInFriction;
    static MObject aInPhysicsMode;

    // Per-body kinematic-anchor input: aAnchorWorlds[i] <- the related joint's
    // worldMatrix[0] (a real DATA connection, wired by pmxRigidBody), read
    // through the data block.  This is the solver's only per-frame input, and
    // it lives HERE rather than on the shape so the shape stays authored PMX
    // data: a bone move dirties this plug, Maya carries it to the outputs, and
    // the data block caches the value across the solver pulls of one frame.
    // A body with no related joint stores its pinned rest world here (the
    // pmxRigidBody command sets the element value); dynamic bodies leave the
    // element unset — their anchors are never read.
    //
    // A bone that ALSO drives a dynamic body's write-back cannot feed this
    // array: joint.worldMatrix -> anchorWorlds -> outTranslate ->
    // joint.translate would be a DG cycle, so the command pins the anchor
    // instead (see pmxRigidBody).
    static MObject aAnchorWorlds;

    // Per-joint message array: aJointShapes[i] -> the pmxRigidBodyJoint node
    // for PMX joint i (PMX order — one selectable/movable locator per joint).
    // This edge is IDENTITY, not data: it fixes the joint -> index mapping and
    // lets the solver resolve each joint's body links (jointBodyA/B messages)
    // to body indices.  The joint's PMX-verbatim VALUES — the frame included —
    // arrive through aJoints below, a real input read from the data block.
    // Unconnected slots remain placeholders to preserve output indices.
    static MObject aJointShapes;

    // Per-joint simulation INPUT: aJoints[i] is connected from a
    // pmxRigidBodyJoint's `joint` compound by pmxRigidBodyConstraint (one
    // connectAttr per joint, expanded to one edge per child).  The element is
    // decoded by decodeJoint() from the data block; the joint node's long
    // names are repeated here because a compound-to-compound connection pairs
    // children by name.
    static MObject aJoints;
    static MObject aInJointRestTranslate;
    static MObject aInJointRestRotate;
    static MObject aInJointLinearMin;
    static MObject aInJointLinearMax;
    static MObject aInJointAngularMin;
    static MObject aInJointAngularMax;
    static MObject aInJointLinearStiffness;
    static MObject aInJointAngularStiffness;

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

    // Joint guide outputs: each joint's live attachment on body A (in the
    // RigidBodies group's space) — written for EVERY joint index.
    // pmxRigidBodyConstraint connects them to the joint guide transforms, so
    // the node's pivot follows the drawn hub (and the constraint).
    static MObject aOutJointTranslate;
    static MObject aOutJointTranslateX; // kDistance child
    static MObject aOutJointTranslateY; // kDistance child
    static MObject aOutJointTranslateZ; // kDistance child
    static MObject aOutJointRotate;
    static MObject aOutJointRotateX; // kAngle child
    static MObject aOutJointRotateY; // kAngle child
    static MObject aOutJointRotateZ; // kAngle child

    // The connected pmxRigidBodyShape nodes in body order, with a lookup from
    // a shape to its body index (joints link bodies by connection).
    struct BodyShapes
    {
        std::vector<MObjectHandle> nodes;
        // hashCode -> body index, kept sorted (a flat map).  A vector's move
        // is a pointer swap, whereas every associative container allocates a
        // sentinel node in its move constructor on MSVC — which would
        // terminate when the implicitly noexcept move of the record holding
        // this one (Inputs, World) runs.
        std::vector<std::pair<unsigned int, long>> byHash;
        void add(const MObject& shape);
        [[nodiscard]] long indexOf(const MObject& shape) const;
    };

    // The PMX-verbatim inputs of one evaluation: what the solver reads from
    // the scene's body/joint nodes.  Not the node's state — compute() keeps a
    // copy (mInputs) so an unedited frame does not have to read it again.
    struct Inputs
    {
        std::vector<mmd::core::RigidBodySimulation::BodyDefinition> bodies;
        BodyShapes bodyShapes; // the shape node of each body
        std::vector<mmd::core::RigidBodySimulation::JointDefinition> joints;
        mmd::core::Double3 gravity;
    };

    // The full per-node simulation state: the Bullet world plus the config,
    // wiring, and derived write-back offsets it was built with, bundled and
    // replaced ATOMICALLY on rebuild.  A plain data record — public only so
    // the translation-unit frame functions can build / advance it; every
    // field is touched exclusively from rigid_body_node.cpp.
    struct World
    {
        mmd::core::RigidBodySimulation sim;
        std::vector<mmd::core::RigidBodySimulation::BodyDefinition> bodies; // wired config
        BodyShapes bodyShapes; // the shape node of each body
        std::vector<mmd::core::RigidBodySimulation::JointDefinition> joints;
        mmd::core::Double3 gravity;
        std::vector<MMatrix> k; // write-back offsets (body-indexed; identity for no joint)
        // The kinematic anchors' RAW world matrices (anchorWorlds[i], BEFORE the
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
        // Per-anchor constant for the per-frame anchor update (body-indexed;
        // identity = no rest offset): bodyRest * anchorRest^-1, the raw rigid
        // attachment.  Resolved at build so the per-frame work is one data
        // block read and one matrix multiply per anchor, instead of rebuilding
        // and inverting the rest matrices every frame.
        std::vector<MMatrix> anchorOffset;
        double lastTime = -1.0;
        MTime::Unit lastTimeUnit = MTime::kFilm;
    };

  private:
    // The simulation state; empty = no bodies (a valid no-op).  compute()
    // runs a pure frame transition (build or advance) that produces a NEW
    // World, which atomically replaces this one — the node never mutates
    // simulation state piecemeal.
    std::optional<World> mWorld;

    // ── Input cache ───────────────────────────────────────────────────────
    // The PMX-verbatim inputs are expensive to read, so they are cached:
    //   * the BODY records come from the `bodies[]` data-block input, re-read
    //     when that plug arrived dirty (mBodiesDirty, set from both
    //     setDependentsDirty and preEvaluation),
    //   * the JOINT records come from the `joints[]` data-block input the same
    //     way (mJointsDirty), and
    //   * a bodyShapes[]/jointShapes[] element dirt whose connection no longer
    //     matches the cached shape (a re-bind or a deleted body) is structural.
    Inputs mInputs;
    bool mInputsValid = false;
    // Set when the data block's bodies[]/joints[] arrived dirty (DG or
    // Evaluation Manager); compute() re-reads those records then.
    std::atomic<bool> mBodiesDirty{true};
    std::atomic<bool> mJointsDirty{true};
    std::vector<bool> mBodySlotDirty;  // bodyShapes[i] dirtied since the last read
    std::vector<bool> mJointSlotDirty; // jointShapes[i] dirtied since the last read

    // True when the whole input set must be read again: a reported config
    // edit, an element whose connection changed, or no cached inputs.
    [[nodiscard]] bool inputsStale(const MObject& node) const;
};
