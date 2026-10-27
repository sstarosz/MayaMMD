/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint.hpp
 *
 * RigidBodyJoint — one selectable, movable Maya node per PMX rigid-body
 * JOINT (constraint), parallel to the per-body pmxRigidBodyShape.
 *
 * Each PMX joint becomes its own MPxLocatorNode under the model's
 * `{model}_Joints` sub-group (inside the `{model}_RigidBodies` group).  The
 * node is BOTH:
 *   1. a data holder — every PMX joint field except the FRAME is a storable
 *      attribute (the solver reads them through its `jointShapes[]` message
 *      array), except the two BODIES, which are message connections from
 *      their pmxRigidBodyShape nodes, and
 *   2. the joint's FRAME — unlike the bodies (whose guide transform is
 *      ANIMATED by the solver), a joint is static config, so its DAG
 *      transform IS the joint frame: moving/rotating the node repositions
 *      the constraint, and the solver reads the frame straight from the
 *      transform (translate + XYZ rotate) in the {model}_RigidBodies
 *      group's space — the space of the bodies' rest poses.
 *
 * The joint is drawn by RigidBodyJointGeometryOverride, registered for this
 * node's classification (`drawdb/geometry/pmxRigidBodyJoint`; see
 * docs/JointVisualization.md), with a per-node `drawMode` attribute.  The
 * node has no compute() — the solver pulls the attributes.
 *
 * Registered by MayaMMD.mll.
 */

#pragma once

#include <maya/MBoundingBox.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>

#include "rigid_body_simulation.hpp" // core JointDefinition

// ===========================================================================
// RigidBodyJoint
// ===========================================================================
class RigidBodyJoint : public MPxLocatorNode
{
  public:
    static const MTypeId kTypeId;
    static constexpr const char* kNodeName = "pmxRigidBodyJoint";
    // VP2 draw-database classification — the geometry override is registered
    // under this string.
    static constexpr const char* kNodeClassify = "drawdb/geometry/pmxRigidBodyJoint";
    // Custom selection type for the hub.  As a plain locator the hub ranks at
    // or below a Maya joint, so the bone it usually sits on always won the
    // pick.  The bone sphere is larger, so it stays pickable around the hub.
    static constexpr const char* kSelectionType = "pmxRigidBodyJoint";
    static constexpr int kSelectionPriority = 10;

    // PMX joint type — enum attribute values match the engine
    // JointDefinition::type.  PMX 2.0 only has Spring6Dof; the PMX 2.1 types
    // (1..5) are not supported yet.
    enum JointType : short
    {
        kSpring6Dof = 0,
    };

    // Viewport draw mode (see docs/JointVisualization.md):
    //   0 Off    — nothing
    //   1 Hub    — the state-coloured hub; when selected, the A/B attachments
    //   2 Ranges — Hub, plus the selected joint's motion-range guides
    enum DrawMode : short
    {
        kDrawOff = 0,
        kDrawHub = 1,
        kDrawRanges = 2,
    };

    RigidBodyJoint();
    ~RigidBodyJoint() override;

    // MPxNode overrides — data-holder only (no compute()).
    // Bounded by what the joint draws, around the hub's LIVE position on
    // body A (it leaves the frame as body A moves): Frame Selected works and
    // the joint can be culled.
    bool isBounded() const override { return true; }
    MBoundingBox boundingBox() const override;

    // The joint draws the solver's bodies, data from outside its own node,
    // so RigidBodyJointGeometryOverride updates only when told to: every
    // change to this node's inputs (solverStep after each solver step, the
    // frame, the joint fields) marks its geometry dirty.  Both paths are
    // needed: DG evaluation dirties plugs (setDependentsDirty), while the
    // Evaluation Manager propagates no dirty during playback and only
    // evaluates the node (postEvaluation).
    MStatus setDependentsDirty(const MPlug& plug, MPlugArray& affected) override;
    MStatus postEvaluation(const MDGContext& context, const MEvaluationNode& evaluationNode,
                           PostEvaluationType evalType) override;

    // Registration helpers
    static void* creator();
    static MStatus initialize();

    // Read this node into a core JointDefinition (used by the solver's
    // readJointData — it pulls each joint from its connected joint node).
    // The FRAME comes from the node's DAG transform, measured in the space of
    // `groupWorld` — the {model}_RigidBodies group's world matrix
    // (RigidBodyNode::groupWorldMatrix), the space the bodies' rest poses
    // live in, so moving the model root or the group never re-rigs the
    // joints.  The rest comes from attributes.  bodyA / bodyB stay -1: the
    // bodies are connections (bodyShape), which only the solver can turn
    // into its body indices (RigidBodyNode::bodyIndex).  Returns false when
    // `node` is not a pmxRigidBodyJoint.
    static bool readJointDefinition(const MObject& node, const MMatrix& groupWorld,
                                    mmd::core::RigidBodySimulation::JointDefinition& out);
    // The pmxRigidBodyShape connected to `end` (aJointBodyA / aJointBodyB);
    // null when the end is unconnected or not a rigid body.
    static MObject bodyShape(const MObject& joint, const MObject& end);
    // Just the FRAME (translate + XYZ rotate in degrees, in `groupWorld`'s
    // space) of a joint node; false when it has no parent transform.
    static bool readJointFrame(const MObject& node, const MMatrix& groupWorld,
                               mmd::core::Double3& translate, mmd::core::Double3& rotate);

    // ------------------------------------------------------------------
    // Attributes — the PMX joint data.  The joint FRAME (position +
    // orientation) is NOT an attribute — it is the node's DAG transform.
    // ------------------------------------------------------------------
    static MObject aSolver; // message -> owning pmxRigidBodyNode (wired at import)
    // Input <- solver.outStep: a real DG dependency, so every solver step
    // reaches the joint (which then marks its drawing dirty; see
    // postEvaluation) and reading it evaluates the solver first.  The value
    // itself is unused.
    static MObject aSolverStep;
    // Input <- the joint transform's worldMatrix[0] (wired by the command):
    // moving the frame dirties the solver through it.  The solver still reads
    // the frame from the DAG (readJointFrame); the value itself is unused.
    static MObject aFrameMatrix;
    static MObject aJointNameLocal;     // string — PMX name_local; "" = none
    static MObject aJointNameUniversal; // string — PMX name_universal; "" = none
    // message <- body A / B's pmxRigidBodyShape.message (PMX
    // rigid_body_index_a / _b, linked by node so it survives body edits).
    static MObject aJointBodyA;
    static MObject aJointBodyB;
    static MObject aJointType;          // enum — PMX joint type (JointType)
    static MObject aJointLinearMin;     // float3 — linear limit min (joint-frame offsets)
    static MObject aJointLinearMax;     // float3 — linear limit max (joint-frame offsets)
    static MObject
        aJointAngularMin; // angle3 — angular limit min (radians internally, UI-unit in scripts/AE)
    static MObject aJointAngularMax; // angle3 — angular limit max
    static MObject
        aJointLinearStiffness; // float3 — linear axis stiffness (Bullet spring stiffness)
    static MObject
        aJointAngularStiffness; // float3 — angular axis stiffness (Bullet spring stiffness)
    static MObject aDrawMode;   // enum — viewport draw mode (DrawMode)
};
