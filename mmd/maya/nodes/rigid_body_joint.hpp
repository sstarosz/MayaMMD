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
 *      array), and
 *   2. the joint's FRAME — unlike the bodies (whose guide transform is
 *      ANIMATED by the solver), a joint is static config, so its DAG
 *      transform IS the joint frame: moving/rotating the node repositions
 *      the constraint, and the solver reads the frame straight from the
 *      transform in WORLD space (translate + XYZ rotate).
 *
 * The joint is drawn by a geometry override registered for this node's
 * classification (`drawdb/geometry/pmxRigidBodyJoint`) — a lit solid showing
 * the constraint's linear-limit range — with a per-node `drawMode`
 * attribute.  The node has no compute() — the solver pulls the attributes.
 *
 * Registered by MayaMMD.mll.
 */

#pragma once

#include <maya/MBoundingBox.h>
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

    // PMX joint type — enum attribute values (persisted in scenes, so they
    // must never change).  Values match the engine JointDefinition::type.
    enum JointType : short
    {
        kSpring6Dof = 0,
        kSixDof = 1,
        kP2P = 2,
        kConeTwist = 3,
        kSlider = 4,
        kHinge = 5,
    };

    // Viewport draw mode (0 = off, 1 = wire, 2 = solid, 3 = wire + solid).
    enum DrawMode : short
    {
        kDrawOff = 0,
        kDrawWire = 1,
        kDrawSolid = 2,
        kDrawWireSolid = 3,
    };

    RigidBodyJoint();
    ~RigidBodyJoint() override;

    // MPxNode overrides — data-holder only (no compute()).
    MBoundingBox boundingBox() const override; // object-space limit box

    // Registration helpers
    static void* creator();
    static MStatus initialize();

    // Read this node into a core JointDefinition (used by the solver's
    // readJointData — it pulls each joint from its connected joint node).
    // The FRAME comes from the node's DAG transform (read in WORLD space —
    // the node's parent group, {model}_Joints on import, does not matter);
    // the rest from attributes.  Returns
    // false when `node` is not a pmxRigidBodyJoint.
    static bool readJointDefinition(const MObject& node,
                                    mmd::core::RigidBodySimulation::JointDefinition& out);

    // ------------------------------------------------------------------
    // Attributes — the PMX joint data (mirrors the fields that used to live
    // in pmxRigidBodyNode's joints[] compound).  The joint FRAME (position +
    // orientation) is NOT an attribute — it is the node's DAG transform.
    // ------------------------------------------------------------------
    static MObject aSolver;             // message -> owning pmxRigidBodyNode (wired at import)
    static MObject aJointNameLocal;     // string — PMX name_local; "" = none
    static MObject aJointNameUniversal; // string — PMX name_universal; "" = none
    static MObject aJointBodyA;         // long — PMX rigid_body_index_a
    static MObject aJointBodyB;         // long — PMX rigid_body_index_b
    static MObject aJointType;          // enum — PMX joint type (JointType)
    static MObject aJointLinearMin;     // float3 — linear limit min (joint-frame offsets)
    static MObject aJointLinearMax;     // float3 — linear limit max (joint-frame offsets)
    static MObject aJointAngularMin;    // float3 — angular limit min (radians)
    static MObject aJointAngularMax;    // float3 — angular limit max (radians)
    static MObject aJointLinearSpring;  // float3 — linear spring constants
    static MObject aJointAngularSpring; // float3 — angular spring constants
    static MObject aDrawMode;           // enum — viewport draw mode (DrawMode)
};
