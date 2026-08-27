/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape.hpp
 *
 * RigidBodyShape — one selectable, movable Maya node per PMX rigid body
 * (Phase 2 viewport work).
 *
 * Each PMX rigid body becomes its own MPxLocatorNode under the model's
 * `{model}_RigidBodies` group.  The node is BOTH:
 *   1. a data holder — every PMX body field is a storable attribute (the
 *      solver reads them through its `bodyShapes[]` message array), and
 *   2. the viewport GUIDE — its transform is DRIVEN by the solver each frame
 *      to the body's CURRENT world pose, so the collider (drawn by the
 *      geometry override at the guide-local origin) follows the animation:
 *      kinematic
 *      bodies track their bone, dynamic bodies track the simulated pose.
 *      The body's REST pose lives in the `bodyRestTranslate` /
 *      `bodyRestRotate` attributes (world space, PMX-verbatim after the
 *      MMD→Maya flip) — deliberately NOT in the transform, because the
 *      transform is animated.
 *
 * The collider is drawn by a geometry override registered for this node's
 * classification (`drawdb/geometry/pmxRigidBodyShape`); draw mode
 * (none/wire/solid) is a per-node `drawMode` attribute so bodies can be
 * shown differently.  The node has no compute() — the solver pulls the
 * attributes.
 *
 * Registered by MayaMMD.mll.
 */

#pragma once

#include <maya/MBoundingBox.h>
#include <maya/MObject.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>

#include <array>

#include "rigid_body_simulation.hpp" // core BodyDefinition

// ===========================================================================
// RigidBodyShape
// ===========================================================================
class RigidBodyShape : public MPxLocatorNode
{
  public:
    static const MTypeId kTypeId;
    static constexpr const char* kNodeName = "pmxRigidBodyShape";
    // VP2 draw-database classification — the geometry override is registered
    // under this string.
    static constexpr const char* kNodeClassify = "drawdb/geometry/pmxRigidBodyShape";

    // PMX collider type — enum attribute values (persisted in scenes, so they
    // must never change).  Values match RigidBodyNode::ColliderType.
    enum ColliderType : short
    {
        kColliderBox = 1,
        kColliderSphere = 2,
        kColliderCapsule = 3,
    };

    // Viewport draw mode (0 = off, 1 = wire, 2 = solid, 3 = wire + solid).
    enum DrawMode : short
    {
        kDrawOff = 0,
        kDrawWire = 1,
        kDrawSolid = 2,
        kDrawWireSolid = 3,
    };

    RigidBodyShape();
    ~RigidBodyShape() override;

    // MPxNode overrides — data-holder only (no compute()).
    MBoundingBox boundingBox() const override; // object-space rest collider box

    // Registration helpers
    static void* creator();
    static MStatus initialize();

    // Read this node into a core BodyDefinition (used by the solver's
    // readBodyData — it pulls each body from its connected body shape node).
    // Returns false when `node` is not a pmxRigidBodyShape.
    static bool readBodyDefinition(const MObject& node,
                                   mmd::core::RigidBodySimulation::BodyDefinition& out);

    // ------------------------------------------------------------------
    // Attributes — the PMX rigid-body data, stored VERBATIM (mirrors the
    // fields that used to live in pmxRigidBodyNode's bodies[] compound).
    // ------------------------------------------------------------------
    static MObject aSolver;            // message -> owning pmxRigidBodyNode (wired at import)
    static MObject aBodyJoint;         // message <- related joint (its bone).  Drives the
                                       // FOLLOW_BONE kinematic-anchor binding and the
                                       // write-back parent/reset chain (resolved by the
                                       // solver via the joint DAG).
    static MObject aBodyAnchorWorld;   // matrix — kinematic-anchor input (joint.worldMatrix[0])
    static MObject aBodyEnabled;       // bool (custom) — disabled bodies are skipped by the solver
    static MObject aBodyNameLocal;     // string — PMX name_local; "" = none
    static MObject aBodyNameUniversal; // string — PMX name_universal; "" = none
    static MObject aBodyGroupId;       // enum — PMX group_id 0..15 ("Group 0".."Group 15")
    // THE collision mask — one bool per collision group (0..15), True = the
    // body collides with that group (PMX non_collision_group stored verbatim,
    // bit set = collides — the solver uses it exactly as read).
    static std::array<MObject, 16> aBodyMaskGroup;
    static MObject aBodyColliderType;   // enum — PMX shape (kColliderBox/Sphere/Capsule)
    static MObject aBodyShapeSize;      // float3 — PMX shape_size verbatim (box = half-extent)
    static MObject aBodyRestTranslate;  // float3 — rest position (world space, MMD→Maya)
    static MObject aBodyRestRotate;     // float3 — rest rotation (degrees, MMD→Maya)
    static MObject aBodyMass;           // double — PMX mass
    static MObject aBodyLinearDamping;  // double — PMX move_attenuation
    static MObject aBodyAngularDamping; // double — PMX rotation_damping
    static MObject aBodyRestitution;    // double — PMX repulsion
    static MObject aBodyFriction;       // double — PMX friction_force
    static MObject aBodyPhysicsMode;    // enum — PMX physics_mode (PhysicsMode)
    static MObject aDrawMode;           // enum — viewport draw mode (DrawMode)
};
