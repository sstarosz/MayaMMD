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
 * The node is an adapter over the Maya-free mmd::core::Simulation engine: it
 * reads the scene attributes into a Simulation::Definition, rebuilds the world
 * when those inputs change (or time is scrubbed backwards), steps it when time
 * advances or a kinematic anchor moves, and writes each dynamic body's solved
 * local pose to outTranslate[i]/outRotate[i] (which pmxRigidBody connects
 * directly into the related joints at create).  Registered by MayaMMD.mll.
 */

#pragma once

#include <maya/MBoundingBox.h>
#include <maya/MEvaluationNode.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MString.h>
#include <maya/MTime.h>
#include <maya/MTypeId.h>

#include <array>
#include <cstddef>
#include <vector>

#include "simulation.hpp"

// ===========================================================================
// RigidBodyNode
// ===========================================================================
class RigidBodyNode : public MPxLocatorNode
{
  public:
    static const MTypeId kTypeId;
    static constexpr const char* kNodeName = "pmxRigidBodyNode";
    // VP2 draw-database classification.  A drawable locator node must be
    // registered with the SAME classification string its draw override is
    // registered under ("drawdb/geometry/<nodeType>") or VP2 never associates
    // the override with the node and no guides are drawn.
    static constexpr const char* kNodeClassify = "drawdb/geometry/pmxRigidBodyNode";

    // PMX rigid-body physics mode — stored in the bodyPhysicsMode enum
    // attribute.  The attribute VALUES are mmd::core::Simulation::PhysicsMode
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

    // ------------------------------------------------------------------
    // Draw support — per-body primitive data for the guide visualization.
    // A viewport draw override (planned) pulls this from the node's CURRENT
    // solver state: solved world poses if the Bullet
    // world is built, rest poses otherwise.  The node is an MPxLocatorNode so
    // a default locator is drawn until the override lands.
    // ------------------------------------------------------------------
    struct DrawBody
    {
        double pos[3] = {};
        double quat[4] = {};                      // (x, y, z, w)
        ColliderType colliderType = kColliderBox; // PMX shape type (box/sphere/capsule)
        // PMX shape_size VERBATIM (3 doubles, FULL size — box extents are
        // full, not half).  This is the data contract the follow-up viewport
        // draw override reads; it is derived from the engine's
        // radius/extents/length by collider type via
        // mmd::core::shapeSizeFromBodyDefinition.
        double shapeSize[3] = {};
        int groupId = 0; // collision group 0..15 (draw palette)
        bool kinematic = false;
    };
    // Fill *out* with one DrawBody per rigid body (body-index aligned).
    void collectDrawData(std::vector<DrawBody>& out) const;
    // Object-space bounding box over the body rest poses (selection/culling).
    MBoundingBox boundingBox() const override;

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
    // (mmd::core::applyShapeSize) in readBodyData; the draw fallback reads it
    // verbatim.
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

  private:
    // Maya-free Bullet engine + the config it was built with (see
    // configChanged).  The engine owns all Bullet state; this node adapts
    // attributes <-> engine poses and manages the timeline.
    mmd::core::Simulation mSim;
    std::vector<mmd::core::Simulation::BodyDefinition> mBodies;
    std::vector<mmd::core::Simulation::JointDefinition> mJoints;

    double mLastTime = -1.0;
    MTime::Unit mLastTimeUnit = MTime::kFilm; // time unit of mLastTime (for dt)
    // The config the world was last built with — compute() re-reads the inputs
    // every evaluation and rebuilds in place when they differ (a body/joint/
    // gravity edit takes effect immediately).  The per-body write-back offset
    // K = jointRestWorld * bodyRestWorld^-1 is DERIVED only when the world is
    // (re)built — from the joints' pmxRest*/jointOrient attributes (static —
    // captured by the bone builder) plus the stored body rest pose — and
    // cached in mK for the per-frame anchor/write-back consumers.  The anchor
    // matrix VALUES are per-frame (bodies[i].bodyAnchorWorld, read fresh in
    // updateKinematicAnchors).
    mmd::core::Double3 mGravity = mmd::core::Double3();
    std::vector<MMatrix> mK; // derived per build; identity for no-joint bodies

    // Helpers
    std::vector<mmd::core::Simulation::BodyDefinition> readBodyData(MDataBlock& dataBlock);
    // Derive the per-body write-back offsets (K) from the joints' rest data
    // and cache them in mK.  Called only when the world is (re)built — the
    // inputs are static (pmxRest*/jointOrient + body rest pose), so deriving
    // per evaluation would waste a DAG walk per body per frame.
    void deriveWriteBackOffsets(const std::vector<mmd::core::Simulation::BodyDefinition>& bodies);
    static std::vector<mmd::core::Simulation::JointDefinition> readJointData(MDataBlock& dataBlock);
    static mmd::core::Double3 readGravity(MDataBlock& dataBlock);
    bool buildWorld(const mmd::core::Double3& gravity,
                    const std::vector<mmd::core::Simulation::BodyDefinition>& bodies,
                    const std::vector<mmd::core::Simulation::JointDefinition>& joints);
    // True when the fresh inputs differ from what the world was built with.
    bool configChanged(const std::vector<mmd::core::Simulation::BodyDefinition>& bodies,
                       const std::vector<mmd::core::Simulation::JointDefinition>& joints,
                       const mmd::core::Double3& gravity) const;
    // Remember the config the world was just built with.
    void storeConfig(const std::vector<mmd::core::Simulation::BodyDefinition>& bodies,
                     const std::vector<mmd::core::Simulation::JointDefinition>& joints,
                     const mmd::core::Double3& gravity);
    void destroyWorld();
    // Refresh the kinematic anchor transforms from their inputs; returns true
    // if any anchor moved since the previous evaluation (a dragged bone at a
    // fixed time steps the sim immediately).
    bool updateKinematicAnchors(MDataBlock& dataBlock);
    bool writeOutputs(MDataBlock& dataBlock);
};
