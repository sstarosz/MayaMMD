/*
 * SPDX-License-Identifier: MIT
 *
 * physics_node.h
 *
 * PhysicsNode — native rigid-body physics node for MMD secondary movement.
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
 * local pose to outTranslate[i]/outRotate[i] (which Python connects directly
 * into the related joints).  Registered by MayaMMD.mll.
 */

#pragma once

#include <maya/MBoundingBox.h>
#include <maya/MEvaluationNode.h>
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
// PhysicsNode
// ===========================================================================
class PhysicsNode : public MPxLocatorNode
{
  public:
    static const MTypeId kTypeId;
    static constexpr const char* kNodeName = "pmxPhysicsNode";
    // VP2 draw-database classification.  A drawable locator node must be
    // registered with the SAME classification string its draw override is
    // registered under ("drawdb/geometry/<nodeType>") or VP2 never associates
    // the override with the node and no guides are drawn.
    static constexpr const char* kNodeClassify = "drawdb/geometry/pmxPhysicsNode";

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

    // Viewport draw style for the guide visualization (drawMode attribute).
    // This is a VIEW attribute only — it never affects the Bullet world, so
    // it is deliberately NOT part of the node's config comparison (changing it
    // must not rebuild the simulation).
    enum DrawMode : short
    {
        kDrawOff = 0,               // no colliders drawn
        kDrawWireframe = 1,         // outline only (default)
        kDrawSolid = 2,             // filled primitives
        kDrawWireframeAndSolid = 3, // both
    };

    // ------------------------------------------------------------------
    // Draw support — per-body primitive data for the guide visualization.
    // The viewport draw override (physics_draw_override.cpp) pulls this from
    // the node's CURRENT solver state: solved world poses if the Bullet world
    // is built, rest poses otherwise.  The node is an MPxLocatorNode so a
    // default locator is drawn until the override is registered.
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

    PhysicsNode();
    // Defaulted out-of-line: the node is destroyed polymorphically through its
    // MPxNode base (Maya deletes it via the base pointer), and the default
    // teardown is exactly what we want — see the lifecycle notes in the cpp.
    ~PhysicsNode() override;

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

    // ── View-only draw attributes (never part of the config comparison) ──
    // These configure the viewport guide visualization; changing them must
    // NOT rebuild the Bullet world, so they are excluded from configChanged()
    // and have no attributeAffects on the outputs.
    static MObject aDrawMode;         // enum DrawMode — collider draw style
    static MObject aDrawOpacity;      // float 0..1 — transparency of solid draws
    static MObject
        aUiSelectedBodyIndex; // long — body the user picked in the viewport (-1 = none); written
                              // by the draw override's userSelect, read by the AE template

    // Kinematic anchors — one per FOLLOW_BONE body, in kinematic order.  The
    // anchor world is the related joint's world matrix; the node applies the
    // body<->joint rest offset (K^-1, derived from bodyWriteBackOffset) to
    // place the collider on its bone.  The Bullet world runs in WORLD space,
    // so the solver's own location never matters.
    static MObject aAnchorWorldMatrix;

    // Write-back inputs — the node outputs each dynamic body's JOINT-LOCAL
    // pose (boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1, with
    // K = jointRestWorld * bodyRestWorld^-1 and M_parent = K[parentBodyIndex]),
    // so Python connects outTranslate/outRotate straight into the joints.
    static MObject aBodyWriteBackOffset; // matrix array, body-indexed: K
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
    static MObject aBodyShapeSize;       // float3 — PMX shape_size verbatim
    static MObject aBodyRestTranslate;   // float3 — PMX shape_position (rest, world space)
    static MObject aBodyRestRotate;      // float3 — PMX shape_rotation (degrees)
    static MObject aBodyMass;            // double — PMX mass
    static MObject aBodyLinearDamping;   // double — PMX move_attenuation
    static MObject aBodyAngularDamping;  // double — PMX rotation_damping
    static MObject aBodyRestitution;     // double — PMX repulsion
    static MObject aBodyFriction;        // double — PMX friction_force
    static MObject aBodyPhysicsMode;     // enum — PMX physics_mode (PhysicsMode)
    static MObject aBodyParentBodyIndex; // long (wiring) — write-back parent body index; -1 = none
    static MObject
        aBodyResetAnchorIndex; // long (wiring) — kinematic anchor for scrub-back reset; -1 = none

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
    // gravity edit, or a changed anchor/write-back count, takes effect
    // immediately).  The anchor/write-back matrix VALUES are per-frame and are
    // read fresh every evaluation, so only their counts are cached here.
    mmd::core::Double3 mGravity = mmd::core::Double3();
    std::size_t mAnchorCount = 0;
    std::size_t mWriteBackOffsetCount = 0;

    // Helpers
    static std::vector<mmd::core::Simulation::BodyDefinition> readBodyData(MDataBlock& dataBlock);
    static std::vector<mmd::core::Simulation::JointDefinition> readJointData(MDataBlock& dataBlock);
    static mmd::core::Double3 readGravity(MDataBlock& dataBlock);
    static std::size_t arrayElementCount(MDataBlock& dataBlock, const MObject& attr);
    bool buildWorld(const mmd::core::Double3& gravity,
                    const std::vector<mmd::core::Simulation::BodyDefinition>& bodies,
                    const std::vector<mmd::core::Simulation::JointDefinition>& joints);
    // True when the fresh inputs differ from what the world was built with.
    bool configChanged(const std::vector<mmd::core::Simulation::BodyDefinition>& bodies,
                       const std::vector<mmd::core::Simulation::JointDefinition>& joints,
                       const mmd::core::Double3& gravity, std::size_t anchorCount,
                       std::size_t wbOffsetCount) const;
    // Remember the config the world was just built with.
    void storeConfig(const std::vector<mmd::core::Simulation::BodyDefinition>& bodies,
                     const std::vector<mmd::core::Simulation::JointDefinition>& joints,
                     const mmd::core::Double3& gravity, std::size_t anchorCount,
                     std::size_t wbOffsetCount);
    void destroyWorld();
    // Refresh the kinematic anchor transforms from their inputs; returns true
    // if any anchor moved since the previous evaluation (a dragged bone at a
    // fixed time steps the sim immediately).
    bool updateKinematicAnchors(MDataBlock& dataBlock);
    bool writeOutputs(MDataBlock& dataBlock);
};
