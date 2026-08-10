/*
 * SPDX-License-Identifier: MIT
 *
 * physics_node.h
 *
 * PhysicsNode — native rigid-body physics node for MMD secondary movement.
 *
 * WHY THIS EXISTS (replaces the mayaBullet dynamic layer):
 *   mayaBullet's bulletSolverShape is a STATEFUL node.  Cached Playback's
 *   evaluation cache treats node outputs as pure functions of their inputs and
 *   does not re-evaluate / step stateful nodes, so mayaBullet dynamic bodies
 *   froze at rest under Cached Playback (and the write-back constraints then
 *   locked the skeleton — "lost mesh binding").
 *
 *   This node is a normal MPxNode: it owns a Bullet (btDiscreteDynamicsWorld)
 *   world internally, is driven by `time1.outTime`, and is evaluated by the
 *   evaluation manager on every time step exactly like a parentConstraint —
 *   the mechanism that is proven to work under Cached Playback.  No scriptJob,
 *   no external solver plugin, no stateful third-party node.
 *
 * DATA FLOW
 *   - Inputs:  `time`, `gravity`, an array of ANCHOR world matrices (the
 *     FOLLOW_BONE guides / animated bones that kinematically drive the
 *     chains), a per-body compound array (rest pose, mass, damping, collider,
 *     group/mask, kinematic flag), and a per-joint compound array (type,
 *     frame, limits, spring constants).
 *   - Compute: updates kinematic (anchor) bodies from the anchor matrices,
 *     steps the Bullet world, and writes each dynamic body's solved LOCAL
 *     translate/rotate to the outputs.
 *   - Outputs: `outTranslate[i]` / `outRotate[i]` (float3, Maya degrees) which
 *     Python connects to the dynamic guide transforms (guide → parentConstraint
 *     → bone, exactly like the previous write-back).
 *
 * Registered natively by MayaMMD.mll's initializePlugin.
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
#include <cstdint>
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

    // ------------------------------------------------------------------
    // Draw support — per-body primitive data for the guide visualization.
    // A viewport draw override (planned, redesigned in a later PR) pulls this
    // from the node's CURRENT solver state: solved world poses if the Bullet
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
    // Hidden forced-rebuild trigger.  dt is derived from the scene's time unit
    // via MTime, so the old `fps` attribute (which only ever served as the
    // rebuild trigger) is gone.  Bumping configVersion changes the config
    // signature, so compute() rebuilds the Bullet world even when no other
    // input changed (e.g. after the Python side re-bakes the anchor/write-back
    // offsets).  The default is 0 — an untouched scene never forces a rebuild.
    static MObject aConfigVersion;

    // Anchor world matrices (kinematic drivers) — one per kinematic body, in
    // kinematic (FOLLOW_BONE body) order.  The node computes each anchor's
    // LOCAL matrix as world * groupInverseWorldMatrix so the Bullet world runs
    // in the physics group's local space (mirrors mayaBullet's
    // inWorldMatrix/inParentInverseMatrix).  Phase 3: the anchor world is the
    // JOINT's world matrix, the group inverse is the PHYSICS GROUP's world
    // inverse, and `anchorOffset` is a baked world-frame offset
    // (bodyRestWorld * jointRestWorld^-1) so the collider tracks the joint
    // with the PMX body<->bone offset preserved.
    static MObject aAnchorWorldMatrix;
    static MObject aGroupInverseWorldMatrix; // physics group's world inverse (single)
    static MObject aAnchorOffset;            // matrix array, kinematic-order indexed (Phase 3)

    // Phase 3 direct write-back inputs — the node outputs the JOINT-LOCAL
    // pose directly (boneLocal = K * bodyLocal * groupWorld * parentInverse),
    // so Python connects outTranslate/outRotate straight into the joints and
    // the guide transforms + write-back constraints are gone.
    static MObject aGroupWorldMatrix; // physics group's world matrix (single)
    static MObject
        aBodyWriteBackOffset; // matrix array, body-indexed: K = jointRestWorld * bodyRestWorld^-1
    static MObject
        aBodyParentInverseMatrix; // matrix array, body-indexed: related joint's parentInverseMatrix
                                  // (DG fallback, no-body parent only)
    // Phase 3 cycle fix: the parent inverse for the write-back is derived from
    // the PARENT BODY's solved Bullet transform instead of the DG
    // (boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1).  This removes
    // the dependency on `joint.parentInverseMatrix` — which for a body whose
    // parent JOINT is also node-driven created a DG feedback cycle that
    // exploded the simulation.  M_parent = parentJointRestWorld *
    // parentBodyRestWorld^-1 is the SAME constant as K[parentBodyIndex]
    // (bodyWriteBackOffset of the parent body, for kinematic and dynamic
    // parents) — so no separate parent-offset array exists.
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
    // (mmd::core::applyShapeSize) in readBodyData, computeConfigSignature and
    // the attribute-fallback reader.
    static MObject aBodyShapeSize;       // float3 — PMX shape_size verbatim
    static MObject aBodyRestTranslate;   // float3 — PMX shape_position (rest, group space)
    static MObject aBodyRestRotate;      // float3 — PMX shape_rotation (degrees)
    static MObject aBodyMass;            // double — PMX mass
    static MObject aBodyLinearDamping;   // double — PMX move_attenuation
    static MObject aBodyAngularDamping;  // double — PMX rotation_damping
    static MObject aBodyRestitution;     // double — PMX repulsion
    static MObject aBodyFriction;        // double — PMX friction_force
    static MObject aBodyPhysicsMode;     // enum — PMX physics_mode (PhysicsMode)
    static MObject aBodyParentBodyIndex; // short (wiring) — write-back parent body index; -1 = none
    static MObject
        aBodyResetAnchorIndex; // long (wiring) — kinematic anchor for scrub-back reset; -1 = none

    // Per-joint compound array: aJoints[j].
    static MObject aJoints;
    static MObject aJointNameLocal;      // string — PMX name_local; "" = none
    static MObject aJointNameUniversal;  // string — PMX name_universal; "" = none
    static MObject aJointBodyA;          // long
    static MObject aJointBodyB;          // long
    static MObject aJointType;           // long 0..5 (PMX JointType)
    static MObject aJointFrameTranslate; // float3
    static MObject aJointFrameRotate;    // float3 degrees
    static MObject aJointLinearMin;      // float3
    static MObject aJointLinearMax;      // float3
    static MObject aJointAngularMin;     // float3
    static MObject aJointAngularMax;     // float3
    static MObject aJointLinearSpring;   // float3
    static MObject aJointAngularSpring;  // float3

    // Outputs: solved local translate/rotate per body (float3 array).
    static MObject aOutTranslate;
    static MObject aOutTranslateValue; // float3 child of aOutTranslate
    static MObject aOutRotate;
    static MObject aOutRotateValue; // float3 child of aOutRotate

  private:
    // Maya-free Bullet engine (simulation.hpp) + the PMX body/joint data
    // read from the attributes.  The engine owns ALL Bullet state and the
    // scrub-back reset; this node is an adapter — it reads attributes,
    // converts Maya matrices <-> engine poses, and manages the timeline/state
    // (config signature, last time).
    mmd::core::Simulation mSim;
    std::vector<mmd::core::Simulation::BodyDefinition> mBodies;
    std::vector<mmd::core::Simulation::JointDefinition> mJoints;

    double mLastTime = -1.0;
    MTime::Unit mLastTimeUnit = MTime::kFilm; // time unit of mLastTime (for dt)
    // Phase 4: FNV-1a hash of the config inputs (gravity/configVersion/
    // bodies/joints/anchor counts) captured at build time.  When compute() sees
    // a different signature the world is rebuilt in place — a body/joint/
    // gravity/configVersion edit takes effect immediately, without a rewind.
    uint64_t mConfigSignature = 0;

    // Timeline/state machine — compute() classifies each evaluation into one of
    // these transitions and acts on it (see compute()): the sim is built once,
    // rebuilt when a config input changes or time is scrubbed backwards, and
    // stepped when time advances or a kinematic anchor moves.
    enum class SimulationTransition
    {
        Initialize,           // world not built yet — build from scratch
        ConfigurationChanged, // a config input changed — rebuild at the current pose
        Rewind,               // time scrubbed backwards — rebuild at the current pose
        Advance,              // time advanced — step by the frame span
        PoseChanged,          // time unchanged but an anchor moved — step one tick
        NoChange,             // nothing to do
    };

    // Helpers
    bool readBodyData(MDataBlock& dataBlock);
    bool readJointData(MDataBlock& dataBlock);
    bool buildWorld(MDataBlock& dataBlock);
    // Phase 4: hash of all config inputs (the values that define the Bullet
    // world).  The anchor matrix VALUES are excluded — they change every frame
    // — only the anchor COUNTS are part of the signature.
    static uint64_t computeConfigSignature(MDataBlock& dataBlock);
    void destroyWorld();
    // Refresh the kinematic anchor transforms from their inputs; returns true
    // if any anchor MOVED since the previous frame (used to step the sim when
    // a bone is dragged at a fixed time, without waiting for time to advance).
    bool updateKinematicAnchors(MDataBlock& dataBlock);
    // Teleport dynamic bodies to their related bone's CURRENT pose (used when
    // time is scrubbed backwards) — the opposite of rebuilding at rest.
    void resetDynamicBodies(MDataBlock& dataBlock);
    bool writeOutputs(MDataBlock& dataBlock);

    // Timeline/state helpers (see compute()).
    SimulationTransition classifyTransition(uint64_t configSignature, const MTime& nowTime,
                                            double now, bool anchorsMoved) const;
    bool initializeSimulation(MDataBlock& dataBlock, uint64_t configSignature,
                              const MTime& nowTime);
    bool rebuildSimulationAtCurrentPose(MDataBlock& dataBlock, uint64_t configSignature,
                                        const MTime& nowTime);
};
