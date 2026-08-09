/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_physics_node.h
 *
 * MMDPhysicsNode — native rigid-body physics node for MMD secondary movement.
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
#include <maya/MTypeId.h>

#include <cstdint>
#include <memory>
#include <vector>

struct btCollisionConfiguration;
struct btCollisionDispatcher;
struct btBroadphaseInterface;
struct btConstraintSolver;
class btCollisionShape;
class btDiscreteDynamicsWorld;
class btRigidBody;

// ===========================================================================
// MMDPhysicsNode
// ===========================================================================
class MMDPhysicsNode : public MPxLocatorNode
{
  public:
    static const MTypeId kTypeId;
    static constexpr const char* kNodeName = "mmdPhysicsNode";
    // VP2 draw-database classification.  A drawable locator node must be
    // registered with the SAME classification string its draw override is
    // registered under ("drawdb/geometry/<nodeType>") or VP2 never associates
    // the override with the node and no guides are drawn.
    static constexpr const char* kNodeClassify = "drawdb/geometry/mmdPhysicsNode";

    // ------------------------------------------------------------------
    // Draw support (Phase 1) — the node draws its own guide visualization
    // ------------------------------------------------------------------
    // Per-body primitive pulled by the draw override
    // (mmd_physics_draw_override.cpp) from the node's CURRENT solver state:
    // solved world poses if the Bullet world is built, rest poses otherwise.
    // The node is an MPxLocatorNode so the guides are always visible, and the
    // viewport shows exactly what the simulation has — no scene guide meshes.
    struct DrawBody
    {
        double pos[3];
        double quat[4];     // (x, y, z, w)
        short colliderType; // 1 box, 2 sphere, 3 capsule
        double radius;
        double extents[3]; // box half extents
        double length;     // capsule cylinder length
        int groupId;       // collision group 0..15 (draw palette)
        bool kinematic;
    };
    // Fill *out* with one DrawBody per rigid body (body-index aligned).
    void collectDrawData(std::vector<DrawBody>& out) const;
    // Object-space bounding box over the body rest poses (selection/culling).
    MBoundingBox boundingBox() const override;

    MMDPhysicsNode();
    ~MMDPhysicsNode() override;

    // MPxNode overrides
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;
    void postConstructor() override;
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
    static MObject aFps; // playback frames per second (default 30)

    // Anchor world matrices + parent-inverse matrices (kinematic drivers) —
    // one per kinematic body.  The node computes each anchor's LOCAL matrix as
    // world * parentInverse so the Bullet world runs in the physics group's
    // local space (mirrors mayaBullet's inWorldMatrix/inParentInverseMatrix).
    // Phase 3: the anchor world is the JOINT's world matrix, the parent
    // inverse is the PHYSICS GROUP's world inverse, and `anchorOffset` is a
    // baked world-frame offset (bodyRestWorld * jointRestWorld^-1) so the
    // collider tracks the joint with the PMX body<->bone offset preserved.
    static MObject aAnchorWorldMatrix;
    static MObject aAnchorParentInverseMatrix;
    static MObject aAnchorOffset; // matrix array, kinematic-order indexed (Phase 3)

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
    // parentBodyRestWorld^-1 is baked by Python (the same constant for
    // kinematic and dynamic parents).
    static MObject aBodyParentJointOffset; // matrix array, body-indexed: M_parent baked constant
    // Per-body compound array: aBodies[i].
    static MObject aBodies;
    static MObject aBodyRestTranslate;  // float3 (degrees? no — translate units)
    static MObject aBodyRestRotate;     // float3 degrees
    static MObject aBodyMass;           // double
    static MObject aBodyLinearDamping;  // double
    static MObject aBodyAngularDamping; // double
    static MObject aBodyFriction;       // double
    static MObject aBodyRestitution;    // double
    static MObject aBodyColliderType;   // short: 1 box, 2 sphere, 3 capsule
    static MObject aBodyRadius;         // double (sphere/capsule)
    static MObject aBodyExtents;        // float3 (box half extents)
    static MObject aBodyLength;         // double (capsule)
    static MObject aBodyGroup;          // long collision group
    static MObject aBodyMask;           // long collision mask
    static MObject aBodyGroupId;        // short PMX group id 0..15 (-1 = explicit bodyGroup)
    static MObject
        aBodyNonCollisionGroup;    // long raw PMX non-collision mask (-1 = explicit bodyMask)
    static MObject aBodyKinematic; // bool — kinematic (anchor) vs dynamic
    static MObject
        aBodyPhysicsMode; // short — PMX physics mode 0/1/2 (FOLLOW_BONE/PHYSICS/PHYSICS_BONE)
    static MObject aBodyParentBodyIndex; // short — rigid-body index of the related
                                         // joint's parent joint's body (write-back parent-inverse
                                         // source); -1 = none
    static MObject
        aBodyResetAnchorIndex; // long — index of the kinematic
                               // anchor whose delta drives this body's scrub-back reset; -1 = none

    // Per-joint compound array: aJoints[j].
    static MObject aJoints;
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
    struct Body
    {
        double restPos[3];
        double restRot[3]; // degrees
        double mass;
        double linearDamping;
        double angularDamping;
        double friction;
        double restitution;
        short colliderType; // 1 box, 2 sphere, 3 capsule
        double radius;
        double extents[3];
        double length;
        long group;
        long mask;
        // Raw PMX collision inputs (Phase 2): the node derives the Bullet
        // group bit and the effective mask itself when these are set (>= 0);
        // `group`/`mask` above remain as explicit overrides (used otherwise).
        short groupId;
        long nonCollisionGroup;
        bool kinematic;
        short physicsMode = 1; // PMX physics mode 0/1/2 (write-back mode)
        // Write-back parent-inverse source (Phase 3 cycle fix): rigid-body
        // index of this body's related joint's PARENT joint's body, or -1 if
        // the parent bone has no body (the node then falls back to the DG
        // bodyParentInverseMatrix input — safe, that parent is never
        // node-driven).  The parent inverse is derived from the PARENT BODY's
        // solved Bullet transform so the write-back never depends on a
        // node-driven joint's DG matrix (which created a feedback cycle).
        int parentBodyIndex = -1;
        // Scrub-back reset: index of the kinematic ANCHOR whose current pose
        // drives this body's reset (or -1), plus the constant offset
        // (anchorRest^-1 * bodyRest) captured at build time.
        int resetAnchorIndex = -1;
        bool hasBoneReset = false;
        double resetOffsetPos[3] = {0.0, 0.0, 0.0};
        double resetOffsetQuat[4] = {0.0, 0.0, 0.0, 1.0};
    };

    struct Joint
    {
        long bodyA;
        long bodyB;
        long type; // PMX JointType value
        double frameT[3];
        double frameR[3]; // degrees
        double linearMin[3];
        double linearMax[3];
        double angularMin[3];
        double angularMax[3];
        double linearSpring[3];
        double angularSpring[3];
    };

    // Internal Bullet world + state
    std::unique_ptr<btDiscreteDynamicsWorld> mWorld;
    std::vector<std::unique_ptr<btRigidBody>> mRigidBodies;
    std::vector<std::unique_ptr<class btTypedConstraint>> mConstraints;
    // The world does NOT own its dispatcher / broadphase / collision config /
    // solver, and rigid bodies do NOT own their collision shapes — keep them
    // alive as members so they are freed exactly once (and so the world can be
    // destroyed before the bodies, which btCollisionWorld's destructor needs).
    std::unique_ptr<btCollisionConfiguration> mCollisionConfig;
    std::unique_ptr<btCollisionDispatcher> mDispatcher;
    std::unique_ptr<btBroadphaseInterface> mBroadphase;
    std::unique_ptr<btConstraintSolver> mConstraintSolver;
    std::vector<std::unique_ptr<btCollisionShape>> mShapes;
    std::vector<Body> mBodies;
    std::vector<Joint> mJoints;

    // Kinematic anchor poses (group-local), kept as pos+quat so no Bullet
    // type leaks into the header.  mAnchorRest is captured at build time;
    // mAnchorCurrent is refreshed every frame in updateKinematicAnchors and
    // used to reset dynamic bodies when time is scrubbed backwards.
    struct AnchorPose
    {
        double pos[3] = {0.0, 0.0, 0.0};
        double quat[4] = {0.0, 0.0, 0.0, 1.0};
    };
    std::vector<AnchorPose> mAnchorRest;
    std::vector<AnchorPose> mAnchorCurrent;

    bool mWorldBuilt = false;
    double mLastTime = -1.0;
    // Phase 4: FNV-1a hash of the config inputs (gravity/fps/bodies/joints/
    // anchor counts) captured at build time.  When compute() sees a different
    // signature the world is rebuilt in place — a body/joint/gravity edit takes
    // effect immediately, without a rewind.
    uint64_t mConfigSignature = 0;

    // Helpers
    bool readBodyData(MDataBlock& dataBlock);
    bool readJointData(MDataBlock& dataBlock);
    bool buildWorld(MDataBlock& dataBlock);
    // Phase 4: hash of all config inputs (the values that define the Bullet
    // world).  The anchor matrix VALUES are excluded — they change every frame
    // — only the anchor COUNTS are part of the signature.
    uint64_t computeConfigSignature(MDataBlock& dataBlock) const;
    void destroyWorld();
    // Refresh the kinematic anchor transforms from their inputs; returns true
    // if any anchor MOVED since the previous frame (used to step the sim when
    // a bone is dragged at a fixed time, without waiting for time to advance).
    bool updateKinematicAnchors(MDataBlock& dataBlock);
    // Teleport dynamic bodies to their related bone's CURRENT pose (used when
    // time is scrubbed backwards) — the opposite of rebuilding at rest.
    void resetDynamicBodies(MDataBlock& dataBlock);
    bool writeOutputs(MDataBlock& dataBlock);
};
