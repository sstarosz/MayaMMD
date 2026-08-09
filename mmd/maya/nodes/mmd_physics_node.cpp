/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_physics_node.cpp
 *
 * MMDPhysicsNode — native rigid-body physics node (embedded Bullet 3.25).
 *
 * See mmd_physics_node.h for the design rationale (replaces the mayaBullet
 * dynamic layer which froze under Cached Playback because its solver is a
 * stateful node the evaluation cache does not re-step).
 *
 * The Bullet world lives inside this node and advances in compute() every time
 * `time1.outTime` changes — the same evaluation path as a parentConstraint, so
 * it runs under Cached Playback / the evaluation manager.
 */

#include "mmd_physics_node.h"

#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnData.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MNodeCacheDisablingInfo.h>
#include <maya/MNodeCacheDisablingInfoHelper.h>
#include <maya/MNodeCacheSetupInfo.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MQuaternion.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>

#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include "mmd_physics_masks.h"
#include "mmd_physics_math.h"

#include <algorithm>
#include <cmath>
#include <cstring>

// The pure math (Euler <-> quaternion, row/column matrix transpose) lives in
// the Maya-free mmd_physics_math.h so it can be unit-tested without the Maya
// SDK — see tests/test_mmd_physics_math.cpp.
using namespace mmd_physics_math;

// ===========================================================================
// Constants
// ===========================================================================
const MTypeId MMDPhysicsNode::kTypeId(0x0011C105); // unique Maya node type id for mmdPhysicsNode

// PMX JointType -> Bullet constraint selection
namespace
{
constexpr int kJointSpring6Dof = 0;
constexpr int kJointSixDof = 1;
constexpr int kJointP2P = 2;
constexpr int kJointConeTwist = 3;
constexpr int kJointSlider = 4;
constexpr int kJointHinge = 5;

// Simulation stepping constants (see buildWorld()/compute()).
constexpr double kFixedDt = 1.0 / 60.0; // MMD physics tick
constexpr int kSolverIterations = 30;   // > Bullet's default 10 — long rigid chains need it
constexpr int kMaxSubSteps = 8;         // max internal steps per compute()
constexpr double kMaxStepTime = 0.5;    // clamp for huge time jumps (scrub/tab)
} // namespace

// ===========================================================================
// Attribute declarations
// ===========================================================================
MObject MMDPhysicsNode::aTime;
MObject MMDPhysicsNode::aGravity;
MObject MMDPhysicsNode::aFps;
MObject MMDPhysicsNode::aAnchorWorldMatrix;
MObject MMDPhysicsNode::aAnchorParentInverseMatrix;
MObject MMDPhysicsNode::aAnchorOffset;
MObject MMDPhysicsNode::aGroupWorldMatrix;
MObject MMDPhysicsNode::aBodyWriteBackOffset;
MObject MMDPhysicsNode::aBodyParentInverseMatrix;
MObject MMDPhysicsNode::aBodyParentJointOffset;

MObject MMDPhysicsNode::aBodies;
MObject MMDPhysicsNode::aBodyRestTranslate;
MObject MMDPhysicsNode::aBodyRestRotate;
MObject MMDPhysicsNode::aBodyMass;
MObject MMDPhysicsNode::aBodyLinearDamping;
MObject MMDPhysicsNode::aBodyAngularDamping;
MObject MMDPhysicsNode::aBodyFriction;
MObject MMDPhysicsNode::aBodyRestitution;
MObject MMDPhysicsNode::aBodyColliderType;
MObject MMDPhysicsNode::aBodyRadius;
MObject MMDPhysicsNode::aBodyExtents;
MObject MMDPhysicsNode::aBodyLength;
MObject MMDPhysicsNode::aBodyMask;
MObject MMDPhysicsNode::aBodyGroupId;
MObject MMDPhysicsNode::aBodyNonCollisionGroup;
MObject MMDPhysicsNode::aBodyKinematic;
MObject MMDPhysicsNode::aBodyPhysicsMode;
MObject MMDPhysicsNode::aBodyParentBodyIndex;
MObject MMDPhysicsNode::aBodyResetAnchorIndex;
MObject MMDPhysicsNode::aBodyNameLocal;
MObject MMDPhysicsNode::aBodyNameUniversal;
MObject MMDPhysicsNode::aBodyEnabled;

MObject MMDPhysicsNode::aJoints;
MObject MMDPhysicsNode::aJointBodyA;
MObject MMDPhysicsNode::aJointBodyB;
MObject MMDPhysicsNode::aJointType;
MObject MMDPhysicsNode::aJointFrameTranslate;
MObject MMDPhysicsNode::aJointFrameRotate;
MObject MMDPhysicsNode::aJointLinearMin;
MObject MMDPhysicsNode::aJointLinearMax;
MObject MMDPhysicsNode::aJointAngularMin;
MObject MMDPhysicsNode::aJointAngularMax;
MObject MMDPhysicsNode::aJointLinearSpring;
MObject MMDPhysicsNode::aJointAngularSpring;

MObject MMDPhysicsNode::aOutTranslate;
MObject MMDPhysicsNode::aOutTranslateValue;
MObject MMDPhysicsNode::aOutRotate;
MObject MMDPhysicsNode::aOutRotateValue;

// ===========================================================================
// Maya-specific conversion (the shared pure math is in mmd_physics_math.h)
// ===========================================================================
namespace
{

btTransform mayaMatrixToBtTransform(const MMatrix& m)
{
    // Maya matrices are ROW-vector (p' = p * M): row r holds the image of the
    // r-th basis vector and m(3, 0..2) is the translation.  Bullet uses
    // COLUMN-vector matrices (v' = M * v), so the same orientation's matrix is
    // the TRANSPOSE of Maya's.  Copying the row matrix directly (as done
    // before) gave every rotated anchor a transposed — i.e. wrong — basis,
    // which yanked the attached rigid chains into a mess.  The transpose
    // itself is the (unit-tested) mmd_physics_math::doubleMatrixToBtTransform;
    // this wrapper only adapts MMatrix's accessor.
    double mm[4][4];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            mm[r][c] = m(r, c);
    return doubleMatrixToBtTransform(mm);
}

// Read one body element's attributes into a DrawBody.  Used by
// collectDrawData to draw the REST guides even before the first compute()
// (mBodies is only filled lazily on first evaluation) — so the colliders are
// visible immediately after import and whenever the solver is not being pulled
// by the DG.
void readDrawBodyFromPlug(const MPlug& el, MMDPhysicsNode::DrawBody& db)
{
    db.colliderType = static_cast<short>(el.child(MMDPhysicsNode::aBodyColliderType).asShort());
    db.radius = el.child(MMDPhysicsNode::aBodyRadius).asDouble();
    const double* e = el.child(MMDPhysicsNode::aBodyExtents).asMDataHandle().asDouble3();
    db.extents[0] = e[0];
    db.extents[1] = e[1];
    db.extents[2] = e[2];
    db.length = el.child(MMDPhysicsNode::aBodyLength).asDouble();
    db.kinematic = el.child(MMDPhysicsNode::aBodyKinematic).asBool();
    // group id straight from the raw PMX id (the Bullet group bit is derived
    // from it in buildWorld); clamp legacy scenes where it is -1.
    db.groupId = el.child(MMDPhysicsNode::aBodyGroupId).asShort();
    if (db.groupId < 0)
        db.groupId = 0;
    const double* p = el.child(MMDPhysicsNode::aBodyRestTranslate).asMDataHandle().asDouble3();
    db.pos[0] = p[0];
    db.pos[1] = p[1];
    db.pos[2] = p[2];
    const double* r = el.child(MMDPhysicsNode::aBodyRestRotate).asMDataHandle().asDouble3();
    const btQuaternion q = eulerDegreesToQuat(r[0], r[1], r[2]);
    db.quat[0] = q.x();
    db.quat[1] = q.y();
    db.quat[2] = q.z();
    db.quat[3] = q.w();
}

} // namespace

// ===========================================================================
// Config signature hashing (Phase 4)
// ===========================================================================
namespace
{

// FNV-1a 64-bit — order-sensitive, cheap, good enough to detect any config
// edit (we don't need collision resistance, just change detection).
uint64_t fnv1aBytes(uint64_t h, const void* data, size_t len)
{
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < len; ++i)
    {
        h ^= p[i];
        h *= 0x100000001b3ULL;
    }
    return h;
}

template <typename T> uint64_t hashValue(uint64_t h, const T& v)
{
    return fnv1aBytes(h, &v, sizeof(v));
}

uint64_t hashDouble3(uint64_t h, const double v[3])
{
    h = hashValue(h, v[0]);
    h = hashValue(h, v[1]);
    return hashValue(h, v[2]);
}

} // namespace

// ===========================================================================
// Node lifecycle
// ===========================================================================
MMDPhysicsNode::MMDPhysicsNode() = default;

MMDPhysicsNode::~MMDPhysicsNode()
{
    destroyWorld();
}

void MMDPhysicsNode::postConstructor()
{
    // Nothing extra needed — Bullet world is built lazily on first compute.
}

void* MMDPhysicsNode::creator()
{
    return new MMDPhysicsNode();
}

void MMDPhysicsNode::destroyWorld()
{
    // CRITICAL teardown order: destroy the WORLD first while every body and
    // constraint is still alive.  btCollisionWorld's destructor iterates
    // m_collisionObjects and calls getBroadphase()->destroyProxy() on each —
    // if the bodies were already freed that is a use-after-free (access
    // violation, seen as intermittent Maya crashes during scene teardown).
    mWorld.reset(); // base dtor cleans up broadphase proxies on live bodies
    mConstraints.clear();
    mRigidBodies.clear();
    mShapes.clear();
    mConstraintSolver.reset();
    mBroadphase.reset();
    mDispatcher.reset();
    mCollisionConfig.reset();
    mWorldBuilt = false;
    mLastTime = -1.0;
    mLastTimeUnit = MTime::kFilm;
    mBodies.clear();
    mJoints.clear();
}

// ===========================================================================
// Attribute registration
// ===========================================================================
MStatus MMDPhysicsNode::initialize()
{
    MFnNumericAttribute nAttr;
    MFnCompoundAttribute cAttr;
    MFnMatrixAttribute mAttr;
    MFnUnitAttribute uAttr;
    MStatus stat;

    // --- time ---
    aTime = uAttr.create("time", "tm", MFnUnitAttribute::kTime, 0.0, &stat);
    CHECK_MSTATUS(stat);
    uAttr.setStorable(true);
    uAttr.setKeyable(true);
    uAttr.setHidden(true);

    // --- gravity ---
    aGravity = nAttr.create("gravity", "grav", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    nAttr.setDefault(0.0, -9.8, 0.0); // MMD's physics engine uses exactly -9.8
    nAttr.setStorable(true);
    nAttr.setKeyable(false);

    // --- fps ---
    aFps = nAttr.create("fps", "fps", MFnNumericData::kDouble, 30.0, &stat);
    CHECK_MSTATUS(stat);
    nAttr.setStorable(true);
    nAttr.setMin(1.0);
    nAttr.setKeyable(false);

    // --- anchor world matrices ---
    aAnchorWorldMatrix =
        mAttr.create("anchorWorldMatrix", "awm", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    aAnchorParentInverseMatrix =
        mAttr.create("anchorParentInverseMatrix", "apim", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    // Phase 3: baked world-frame offset per kinematic anchor
    // (colliderRestWorld * jointRestWorld^-1) so the collider tracks the JOINT
    // with the PMX body<->bone offset preserved.  Indexed by kinematic order,
    // 1:1 with anchorWorldMatrix.
    aAnchorOffset = mAttr.create("anchorOffset", "aof", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    // Phase 3 direct write-back: the physics group's world matrix (single) and
    // the per-dynamic-body baked offset + related-joint parent inverse.  These
    // are TOP-LEVEL matrix arrays (compound matrix children are awkward), so
    // the node reads them by body index in writeOutputs.
    aGroupWorldMatrix = mAttr.create("groupWorldMatrix", "gwm", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setKeyable(false);

    aBodyWriteBackOffset =
        mAttr.create("bodyWriteBackOffset", "bwo", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    aBodyParentInverseMatrix =
        mAttr.create("bodyParentInverseMatrix", "bpim", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    // Phase 3 cycle fix: baked parent-joint world offset per dynamic body
    // (M_parent = parentJointRestWorld * parentBodyRestWorld^-1).  The node
    // derives the parent joint's world from the PARENT BODY's solved Bullet
    // transform (M_parent * B_parent * groupWorld) instead of reading the DG
    // `joint.parentInverseMatrix` — which for a body whose parent JOINT is
    // also node-driven created a DG feedback cycle that exploded the sim.
    aBodyParentJointOffset =
        mAttr.create("bodyParentJointOffset", "bpjo", MFnMatrixAttribute::kDouble, &stat);
    CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    // --- body compound ---
    aBodyRestTranslate =
        nAttr.create("bodyRestTranslate", "brt", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aBodyRestRotate = nAttr.create("bodyRestRotate", "brr", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aBodyMass = nAttr.create("bodyMass", "bm", MFnNumericData::kDouble, 1.0, &stat);
    CHECK_MSTATUS(stat);
    aBodyLinearDamping =
        nAttr.create("bodyLinearDamping", "bld", MFnNumericData::kDouble, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aBodyAngularDamping =
        nAttr.create("bodyAngularDamping", "bad", MFnNumericData::kDouble, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aBodyFriction = nAttr.create("bodyFriction", "bfr", MFnNumericData::kDouble, 0.5, &stat);
    CHECK_MSTATUS(stat);
    aBodyRestitution = nAttr.create("bodyRestitution", "bre", MFnNumericData::kDouble, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aBodyColliderType =
        nAttr.create("bodyColliderType", "bct", MFnNumericData::kShort, kColliderBox, &stat);
    CHECK_MSTATUS(stat);
    aBodyRadius = nAttr.create("bodyRadius", "brad", MFnNumericData::kDouble, 0.5, &stat);
    CHECK_MSTATUS(stat);
    aBodyExtents = nAttr.create("bodyExtents", "bext", MFnNumericData::k3Double, 1.0, &stat);
    CHECK_MSTATUS(stat);
    aBodyLength = nAttr.create("bodyLength", "blen", MFnNumericData::kDouble, 1.0, &stat);
    CHECK_MSTATUS(stat);
    aBodyMask = nAttr.create("bodyMask", "bmk", MFnNumericData::kLong, 0xFFFF, &stat);
    CHECK_MSTATUS(stat);
    // Raw PMX collision inputs: the node derives the Bullet group bit from
    // bodyGroupId, and the effective mask itself when nonCollisionGroup >= 0;
    // bodyMask stays as an explicit override used otherwise (legacy scenes).
    aBodyGroupId = nAttr.create("bodyGroupId", "bgid", MFnNumericData::kShort, -1, &stat);
    CHECK_MSTATUS(stat);
    aBodyNonCollisionGroup =
        nAttr.create("bodyNonCollisionGroup", "bncg", MFnNumericData::kLong, -1, &stat);
    CHECK_MSTATUS(stat);
    aBodyKinematic = nAttr.create("bodyKinematic", "bkn", MFnNumericData::kBoolean, false, &stat);
    CHECK_MSTATUS(stat);
    // PMX physics mode: 0 FOLLOW_BONE, 1 PHYSICS, 2 PHYSICS_BONE.  The node
    // writes the joint-local pose for mode 1/2 (mode 2 = rotation only — Python
    // connects only outRotate for those bodies).
    aBodyPhysicsMode = nAttr.create("bodyPhysicsMode", "bpm", MFnNumericData::kShort, 1, &stat);
    CHECK_MSTATUS(stat);
    // Rigid-body index of the related joint's PARENT joint's body (the
    // write-back derives the parent inverse from that body's solved Bullet
    // transform); -1 = parent bone has no body (DG parentInverse fallback).
    aBodyParentBodyIndex =
        nAttr.create("bodyParentBodyIndex", "bpbi", MFnNumericData::kShort, -1, &stat);
    CHECK_MSTATUS(stat);
    aBodyResetAnchorIndex =
        nAttr.create("bodyResetAnchorIndex", "brai", MFnNumericData::kLong, -1, &stat);
    CHECK_MSTATUS(stat);
    // PMX body names (local + universal — Query/UI display; the node itself
    // never reads them, they just need to be storable attributes) and the
    // enabled flag (Remove support — disabled bodies are skipped by
    // buildWorld).
    MFnTypedAttribute tAttr;
    aBodyNameLocal =
        tAttr.create("bodyNameLocal", "bnml", MFnData::kString, MObject::kNullObj, &stat);
    CHECK_MSTATUS(stat);
    tAttr.setStorable(true);
    tAttr.setKeyable(false);
    aBodyNameUniversal =
        tAttr.create("bodyNameUniversal", "bnmu", MFnData::kString, MObject::kNullObj, &stat);
    CHECK_MSTATUS(stat);
    tAttr.setStorable(true);
    tAttr.setKeyable(false);
    aBodyEnabled = nAttr.create("bodyEnabled", "ben", MFnNumericData::kBoolean, true, &stat);
    CHECK_MSTATUS(stat);

    for (MObject* a : {&aBodyRestTranslate, &aBodyRestRotate, &aBodyMass, &aBodyLinearDamping,
                       &aBodyAngularDamping, &aBodyFriction, &aBodyRestitution, &aBodyColliderType,
                       &aBodyRadius, &aBodyExtents, &aBodyLength, &aBodyMask, &aBodyGroupId,
                       &aBodyNonCollisionGroup, &aBodyKinematic, &aBodyPhysicsMode,
                       &aBodyParentBodyIndex, &aBodyResetAnchorIndex, &aBodyEnabled})
    {
        MFnNumericAttribute fn(*a);
        fn.setStorable(true);
        fn.setKeyable(false);
    }

    aBodies = cAttr.create("bodies", "bds", &stat);
    CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setStorable(true);
    cAttr.setKeyable(false);
    cAttr.addChild(aBodyRestTranslate);
    cAttr.addChild(aBodyRestRotate);
    cAttr.addChild(aBodyMass);
    cAttr.addChild(aBodyLinearDamping);
    cAttr.addChild(aBodyAngularDamping);
    cAttr.addChild(aBodyFriction);
    cAttr.addChild(aBodyRestitution);
    cAttr.addChild(aBodyColliderType);
    cAttr.addChild(aBodyRadius);
    cAttr.addChild(aBodyExtents);
    cAttr.addChild(aBodyLength);
    cAttr.addChild(aBodyMask);
    cAttr.addChild(aBodyGroupId);
    cAttr.addChild(aBodyNonCollisionGroup);
    cAttr.addChild(aBodyKinematic);
    cAttr.addChild(aBodyPhysicsMode);
    cAttr.addChild(aBodyParentBodyIndex);
    cAttr.addChild(aBodyResetAnchorIndex);
    cAttr.addChild(aBodyNameLocal);
    cAttr.addChild(aBodyNameUniversal);
    cAttr.addChild(aBodyEnabled);

    // --- joint compound ---
    aJointBodyA = nAttr.create("jointBodyA", "jba", MFnNumericData::kLong, 0, &stat);
    CHECK_MSTATUS(stat);
    aJointBodyB = nAttr.create("jointBodyB", "jbb", MFnNumericData::kLong, 0, &stat);
    CHECK_MSTATUS(stat);
    aJointType = nAttr.create("jointType", "jt", MFnNumericData::kLong, 0, &stat);
    CHECK_MSTATUS(stat);
    aJointFrameTranslate =
        nAttr.create("jointFrameTranslate", "jft", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aJointFrameRotate =
        nAttr.create("jointFrameRotate", "jfr", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aJointLinearMin = nAttr.create("jointLinearMin", "jlmn", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aJointLinearMax = nAttr.create("jointLinearMax", "jlmx", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aJointAngularMin =
        nAttr.create("jointAngularMin", "jamn", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aJointAngularMax =
        nAttr.create("jointAngularMax", "jamx", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aJointLinearSpring =
        nAttr.create("jointLinearSpring", "jls", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    aJointAngularSpring =
        nAttr.create("jointAngularSpring", "jas", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);

    for (MObject* a : {&aJointBodyA, &aJointBodyB, &aJointType, &aJointFrameTranslate,
                       &aJointFrameRotate, &aJointLinearMin, &aJointLinearMax, &aJointAngularMin,
                       &aJointAngularMax, &aJointLinearSpring, &aJointAngularSpring})
    {
        MFnNumericAttribute fn(*a);
        fn.setStorable(true);
        fn.setKeyable(false);
    }

    aJoints = cAttr.create("joints", "jns", &stat);
    CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setStorable(true);
    cAttr.setKeyable(false);
    cAttr.addChild(aJointBodyA);
    cAttr.addChild(aJointBodyB);
    cAttr.addChild(aJointType);
    cAttr.addChild(aJointFrameTranslate);
    cAttr.addChild(aJointFrameRotate);
    cAttr.addChild(aJointLinearMin);
    cAttr.addChild(aJointLinearMax);
    cAttr.addChild(aJointAngularMin);
    cAttr.addChild(aJointAngularMax);
    cAttr.addChild(aJointLinearSpring);
    cAttr.addChild(aJointAngularSpring);

    // --- outputs ---
    aOutTranslateValue =
        nAttr.create("outTranslateValue", "otv", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    MFnNumericAttribute otFn(aOutTranslateValue);
    otFn.setWritable(false);
    otFn.setStorable(false);

    aOutTranslate = cAttr.create("outTranslate", "otr", &stat);
    CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutTranslateValue);

    aOutRotateValue = nAttr.create("outRotateValue", "orv", MFnNumericData::k3Double, 0.0, &stat);
    CHECK_MSTATUS(stat);
    MFnNumericAttribute orFn(aOutRotateValue);
    orFn.setWritable(false);
    orFn.setStorable(false);

    aOutRotate = cAttr.create("outRotate", "ort", &stat);
    CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutRotateValue);

    // --- node attribute registration ---
    stat = addAttribute(aTime);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aGravity);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aFps);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aAnchorWorldMatrix);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aAnchorParentInverseMatrix);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aAnchorOffset);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aGroupWorldMatrix);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyWriteBackOffset);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyParentInverseMatrix);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyParentJointOffset);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aBodies);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aJoints);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = addAttribute(aOutRotate);
    CHECK_MSTATUS(stat);

    // Make `time` drive the outputs.
    stat = attributeAffects(aTime, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aTime, aOutRotate);
    CHECK_MSTATUS(stat);

    // Phase 4: every config input drives the outputs too, so the node is
    // re-evaluated when a body/joint/gravity/anchor input changes — that is
    // what lets compute() detect the config edit and rebuild the Bullet world.
    // (The anchor matrix VALUES change every frame; declaring the dependency
    // also makes a kinematic bone dragged at a fixed time re-evaluate the node
    // so the attached chains follow immediately.)
    stat = attributeAffects(aGravity, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aGravity, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aFps, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aFps, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodies, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodies, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorWorldMatrix, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorWorldMatrix, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorParentInverseMatrix, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorParentInverseMatrix, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorOffset, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorOffset, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aGroupWorldMatrix, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aGroupWorldMatrix, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyWriteBackOffset, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyWriteBackOffset, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyParentInverseMatrix, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyParentInverseMatrix, aOutRotate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyParentJointOffset, aOutTranslate);
    CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyParentJointOffset, aOutRotate);
    CHECK_MSTATUS(stat);

    return MS::kSuccess;
}

// ===========================================================================
// Data reading
// ===========================================================================
bool MMDPhysicsNode::readBodyData(MDataBlock& dataBlock)
{
    mBodies.clear();
    MArrayDataHandle bodiesHandle = dataBlock.inputArrayValue(aBodies);
    const unsigned int bodyCount = bodiesHandle.elementCount();
    for (unsigned int i = 0; i < bodyCount; ++i)
    {
        bodiesHandle.jumpToArrayElement(i);
        MDataHandle bodyHandle = bodiesHandle.inputValue();
        Body b;
        std::memset(&b, 0, sizeof(b));
        b.mass = 1.0;
        b.friction = 0.5;
        b.extents[0] = b.extents[1] = b.extents[2] = 1.0;
        b.length = 1.0;
        b.radius = 0.5;
        b.group = 1;
        b.mask = 0xFFFF;
        b.groupId = -1;
        b.nonCollisionGroup = -1;
        b.enabled = true;

        auto read3 = [&](const MObject& attr, double out[3])
        {
            MDataHandle h = bodyHandle.child(attr);
            out[0] = h.asDouble3()[0];
            out[1] = h.asDouble3()[1];
            out[2] = h.asDouble3()[2];
        };
        read3(aBodyRestTranslate, b.restPos);
        read3(aBodyRestRotate, b.restRot);
        b.mass = bodyHandle.child(aBodyMass).asDouble();
        b.linearDamping = bodyHandle.child(aBodyLinearDamping).asDouble();
        b.angularDamping = bodyHandle.child(aBodyAngularDamping).asDouble();
        b.friction = bodyHandle.child(aBodyFriction).asDouble();
        b.restitution = bodyHandle.child(aBodyRestitution).asDouble();
        b.colliderType = bodyHandle.child(aBodyColliderType).asShort();
        b.radius = bodyHandle.child(aBodyRadius).asDouble();
        read3(aBodyExtents, b.extents);
        b.length = bodyHandle.child(aBodyLength).asDouble();
        b.mask = bodyHandle.child(aBodyMask).asInt();
        b.groupId = bodyHandle.child(aBodyGroupId).asShort();
        b.nonCollisionGroup = bodyHandle.child(aBodyNonCollisionGroup).asInt();
        b.kinematic = bodyHandle.child(aBodyKinematic).asBool();
        b.physicsMode = bodyHandle.child(aBodyPhysicsMode).asShort();
        b.parentBodyIndex = bodyHandle.child(aBodyParentBodyIndex).asShort();
        b.resetAnchorIndex = bodyHandle.child(aBodyResetAnchorIndex).asInt();
        b.enabled = bodyHandle.child(aBodyEnabled).asBool();
        mBodies.push_back(b);
    }
    return !mBodies.empty();
}

bool MMDPhysicsNode::readJointData(MDataBlock& dataBlock)
{
    mJoints.clear();
    MArrayDataHandle jointsHandle = dataBlock.inputArrayValue(aJoints);
    const unsigned int jointCount = jointsHandle.elementCount();
    for (unsigned int i = 0; i < jointCount; ++i)
    {
        jointsHandle.jumpToArrayElement(i);
        MDataHandle jointHandle = jointsHandle.inputValue();
        Joint j;
        std::memset(&j, 0, sizeof(j));
        j.bodyA = jointHandle.child(aJointBodyA).asInt();
        j.bodyB = jointHandle.child(aJointBodyB).asInt();
        j.type = jointHandle.child(aJointType).asInt();
        auto read3 = [&](const MObject& attr, double out[3])
        {
            MDataHandle h = jointHandle.child(attr);
            out[0] = h.asDouble3()[0];
            out[1] = h.asDouble3()[1];
            out[2] = h.asDouble3()[2];
        };
        read3(aJointFrameTranslate, j.frameT);
        read3(aJointFrameRotate, j.frameR);
        read3(aJointLinearMin, j.linearMin);
        read3(aJointLinearMax, j.linearMax);
        read3(aJointAngularMin, j.angularMin);
        read3(aJointAngularMax, j.angularMax);
        read3(aJointLinearSpring, j.linearSpring);
        read3(aJointAngularSpring, j.angularSpring);
        mJoints.push_back(j);
    }
    return true;
}

// ===========================================================================
// Config signature (Phase 4)
// ===========================================================================
// The node rebuilds the Bullet world when the user edits any of the inputs
// that DEFINE it: gravity, fps, the bodies/joints arrays (values AND counts),
// and the number of kinematic anchors.  The anchor matrix VALUES are
// deliberately excluded — they change every frame — only their counts matter.
// Mass, damping, friction, restitution, collider size, group/mask, joint
// limits and springs are all baked into the Bullet construction info at build
// time, so an edit only takes effect after a rebuild; hashing them lets
// compute() detect the edit and rebuild in place.
uint64_t MMDPhysicsNode::computeConfigSignature(MDataBlock& dataBlock) const
{
    uint64_t h = 0xcbf29ce484222325ULL; // FNV-1a offset basis

    auto read3 = [](MDataHandle& hd, const MObject& attr, double out[3])
    {
        MDataHandle ch = hd.child(attr);
        out[0] = ch.asDouble3()[0];
        out[1] = ch.asDouble3()[1];
        out[2] = ch.asDouble3()[2];
    };

    // gravity + fps
    MDataHandle grav = dataBlock.inputValue(aGravity);
    h = hashDouble3(h, grav.asDouble3());
    h = hashValue(h, dataBlock.inputValue(aFps).asDouble());

    // bodies
    MArrayDataHandle bodiesHandle = dataBlock.inputArrayValue(aBodies);
    const unsigned int bodyCount = bodiesHandle.elementCount();
    h = hashValue(h, bodyCount);
    double v3[3];
    for (unsigned int i = 0; i < bodyCount; ++i)
    {
        bodiesHandle.jumpToArrayElement(i);
        MDataHandle bh = bodiesHandle.inputValue();
        read3(bh, aBodyRestTranslate, v3);
        h = hashDouble3(h, v3);
        read3(bh, aBodyRestRotate, v3);
        h = hashDouble3(h, v3);
        h = hashValue(h, bh.child(aBodyMass).asDouble());
        h = hashValue(h, bh.child(aBodyLinearDamping).asDouble());
        h = hashValue(h, bh.child(aBodyAngularDamping).asDouble());
        h = hashValue(h, bh.child(aBodyFriction).asDouble());
        h = hashValue(h, bh.child(aBodyRestitution).asDouble());
        h = hashValue(h, bh.child(aBodyColliderType).asShort());
        h = hashValue(h, bh.child(aBodyRadius).asDouble());
        read3(bh, aBodyExtents, v3);
        h = hashDouble3(h, v3);
        h = hashValue(h, bh.child(aBodyLength).asDouble());
        h = hashValue(h, bh.child(aBodyMask).asInt());
        h = hashValue(h, bh.child(aBodyGroupId).asShort());
        h = hashValue(h, bh.child(aBodyNonCollisionGroup).asInt());
        h = hashValue(h, bh.child(aBodyKinematic).asBool());
        h = hashValue(h, bh.child(aBodyPhysicsMode).asShort());
        h = hashValue(h, bh.child(aBodyParentBodyIndex).asShort());
        h = hashValue(h, bh.child(aBodyResetAnchorIndex).asInt());
        // enabled is part of the config (toggling it rebuilds the world);
        // bodyNameLocal/bodyNameUniversal are NOT hashed — no simulation effect.
        h = hashValue(h, bh.child(aBodyEnabled).asBool());
    }

    // joints
    MArrayDataHandle jointsHandle = dataBlock.inputArrayValue(aJoints);
    const unsigned int jointCount = jointsHandle.elementCount();
    h = hashValue(h, jointCount);
    for (unsigned int i = 0; i < jointCount; ++i)
    {
        jointsHandle.jumpToArrayElement(i);
        MDataHandle jh = jointsHandle.inputValue();
        h = hashValue(h, jh.child(aJointBodyA).asInt());
        h = hashValue(h, jh.child(aJointBodyB).asInt());
        h = hashValue(h, jh.child(aJointType).asInt());
        read3(jh, aJointFrameTranslate, v3);
        h = hashDouble3(h, v3);
        read3(jh, aJointFrameRotate, v3);
        h = hashDouble3(h, v3);
        read3(jh, aJointLinearMin, v3);
        h = hashDouble3(h, v3);
        read3(jh, aJointLinearMax, v3);
        h = hashDouble3(h, v3);
        read3(jh, aJointAngularMin, v3);
        h = hashDouble3(h, v3);
        read3(jh, aJointAngularMax, v3);
        h = hashDouble3(h, v3);
        read3(jh, aJointLinearSpring, v3);
        h = hashDouble3(h, v3);
        read3(jh, aJointAngularSpring, v3);
        h = hashDouble3(h, v3);
    }

    // anchor counts (the values are per-frame; the counts define the world
    // structure — adding/removing a kinematic anchor is a config change)
    MArrayDataHandle anchors = dataBlock.inputArrayValue(aAnchorWorldMatrix);
    h = hashValue(h, anchors.elementCount());
    MArrayDataHandle parentInv = dataBlock.inputArrayValue(aAnchorParentInverseMatrix);
    h = hashValue(h, parentInv.elementCount());
    // Phase 3 write-back arrays: only the COUNTS (the offset matrices are
    // baked constants; the parent-inverse matrices vary every frame).
    MArrayDataHandle anchorOffset = dataBlock.inputArrayValue(aAnchorOffset);
    h = hashValue(h, anchorOffset.elementCount());
    MArrayDataHandle wbOffset = dataBlock.inputArrayValue(aBodyWriteBackOffset);
    h = hashValue(h, wbOffset.elementCount());
    MArrayDataHandle wbParentInv = dataBlock.inputArrayValue(aBodyParentInverseMatrix);
    h = hashValue(h, wbParentInv.elementCount());
    MArrayDataHandle wbParentJointOffset = dataBlock.inputArrayValue(aBodyParentJointOffset);
    h = hashValue(h, wbParentJointOffset.elementCount());

    return h;
}

// ===========================================================================
// World construction
// ===========================================================================
bool MMDPhysicsNode::buildWorld(MDataBlock& dataBlock)
{
    if (mWorldBuilt)
        return true;
    if (mBodies.empty())
        return false;

    // NOTE: do NOT call destroyWorld() here — it clears mBodies/mJoints which
    // were just read from the datablock. The caller guarantees the world is
    // not built when this runs.

    // Read gravity + fps
    MDataHandle gravHandle = dataBlock.inputValue(aGravity);
    btVector3 gravity(gravHandle.asDouble3()[0], gravHandle.asDouble3()[1],
                      gravHandle.asDouble3()[2]);

    // Resolve the collision group + effective mask (Phase 2).  Python feeds the
    // RAW PMX data (bodyGroupId + bodyNonCollisionGroup); the node derives the
    // Bullet group bit and the effective mask itself — an exact port of the
    // previous Python-side proximity + cloth-on-cloth corrections (see
    // mmd_physics_masks.h).  bodyMask stays as an explicit override used only
    // when bodyNonCollisionGroup is < 0 (legacy scenes).
    bool needMasks = false;
    for (const Body& b : mBodies)
        if (b.nonCollisionGroup != -1)
            needMasks = true;

    std::vector<long> computedMasks;
    if (needMasks)
    {
        std::vector<mmd_physics_masks::BodyInput> inputs;
        inputs.reserve(mBodies.size());
        for (const Body& b : mBodies)
        {
            mmd_physics_masks::BodyInput bi;
            bi.pos[0] = b.restPos[0];
            bi.pos[1] = b.restPos[1];
            bi.pos[2] = b.restPos[2];
            bi.colliderType = b.colliderType;
            bi.radius = b.radius;
            bi.extents[0] = b.extents[0];
            bi.extents[1] = b.extents[1];
            bi.extents[2] = b.extents[2];
            bi.length = b.length;
            // Use the raw PMX group id (legacy scenes without it default to 0).
            bi.groupId = b.groupId >= 0 ? b.groupId : 0;
            bi.kinematic = b.kinematic;
            bi.nonCollisionGroup = b.nonCollisionGroup;
            inputs.push_back(bi);
        }
        std::vector<mmd_physics_masks::JointInput> jins;
        jins.reserve(mJoints.size());
        for (const Joint& j : mJoints)
        {
            mmd_physics_masks::JointInput ji;
            ji.bodyA = static_cast<int>(j.bodyA);
            ji.bodyB = static_cast<int>(j.bodyB);
            jins.push_back(ji);
        }
        mmd_physics_masks::computeEffectiveMasks(inputs, jins, computedMasks);
    }

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        // Bullet group bit from the raw PMX group id (legacy scenes without
        // it keep the default group 0).
        mBodies[i].group = 1L << ((mBodies[i].groupId >= 0 ? mBodies[i].groupId : 0) & 0x0F);
        if (mBodies[i].nonCollisionGroup != -1 && needMasks)
            mBodies[i].mask = computedMasks[i];
    }

    // World-level objects.  The world does NOT own the dispatcher / broadphase
    // / collision config / solver — keep them as members so they are freed
    // exactly once in destroyWorld() (after the world, which uses them during
    // its destructor).
    mCollisionConfig.reset(new btDefaultCollisionConfiguration());
    mDispatcher.reset(new btCollisionDispatcher(mCollisionConfig.get()));
    mBroadphase.reset(new btDbvtBroadphase());
    mConstraintSolver.reset(new btSequentialImpulseConstraintSolver());
    mWorld.reset(new btDiscreteDynamicsWorld(mDispatcher.get(), mBroadphase.get(),
                                             mConstraintSolver.get(), mCollisionConfig.get()));
    mWorld->setGravity(gravity);
    // Long rigid chains (MMD skirt/hair/ponytail strands are 10-30 links) need
    // more constraint iterations than Bullet's default 10, or the tension never
    // propagates and the chain detaches from its kinematic anchor (free-falls).
    mWorld->getSolverInfo().m_numIterations = kSolverIterations;

    // Create bodies
    mAnchorRest.clear();
    mAnchorCurrent.clear();
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        const Body& b = mBodies[i];
        if (!b.enabled)
        {
            // Disabled (removed): keep the body index ALIGNED so the outputs
            // and draw data stay body-indexed — store a null placeholder and
            // never add it to the world (no collision, no simulation).  A
            // disabled kinematic body also gets no anchor entry.
            mRigidBodies.emplace_back(nullptr);
            continue;
        }
        btTransform start = transformFromRest(b.restPos, b.restRot);

        btCollisionShape* shape = nullptr;
        if (b.colliderType == kColliderSphere)
        {
            shape = new btSphereShape(btScalar(std::max(b.radius, 1e-4)));
            mShapes.emplace_back(shape);
        }
        else if (b.colliderType == kColliderBox)
        {
            shape =
                new btBoxShape(btVector3(std::max(b.extents[0], 1e-4), std::max(b.extents[1], 1e-4),
                                         std::max(b.extents[2], 1e-4)));
            mShapes.emplace_back(shape);
        }
        else // capsule — btCapsuleShape is ALREADY Y-axis (m_upAxis = 1), which
             // matches MMD's vertical capsule and the polyCylinder guide mesh.
             // (An earlier "Bullet capsule axis is Z" rotation was WRONG — it
             // turned every capsule sideways, e.g. the torso capsule pointed its
             // hemispherical cap at the skirt and pushed it ~1 unit out, making
             // the skirt float with a visible gap from the body.)
        {
            auto* capsule = new btCapsuleShape(btScalar(std::max(b.radius, 1e-4)),
                                               btScalar(std::max(b.length, 1e-4)));
            mShapes.emplace_back(capsule);
            shape = capsule;
        }

        btScalar mass = b.kinematic ? 0.0 : std::max(b.mass, 0.0);
        btVector3 localInertia(0, 0, 0);
        if (mass > 0.0)
            shape->calculateLocalInertia(mass, localInertia);

        auto* motionState = new btDefaultMotionState(start);
        btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState, shape, localInertia);
        ci.m_linearDamping = btScalar(b.linearDamping);
        ci.m_angularDamping = btScalar(b.angularDamping);
        ci.m_friction = btScalar(b.friction);
        ci.m_restitution = btScalar(b.restitution);
        auto* body = new btRigidBody(ci);

        if (b.kinematic)
        {
            body->setCollisionFlags(body->getCollisionFlags() |
                                    btCollisionObject::CF_KINEMATIC_OBJECT);
            body->setActivationState(DISABLE_DEACTIVATION);
            body->setGravity(btVector3(0, 0, 0));
            // Record the anchor's REST pose (group-local) for scrub-back.
            mAnchorRest.emplace_back(AnchorPose());
            storeAnchorPose(mAnchorRest.back().pos, mAnchorRest.back().quat, start);
            // mAnchorCurrent must be the SAME size as mAnchorRest — it is
            // refreshed every frame in updateKinematicAnchors and drives the
            // scrub-back reset (empty would silently disable the rewind).
            mAnchorCurrent.emplace_back(AnchorPose());
            storeAnchorPose(mAnchorCurrent.back().pos, mAnchorCurrent.back().quat, start);
        }
        else
        {
            body->setActivationState(ISLAND_SLEEPING); // wake on first step
            body->activate();
        }

        // Scrub-back reset: capture the constant offset bodyRest = anchorRest *
        // offset, where the anchor is the kinematic body whose bone is this
        // body's nearest kinematic ancestor (mapped by Python).  On rewind the
        // body is teleported to anchorCurrent * offset — i.e. its rest pose
        // transformed by the CURRENT skeleton pose, instead of rebuilding at
        // the PMX rest pose while the skeleton is at another frame.
        if (!b.kinematic && b.resetAnchorIndex >= 0 &&
            b.resetAnchorIndex < (int) mAnchorRest.size())
        {
            const btTransform anchorRest = anchorPoseToTransform(
                mAnchorRest[b.resetAnchorIndex].pos, mAnchorRest[b.resetAnchorIndex].quat);
            const btTransform bodyRest = start;
            const btTransform offset = anchorRest.inverse() * bodyRest;
            mBodies[i].hasBoneReset = true;
            const btVector3& o = offset.getOrigin();
            const btQuaternion& q = offset.getRotation();
            mBodies[i].resetOffsetPos[0] = o.x();
            mBodies[i].resetOffsetPos[1] = o.y();
            mBodies[i].resetOffsetPos[2] = o.z();
            mBodies[i].resetOffsetQuat[0] = q.x();
            mBodies[i].resetOffsetQuat[1] = q.y();
            mBodies[i].resetOffsetQuat[2] = q.z();
            mBodies[i].resetOffsetQuat[3] = q.w();
        }

        mWorld->addRigidBody(body, b.group, b.mask);
        mRigidBodies.emplace_back(body);
    }

    // Create joints
    for (const Joint& j : mJoints)
    {
        if (j.bodyA < 0 || j.bodyB < 0 || j.bodyA >= (long) mRigidBodies.size() ||
            j.bodyB >= (long) mRigidBodies.size())
            continue;
        // Skip joints that reference a disabled (removed) body.
        if (!mBodies[j.bodyA].enabled || !mBodies[j.bodyB].enabled)
            continue;
        btRigidBody* rbA = mRigidBodies[j.bodyA].get();
        btRigidBody* rbB = mRigidBodies[j.bodyB].get();
        btTransform frameWorld = transformFromRest(j.frameT, j.frameR);
        btTransform frameInA = rbA->getWorldTransform().inverse() * frameWorld;
        btTransform frameInB = rbB->getWorldTransform().inverse() * frameWorld;

        btTypedConstraint* con = nullptr;
        switch (j.type)
        {
        case kJointSpring6Dof:
        {
            // MMD maps EVERY SPRING_6DOF joint to btGeneric6DofSpring2Constraint —
            // that is exactly what its physics engine creates.  The spring-2
            // limit motor treats upper==lower as LOCKED (see
            // btTranslationalLimitMotor2), so:
            //   * zero springs + zero limits -> proper RIGID WELD (locked),
            //     without btFixedConstraint's infinite-stiffness spring creep;
            //   * zero springs + real limits -> flexible 6DOF (limited);
            //   * nonzero springs            -> springy (PMX stiffness).
            // This is the single mapping that matches MMD's behaviour for the
            // whole joints data (rigid hair/cape chains, springy skirt, etc.).
            auto* g6 = new btGeneric6DofSpring2Constraint(*rbA, *rbB, frameInA, frameInB);
            g6->setLinearLowerLimit(btVector3(j.linearMin[0], j.linearMin[1], j.linearMin[2]));
            g6->setLinearUpperLimit(btVector3(j.linearMax[0], j.linearMax[1], j.linearMax[2]));
            g6->setAngularLowerLimit(btVector3(j.angularMin[0], j.angularMin[1], j.angularMin[2]));
            g6->setAngularUpperLimit(btVector3(j.angularMax[0], j.angularMax[1], j.angularMax[2]));
            for (int ax = 0; ax < 3; ++ax)
            {
                if (j.linearSpring[ax] != 0)
                {
                    g6->enableSpring(ax, true);
                    g6->setStiffness(ax, btScalar(j.linearSpring[ax]));
                }
                if (j.angularSpring[ax] != 0)
                {
                    g6->enableSpring(ax + 3, true);
                    g6->setStiffness(ax + 3, btScalar(j.angularSpring[ax]));
                }
            }
            con = g6;
            break;
        }
        case kJointSixDof:
        {
            auto* g6 = new btGeneric6DofConstraint(*rbA, *rbB, frameInA, frameInB, true);
            g6->setLinearLowerLimit(btVector3(j.linearMin[0], j.linearMin[1], j.linearMin[2]));
            g6->setLinearUpperLimit(btVector3(j.linearMax[0], j.linearMax[1], j.linearMax[2]));
            g6->setAngularLowerLimit(btVector3(j.angularMin[0], j.angularMin[1], j.angularMin[2]));
            g6->setAngularUpperLimit(btVector3(j.angularMax[0], j.angularMax[1], j.angularMax[2]));
            con = g6;
            break;
        }
        case kJointP2P:
        {
            btVector3 pivotInA = frameInA.getOrigin();
            btVector3 pivotInB = frameInB.getOrigin();
            con = new btPoint2PointConstraint(*rbA, *rbB, pivotInA, pivotInB);
            break;
        }
        case kJointConeTwist:
        {
            auto* ct = new btConeTwistConstraint(*rbA, *rbB, frameInA, frameInB);
            ct->setLimit(j.angularMin[1], j.angularMax[1], 0.0, 0.3f, 0.0f, 1.0f);
            con = ct;
            break;
        }
        case kJointSlider:
        {
            auto* sl = new btSliderConstraint(*rbA, *rbB, frameInA, frameInB, true);
            sl->setLowerLinLimit(j.linearMin[1]);
            sl->setUpperLinLimit(j.linearMax[1]);
            sl->setLowerAngLimit(j.angularMin[1]);
            sl->setUpperAngLimit(j.angularMax[1]);
            con = sl;
            break;
        }
        case kJointHinge:
        {
            auto* hi = new btHingeConstraint(*rbA, *rbB, frameInA, frameInB, true);
            hi->setLimit(j.angularMin[1], j.angularMax[1], 0.3f, 0.0f, 1.0f);
            con = hi;
            break;
        }
        default:
            break;
        }

        if (con)
        {
            mWorld->addConstraint(con, /*disableCollisionsBetweenLinkedBodies=*/true);
            mConstraints.emplace_back(con);
        }
    }

    mWorldBuilt = true;
    return true;
}

// ===========================================================================
// Per-frame update
// ===========================================================================
bool MMDPhysicsNode::updateKinematicAnchors(MDataBlock& dataBlock)
{
    if (!mWorld)
        return false;
    // anchorWorldMatrix[i] + anchorParentInverseMatrix[i] map 1:1 to the
    // kinematic bodies in body order.  local = world * parentInverse (row-vector
    // convention) — the Bullet world runs in the physics group's local space.
    bool anchorsMoved = false;
    MArrayDataHandle anchors = dataBlock.inputArrayValue(aAnchorWorldMatrix);
    MArrayDataHandle parentInverse = dataBlock.inputArrayValue(aAnchorParentInverseMatrix);
    MArrayDataHandle anchorOffset = dataBlock.inputArrayValue(aAnchorOffset);
    const unsigned int anchorCount = anchors.elementCount();
    const unsigned int parentInverseCount = parentInverse.elementCount();
    const unsigned int offsetCount = anchorOffset.elementCount();
    int anchorIndex = 0;
    for (size_t i = 0; i < mBodies.size() && anchorIndex < (int) anchorCount; ++i)
    {
        if (!mBodies[i].kinematic || !mBodies[i].enabled)
            continue;
        anchors.jumpToArrayElement(anchorIndex);
        MMatrix w = anchors.inputValue().asMatrix();
        if (anchorIndex < (int) parentInverseCount)
        {
            parentInverse.jumpToArrayElement(anchorIndex);
            w *= parentInverse.inputValue().asMatrix();
        }
        // Phase 3: apply the baked world-frame offset (colliderRestWorld *
        // jointRestWorld^-1) so the kinematic collider tracks the JOINT with
        // the PMX body<->bone offset preserved (this is exactly what the old
        // parentConstraint(joint, guide, maintainOffset) maintained — verified
        // empirically: targetWorld = K * sourceWorld, K constant).  world here
        // is the JOINT's world matrix and parentInverse is the physics GROUP's
        // world inverse, so world * parentInverse is the joint in group space.
        if (anchorIndex < (int) offsetCount)
        {
            anchorOffset.jumpToArrayElement(anchorIndex);
            w = anchorOffset.inputValue().asMatrix() * w;
        }
        btTransform t = mayaMatrixToBtTransform(w);
        // Detect anchor movement (e.g. a bone dragged in the viewport at the
        // current frame): if an anchor moved but time did not advance, the sim
        // still needs to step so attached chains follow the bone immediately
        // (MMD reacts to bone changes instantly — not on the next frame).
        if (anchorIndex < (int) mAnchorCurrent.size())
        {
            const btTransform prev = anchorPoseToTransform(mAnchorCurrent[anchorIndex].pos,
                                                           mAnchorCurrent[anchorIndex].quat);
            const btVector3 d = t.getOrigin() - prev.getOrigin();
            const btVector3 c0 = t.getBasis().getColumn(0);
            const btVector3 p0 = prev.getBasis().getColumn(0);
            const btVector3 c1 = t.getBasis().getColumn(1);
            const btVector3 p1 = prev.getBasis().getColumn(1);
            if (d.length2() > btScalar(1e-6) || c0.dot(p0) < btScalar(1.0) - btScalar(1e-5) ||
                c1.dot(p1) < btScalar(1.0) - btScalar(1e-5))
                anchorsMoved = true;
        }
        mRigidBodies[i]->setWorldTransform(t);
        mRigidBodies[i]->getMotionState()->setWorldTransform(t);
        if (anchorIndex < (int) mAnchorCurrent.size())
            storeAnchorPose(mAnchorCurrent[anchorIndex].pos, mAnchorCurrent[anchorIndex].quat, t);
        ++anchorIndex;
    }
    return anchorsMoved;
}

void MMDPhysicsNode::resetDynamicBodies(MDataBlock& dataBlock)
{
    // Teleport every dynamic body (that has a reset anchor) to its rest pose
    // transformed by the CURRENT skeleton pose, zeroing velocities.  Called
    // when time is scrubbed backwards: the world is NOT rebuilt, so the sim
    // simply continues from the pose the skeleton is actually in — instead of
    // hair/skirt chains hanging at the PMX rest pose while the skeleton is at
    // another frame (which looked broken).  Uses the anchor CURRENT poses
    // captured this frame in updateKinematicAnchors().
    (void) dataBlock;
    if (!mWorld)
        return;
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        if (mBodies[i].kinematic || !mBodies[i].enabled || !mBodies[i].hasBoneReset)
            continue;
        const int aIdx = mBodies[i].resetAnchorIndex;
        if (aIdx < 0 || aIdx >= (int) mAnchorCurrent.size())
            continue;

        const btTransform anchorCurrent =
            anchorPoseToTransform(mAnchorCurrent[aIdx].pos, mAnchorCurrent[aIdx].quat);
        btTransform offset;
        offset.setIdentity();
        offset.setOrigin(btVector3(btScalar(mBodies[i].resetOffsetPos[0]),
                                   btScalar(mBodies[i].resetOffsetPos[1]),
                                   btScalar(mBodies[i].resetOffsetPos[2])));
        offset.setRotation(btQuaternion(
            btScalar(mBodies[i].resetOffsetQuat[0]), btScalar(mBodies[i].resetOffsetQuat[1]),
            btScalar(mBodies[i].resetOffsetQuat[2]), btScalar(mBodies[i].resetOffsetQuat[3])));
        const btTransform target = anchorCurrent * offset;

        btRigidBody* body = mRigidBodies[i].get();
        body->setWorldTransform(target);
        body->getMotionState()->setWorldTransform(target);
        body->setLinearVelocity(btVector3(0, 0, 0));
        body->setAngularVelocity(btVector3(0, 0, 0));
        body->setActivationState(DISABLE_DEACTIVATION);
        body->activate();
    }
}

void MMDPhysicsNode::getCacheSetup(const MEvaluationNode& evalNode,
                                   MNodeCacheDisablingInfo& disablingInfo,
                                   MNodeCacheSetupInfo& setupInfo,
                                   MObjectArray& monitoredAttributes) const
{
    // This node advances an internal Bullet world in compute(), so its outputs
    // are NOT a pure function of its inputs.  Cached Playback must re-evaluate
    // it every frame, exactly like a scripted/expression node.
    MString category("mmdPhysicsNode: stateful Bullet solver (steps every frame)");
    MNodeCacheDisablingInfoHelper::setUnsafeNode(disablingInfo, evalNode, &category);
    MPxNode::getCacheSetup(evalNode, disablingInfo, setupInfo, monitoredAttributes);
}

bool MMDPhysicsNode::writeOutputs(MDataBlock& dataBlock)
{
    // Phase 3 direct write-back: the node outputs the JOINT-LOCAL pose so
    // Python can connect outTranslate/outRotate straight into the joints (no
    // guide transforms, no parent/orientConstraints).  The primary transform
    // is
    //   boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1
    // where K = jointRestWorld * bodyRestWorld^-1 (baked by Python) and the
    // parent inverse is derived from the PARENT BODY's solved Bullet transform
    // (M_parent * B_parent * groupWorld = parentJointWorld, M_parent =
    // parentJointRestWorld * parentBodyRestWorld^-1 baked by Python).  This is
    // the exact world-space offset that parentConstraint(maintainOffset)
    // maintained (verified empirically: targetWorld = K * sourceWorld), so it
    // is EXACT at rest and invariant when the whole model is moved.  Deriving
    // the parent inverse from the parent BODY (not the DG joint matrix) is
    // what keeps the write-back free of the DG feedback cycle that exploded
    // the simulation when a parent joint was itself node-driven.  For bodies
    // whose parent bone has no body (and for old scenes) a DG
    // parent-inverse fallback is used — that parent is never node-driven, so
    // it cannot feed back.
    MMatrix groupWorld;
    bool haveGroupWorld = false;
    MPlug gwPlug(thisMObject(), aGroupWorldMatrix);
    if (gwPlug.isConnected())
    {
        groupWorld = dataBlock.inputValue(aGroupWorldMatrix).asMatrix();
        haveGroupWorld = true;
    }
    MArrayDataHandle offsetHandle = dataBlock.inputArrayValue(aBodyWriteBackOffset);
    MArrayDataHandle parentInvHandle = dataBlock.inputArrayValue(aBodyParentInverseMatrix);
    MArrayDataHandle parentJointOffsetHandle = dataBlock.inputArrayValue(aBodyParentJointOffset);

    // Dynamic bodies → outTranslate[i] / outRotate[i] keyed by BODY index
    // (kinematic bodies get no output element; reading them yields defaults).
    MArrayDataBuilder tBuilder(&dataBlock, aOutTranslate, (unsigned int) mBodies.size());
    MArrayDataBuilder rBuilder(&dataBlock, aOutRotate, (unsigned int) mBodies.size());

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        if (mBodies[i].kinematic || !mBodies[i].enabled)
            continue;
        btRigidBody* body = mRigidBodies[i].get();
        const btTransform& wt = body->getWorldTransform();

        // Start from the group-space body pose (Maya row-vector matrix).
        double outRow[4][4];
        btTransformToRowMatrix(wt, outRow);

        // PRIMARY write-back path (Phase 3 cycle fix): the parent inverse is
        // derived from the PARENT BODY's solved Bullet transform, never from
        // the DG.  For a body whose parent JOINT is also node-driven the old
        // `joint.parentInverseMatrix` dependency created a DG feedback cycle
        // (parentJoint.worldMatrix <- node.outRotate <- node.compute <- ...
        // <- parentJoint.parentInverseMatrix) that made the simulation
        // explode.  Here:
        //   boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1
        // with K = bodyWriteBackOffset[i] (jointRestWorld * bodyRestWorld^-1)
        // and M_parent = bodyParentJointOffset[i] (parentJointRestWorld *
        // parentBodyRestWorld^-1, the same constant for kinematic and dynamic
        // parents).  Because parentJointWorld = M_parent * B_parent *
        // groupWorld, the groupWorld term cancels and boneLocal is EXACT at
        // rest for both parent kinds (verified algebraically).
        const int parentIdx = mBodies[i].parentBodyIndex;
        if (parentIdx >= 0 && (size_t) parentIdx < mRigidBodies.size() && mRigidBodies[parentIdx] &&
            offsetHandle.jumpToArrayElement((unsigned int) i) == MS::kSuccess &&
            parentJointOffsetHandle.jumpToArrayElement((unsigned int) i) == MS::kSuccess)
        {
            MMatrix k = offsetHandle.inputValue().asMatrix();
            MMatrix mp = parentJointOffsetHandle.inputValue().asMatrix();
            double bpRow[4][4];
            btTransformToRowMatrix(mRigidBodies[parentIdx]->getWorldTransform(), bpRow);
            MMatrix bParent(bpRow);
            MMatrix bodyLocal(outRow);
            MMatrix result = k * bodyLocal * bParent.inverse() * mp.inverse();
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    outRow[r][c] = result(r, c);
        }
        else if (haveGroupWorld &&
                 offsetHandle.jumpToArrayElement((unsigned int) i) == MS::kSuccess)
        {
            // FALLBACK (parent bone has no rigid body, or an old scene): the
            // original formula with the DG parent-inverse input.  Only used
            // when the parent joint is NOT node-driven (its bone has no body
            // and no dynamic ancestor), so it cannot feed back into the node.
            MMatrix k = offsetHandle.inputValue().asMatrix();
            double kRow[4][4];
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    kRow[r][c] = k(r, c);
            double tmp[4][4];
            rowMatrixMultiply(kRow, outRow, tmp);
            double gw[4][4];
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    gw[r][c] = groupWorld(r, c);
            double tmp2[4][4];
            rowMatrixMultiply(tmp, gw, tmp2);
            if (parentInvHandle.jumpToArrayElement((unsigned int) i) == MS::kSuccess)
            {
                MMatrix pi = parentInvHandle.inputValue().asMatrix();
                double piRow[4][4];
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c)
                        piRow[r][c] = pi(r, c);
                rowMatrixMultiply(tmp2, piRow, outRow);
            }
            else
            {
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c)
                        outRow[r][c] = tmp2[r][c];
            }
        }

        const double* o = outRow[3]; // row-vector translation
        double rot[3];
        btTransform boneLocal = doubleMatrixToBtTransform(outRow);
        quatToEulerXYZDegrees(boneLocal.getRotation(), rot);

        MDataHandle tEl = tBuilder.addElement((unsigned int) i);
        MDataHandle tChild = tEl.child(aOutTranslateValue);
        tChild.set3Double(o[0], o[1], o[2]);

        MDataHandle rEl = rBuilder.addElement((unsigned int) i);
        MDataHandle rChild = rEl.child(aOutRotateValue);
        rChild.set3Double(rot[0], rot[1], rot[2]);
    }

    MArrayDataHandle tOut = dataBlock.outputArrayValue(aOutTranslate);
    tOut.set(tBuilder);
    tOut.setAllClean();

    MArrayDataHandle rOut = dataBlock.outputArrayValue(aOutRotate);
    rOut.set(rBuilder);
    rOut.setAllClean();

    return true;
}

// ===========================================================================
// Draw support
// ===========================================================================
void MMDPhysicsNode::collectDrawData(std::vector<DrawBody>& out) const
{
    out.clear();
    // Before the first compute() the internal body state is empty — draw the
    // REST guides straight from the node's attributes so the colliders are
    // visible immediately after import (and whenever nothing pulls the DG).
    if (mBodies.empty())
    {
        MPlug bodiesPlug(thisMObject(), aBodies);
        const unsigned int n = bodiesPlug.evaluateNumElements();
        for (unsigned int i = 0; i < n; ++i)
        {
            DrawBody db;
            readDrawBodyFromPlug(bodiesPlug.elementByLogicalIndex(i), db);
            out.push_back(db);
        }
        return;
    }
    out.reserve(mBodies.size());
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        const Body& b = mBodies[i];
        if (!b.enabled)
            continue;
        DrawBody db;
        db.colliderType = b.colliderType;
        db.radius = b.radius;
        db.extents[0] = b.extents[0];
        db.extents[1] = b.extents[1];
        db.extents[2] = b.extents[2];
        db.length = b.length;
        db.kinematic = b.kinematic;
        // group id straight from the raw PMX group id (clamp legacy -1).
        db.groupId = b.groupId >= 0 ? b.groupId : 0;
        if (mWorldBuilt && i < mRigidBodies.size() && mRigidBodies[i])
        {
            // Solved pose — what the simulation actually has right now.
            const btTransform& t = mRigidBodies[i]->getWorldTransform();
            const btVector3& o = t.getOrigin();
            const btQuaternion& q = t.getRotation();
            db.pos[0] = o.x();
            db.pos[1] = o.y();
            db.pos[2] = o.z();
            db.quat[0] = q.x();
            db.quat[1] = q.y();
            db.quat[2] = q.z();
            db.quat[3] = q.w();
        }
        else
        {
            // World not built yet — draw the PMX rest pose.
            db.pos[0] = b.restPos[0];
            db.pos[1] = b.restPos[1];
            db.pos[2] = b.restPos[2];
            const btQuaternion q = eulerDegreesToQuat(b.restRot[0], b.restRot[1], b.restRot[2]);
            db.quat[0] = q.x();
            db.quat[1] = q.y();
            db.quat[2] = q.z();
            db.quat[3] = q.w();
        }
        out.push_back(db);
    }
}

MBoundingBox MMDPhysicsNode::boundingBox() const
{
    MBoundingBox box;
    bool any = false;
    for (const Body& b : mBodies)
    {
        double r;
        if (b.colliderType == kColliderSphere)
            r = b.radius;
        else if (b.colliderType == kColliderBox)
            r = std::max({b.extents[0], b.extents[1], b.extents[2]});
        else
            r = b.radius + b.length * 0.5;
        r = std::max(r, 0.5);
        box.expand(MPoint(b.restPos[0] - r, b.restPos[1] - r, b.restPos[2] - r));
        box.expand(MPoint(b.restPos[0] + r, b.restPos[1] + r, b.restPos[2] + r));
        any = true;
    }
    if (!any)
        return MBoundingBox(MPoint(-1.0, -1.0, -1.0), MPoint(1.0, 1.0, 1.0));
    return box;
}

// ===========================================================================
// compute()
// ===========================================================================
MStatus MMDPhysicsNode::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    if (plug != aOutTranslate && plug != aOutRotate && !plug.isElement() && !plug.isChild())
    {
        return MS::kUnknownParameter;
    }

    MDataHandle timeHandle = dataBlock.inputValue(aTime);
    const MTime nowTime = timeHandle.asTime();
    const double now = nowTime.value();

    const uint64_t configSignature = computeConfigSignature(dataBlock);
    const bool firstEval = !mWorldBuilt;
    if (!mWorldBuilt)
    {
        readBodyData(dataBlock);
        readJointData(dataBlock);
        if (!buildWorld(dataBlock))
            return MS::kFailure;
        mConfigSignature = configSignature;
        mLastTime = now;
        mLastTimeUnit = nowTime.unit();
    }
    else if (configSignature != mConfigSignature)
    {
        // Phase 4 auto-rebuild: the user (or a re-import) edited the config
        // (gravity / fps / bodies / joints / anchor counts).  Mass, damping,
        // limits, collider size etc. are baked into the Bullet construction
        // info, so the world must be rebuilt for the edit to take effect.
        // Keep the dynamic chains glued to the CURRENT skeleton pose — exactly
        // like the rewind path — so an in-place edit does NOT teleport the
        // chains to the PMX rest pose.  (destroyWorld() is safe here: the
        // bodies/joints are re-read from the datablock right after.)
        updateKinematicAnchors(dataBlock); // capture the current skeleton pose
        destroyWorld();
        readBodyData(dataBlock);
        readJointData(dataBlock);
        if (!buildWorld(dataBlock))
            return MS::kFailure;
        mConfigSignature = configSignature;
        updateKinematicAnchors(dataBlock); // re-apply current anchors
        resetDynamicBodies(dataBlock);     // chains stay at the current pose
        mLastTime = now;                   // no time-step on the rebuild frame
        mLastTimeUnit = nowTime.unit();
    }

    // Refresh the kinematic anchors from their inputs every evaluation (so the
    // kinematic colliders track their bones even at a fixed time), and detect
    // whether any anchor moved since the previous step.
    bool anchorsMoved = false;
    if (!firstEval) // on the very first eval the anchor inputs may not be ready
        anchorsMoved = updateKinematicAnchors(dataBlock);

    // Step the sim when time advanced OR a kinematic anchor moved (a bone
    // dragged in the viewport at the current frame — MMD reacts to bone changes
    // immediately, so the attached chains must follow at once, not on the next
    // frame).
    const bool timeChanged = (mLastTime >= 0.0 && now != mLastTime);
    if (timeChanged || anchorsMoved)
    {
        double dt = 0.0;
        if (timeChanged)
        {
            // Frame span (in the scene's current time unit) -> SECONDS via
            // MTime.  This adapts automatically to whatever the scene's
            // playback unit is (film/game/custom 23.976 etc.) and tracks a
            // unit change mid-session — no fps attribute is needed.
            dt = (nowTime - MTime(mLastTime, mLastTimeUnit)).as(MTime::kSeconds);
        }
        bool rewound = false;
        if (dt < 0.0)
        {
            // Scrubbing backwards: REBUILD the Bullet world from the CURRENT
            // skeleton pose instead of keeping it and teleporting bodies.
            // Teleporting alone left the solver's warm-start impulse state
            // (from the previous frame) in place, so the first step after the
            // teleport catastrophically yanked the chains away from their
            // reset pose ("going back in time breaks the animation").  A fresh
            // world has no stale solver state; initializing the dynamic bodies
            // at anchorCurrent * offset (their rest pose transformed by the
            // current skeleton pose) keeps hair/skirt glued to the skeleton at
            // the frame the user actually scrubbed to.
            updateKinematicAnchors(dataBlock); // current skeleton pose -> anchors
            destroyWorld();                    // fresh bodies/constraints/solver
            readBodyData(dataBlock);
            readJointData(dataBlock);
            if (!buildWorld(dataBlock))
                return MS::kFailure;
            updateKinematicAnchors(dataBlock); // re-apply current anchors
            resetDynamicBodies(dataBlock);     // dynamic bodies at current-pose targets
            dt = 0.0;
            rewound = true;
        }
        dt = std::min(dt, kMaxStepTime); // guard against huge jumps
        // Anchor-only movement at a fixed time (a bone dragged in the
        // viewport, no rewind): still run one solver step so the chains are
        // pulled to follow the moved bone (a zero dt makes Bullet skip the
        // solve entirely).  NOT after a rewind though — the reset already
        // teleported the chains (possibly a large distance), and running a
        // solver step right after a big teleport yanks the chains away from
        // their reset pose (catastrophic one-step correction).
        if (!rewound && anchorsMoved && dt <= 0.0)
            dt = kFixedDt;
        mWorld->stepSimulation(btScalar(dt), kMaxSubSteps, btScalar(kFixedDt));
        mLastTime = now;
        mLastTimeUnit = nowTime.unit();
    }

    writeOutputs(dataBlock);

    dataBlock.outputValue(aOutTranslate).setClean();
    dataBlock.outputValue(aOutRotate).setClean();
    return MS::kSuccess;
}
