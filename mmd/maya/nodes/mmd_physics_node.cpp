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
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MNodeCacheDisablingInfo.h>
#include <maya/MNodeCacheDisablingInfoHelper.h>
#include <maya/MNodeCacheSetupInfo.h>
#include <maya/MPlug.h>
#include <maya/MQuaternion.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>

#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <algorithm>
#include <cmath>
#include <cstring>

// ===========================================================================
// Constants
// ===========================================================================
const MTypeId MMDPhysicsNode::kTypeId(0x0011C105); // 0x87000 + 0x100 for mmdPhysicsNode

// PMX JointType -> Bullet constraint selection
namespace
{
constexpr int kJointSpring6Dof = 0;
constexpr int kJointSixDof = 1;
constexpr int kJointP2P = 2;
constexpr int kJointConeTwist = 3;
constexpr int kJointSlider = 4;
constexpr int kJointHinge = 5;

// Collider types (match bulletRigidBodyShape + PMX ShapeType mapping)
constexpr short kColliderBox = 1;
constexpr short kColliderSphere = 2;
constexpr short kColliderCapsule = 3;

constexpr double kPi = 3.14159265358979323846;

double deg2rad(double d)
{
    return d * kPi / 180.0;
}
double rad2deg(double r)
{
    return r * 180.0 / kPi;
}
} // namespace

// ===========================================================================
// Attribute declarations
// ===========================================================================
MObject MMDPhysicsNode::aTime;
MObject MMDPhysicsNode::aGravity;
MObject MMDPhysicsNode::aFps;
MObject MMDPhysicsNode::aAnchorWorldMatrix;
MObject MMDPhysicsNode::aAnchorParentInverseMatrix;

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
MObject MMDPhysicsNode::aBodyGroup;
MObject MMDPhysicsNode::aBodyMask;
MObject MMDPhysicsNode::aBodyKinematic;
MObject MMDPhysicsNode::aBodyResetAnchorIndex;

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
// Euler helpers (Maya XYZ convention, degrees on attributes)
// ===========================================================================
namespace
{

btQuaternion eulerDegreesToQuat(double rx, double ry, double rz)
{
    // Maya's rotate-XYZ (rotateOrder 0) builds the row-vector matrix
    //   M_row = (Rz * Ry * Rx)^T   (verified empirically against Maya 2026)
    // so the equivalent Bullet (column-vector) matrix is M_col = Rz * Ry * Rx,
    // whose quaternion is q = qz * qy * qx.
    btQuaternion qx(btVector3(1, 0, 0), deg2rad(rx));
    btQuaternion qy(btVector3(0, 1, 0), deg2rad(ry));
    btQuaternion qz(btVector3(0, 0, 1), deg2rad(rz));
    return qz * qy * qx; // M_col = Rz * Ry * Rx
}

// Extract XYZ euler (degrees) from a quaternion in the Maya rotate convention.
// The Bullet matrix m = Rz * Ry * Rx (matches eulerDegreesToQuat = qz*qy*qx):
//   sin(ry) = -m[2][0];  rx = atan2(m[2][1], m[2][2]);  rz = atan2(m[1][0], m[0][0])
void quatToEulerXYZDegrees(const btQuaternion& q, double out[3])
{
    btMatrix3x3 m(q);
    const double sy = -m[2][0]; // sin(ry)
    const double epsilon = 1e-6;
    if (sy < -1.0 + epsilon || sy > 1.0 - epsilon)
    {
        // Gimbal lock: ry = ±90°, rx/rz degenerate (only their combination is
        // well-defined).  Set rx = 0 and solve the combined term:
        //   ry=+90: m[0][1]=sin(rx-rz), m[0][2]=cos(rx-rz) -> rz = atan2(-m[0][1], m[0][2])
        //   ry=-90: m[0][1]=-sin(rx+rz), m[1][1]=cos(rx+rz) -> rz = atan2(-m[0][1], m[1][1])
        // (Earlier this extracted for M=Rx*Ry*Rz and flipped ry's sign, which
        // rotated every gimbal-locked body 180° and displaced the bones.)
        double ry;
        double rz;
        if (m[2][0] < 0.0)
        {
            ry = kPi / 2.0;
            rz = std::atan2(-m[0][1], m[0][2]);
        }
        else
        {
            ry = -kPi / 2.0;
            rz = std::atan2(-m[0][1], m[1][1]);
        }
        out[0] = 0.0;
        out[1] = rad2deg(ry);
        out[2] = rad2deg(rz);
        return;
    }
    double rx = std::atan2(m[2][1], m[2][2]);
    double ry = std::asin(sy);
    double rz = std::atan2(m[1][0], m[0][0]);
    out[0] = rad2deg(rx);
    out[1] = rad2deg(ry);
    out[2] = rad2deg(rz);
}

// Build a btTransform from rest position + Maya XYZ euler degrees.
btTransform transformFromRest(const double pos[3], const double rotDeg[3])
{
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    t.setBasis(btMatrix3x3(eulerDegreesToQuat(rotDeg[0], rotDeg[1], rotDeg[2])));
    return t;
}

btTransform mayaMatrixToBtTransform(const MMatrix& m)
{
    // Maya matrices are ROW-vector (p' = p * M): row r holds the image of the
    // r-th basis vector and m(3, 0..2) is the translation.  Bullet uses
    // COLUMN-vector matrices (v' = M * v), so the same orientation's matrix is
    // the TRANSPOSE of Maya's.  Copying the row matrix directly (as done
    // before) gave every rotated anchor a transposed — i.e. wrong — basis,
    // which yanked the attached rigid chains into a mess.
    btTransform t;
    btMatrix3x3 bm;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            bm[c][r] = m(r, c); // transpose: Bullet column matrix = Maya row^T
    t.setBasis(bm);
    t.setOrigin(btVector3(m(3, 0), m(3, 1), m(3, 2)));
    return t;
}

// Store a Bullet transform as pos + quat (no Bullet type in the node header).
void storeAnchorPose(double pos[3], double quat[4], const btTransform& t)
{
    const btVector3& o = t.getOrigin();
    const btQuaternion& q = t.getRotation();
    pos[0] = o.x();
    pos[1] = o.y();
    pos[2] = o.z();
    quat[0] = q.x();
    quat[1] = q.y();
    quat[2] = q.z();
    quat[3] = q.w();
}

btTransform anchorPoseToTransform(const double pos[3], const double quat[4])
{
    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(btScalar(pos[0]), btScalar(pos[1]), btScalar(pos[2])));
    t.setRotation(
        btQuaternion(btScalar(quat[0]), btScalar(quat[1]), btScalar(quat[2]), btScalar(quat[3])));
    return t;
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
    aBodyGroup = nAttr.create("bodyGroup", "bg", MFnNumericData::kLong, 1, &stat);
    CHECK_MSTATUS(stat);
    aBodyMask = nAttr.create("bodyMask", "bmk", MFnNumericData::kLong, 0xFFFF, &stat);
    CHECK_MSTATUS(stat);
    aBodyKinematic = nAttr.create("bodyKinematic", "bkn", MFnNumericData::kBoolean, false, &stat);
    CHECK_MSTATUS(stat);
    aBodyResetAnchorIndex =
        nAttr.create("bodyResetAnchorIndex", "brai", MFnNumericData::kLong, -1, &stat);
    CHECK_MSTATUS(stat);

    for (MObject* a : {&aBodyRestTranslate, &aBodyRestRotate, &aBodyMass, &aBodyLinearDamping,
                       &aBodyAngularDamping, &aBodyFriction, &aBodyRestitution, &aBodyColliderType,
                       &aBodyRadius, &aBodyExtents, &aBodyLength, &aBodyGroup, &aBodyMask,
                       &aBodyKinematic, &aBodyResetAnchorIndex})
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
    cAttr.addChild(aBodyGroup);
    cAttr.addChild(aBodyMask);
    cAttr.addChild(aBodyKinematic);
    cAttr.addChild(aBodyResetAnchorIndex);

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
        b.solverBodyIndex = -1;

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
        b.group = bodyHandle.child(aBodyGroup).asInt();
        b.mask = bodyHandle.child(aBodyMask).asInt();
        b.kinematic = bodyHandle.child(aBodyKinematic).asBool();
        b.resetAnchorIndex = bodyHandle.child(aBodyResetAnchorIndex).asInt();
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
    const double fps = dataBlock.inputValue(aFps).asDouble();
    const double fixedDt = 1.0 / 60.0;

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
    mWorld->getSolverInfo().m_numIterations = 30;

    // Store fps/fixedDt for stepping (node members).
    (void) fixedDt;

    // Create bodies
    int kinematicCount = 0;
    mAnchorRest.clear();
    mAnchorCurrent.clear();
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        const Body& b = mBodies[i];
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
            mBodies[i].solverBodyIndex = kinematicCount; // anchor index
            // Record the anchor's REST pose (group-local) for scrub-back.
            mAnchorRest.emplace_back(AnchorPose());
            storeAnchorPose(mAnchorRest.back().pos, mAnchorRest.back().quat, start);
            // mAnchorCurrent must be the SAME size as mAnchorRest — it is
            // refreshed every frame in updateKinematicAnchors and drives the
            // scrub-back reset (empty would silently disable the rewind).
            mAnchorCurrent.emplace_back(AnchorPose());
            storeAnchorPose(mAnchorCurrent.back().pos, mAnchorCurrent.back().quat, start);
            ++kinematicCount;
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
        btRigidBody* rbA = mRigidBodies[j.bodyA].get();
        btRigidBody* rbB = mRigidBodies[j.bodyB].get();

        // Joint frame in world, then local to each body.
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
    const unsigned int anchorCount = anchors.elementCount();
    const unsigned int parentInverseCount = parentInverse.elementCount();
    int anchorIndex = 0;
    for (size_t i = 0; i < mBodies.size() && anchorIndex < (int) anchorCount; ++i)
    {
        if (!mBodies[i].kinematic)
            continue;
        anchors.jumpToArrayElement(anchorIndex);
        MMatrix w = anchors.inputValue().asMatrix();
        if (anchorIndex < (int) parentInverseCount)
        {
            parentInverse.jumpToArrayElement(anchorIndex);
            w *= parentInverse.inputValue().asMatrix();
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
        if (mBodies[i].kinematic || !mBodies[i].hasBoneReset)
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
    // Dynamic bodies → outTranslate[i] / outRotate[i] keyed by BODY index
    // (kinematic bodies get no output element; reading them yields defaults).
    MArrayDataBuilder tBuilder(&dataBlock, aOutTranslate, (unsigned int) mBodies.size());
    MArrayDataBuilder rBuilder(&dataBlock, aOutRotate, (unsigned int) mBodies.size());

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        if (mBodies[i].kinematic)
            continue;
        btRigidBody* body = mRigidBodies[i].get();
        const btTransform& wt = body->getWorldTransform();
        const btVector3& o = wt.getOrigin();
        double rot[3];
        quatToEulerXYZDegrees(wt.getRotation(), rot);

        MDataHandle tEl = tBuilder.addElement((unsigned int) i);
        MDataHandle tChild = tEl.child(aOutTranslateValue);
        tChild.set3Double(o.x(), o.y(), o.z());

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
// compute()
// ===========================================================================
MStatus MMDPhysicsNode::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    if (plug != aOutTranslate && plug != aOutRotate && !plug.isElement() && !plug.isChild())
    {
        return MS::kUnknownParameter;
    }

    MDataHandle timeHandle = dataBlock.inputValue(aTime);
    const double now = timeHandle.asTime().value();
    const double fps = dataBlock.inputValue(aFps).asDouble();

    const bool firstEval = !mWorldBuilt;
    if (!mWorldBuilt)
    {
        readBodyData(dataBlock);
        readJointData(dataBlock);
        if (!buildWorld(dataBlock))
            return MS::kFailure;
        mLastTime = now;
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
        double dt = timeChanged ? (now - mLastTime) / fps : 0.0;
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
        dt = std::min(dt, 0.5); // guard against huge jumps
        // Anchor-only movement at a fixed time (a bone dragged in the
        // viewport, no rewind): still run one solver step so the chains are
        // pulled to follow the moved bone (a zero dt makes Bullet skip the
        // solve entirely).  NOT after a rewind though — the reset already
        // teleported the chains (possibly a large distance), and running a
        // solver step right after a big teleport yanks the chains away from
        // their reset pose (catastrophic one-step correction).
        if (!rewound && anchorsMoved && dt <= 0.0)
            dt = 1.0 / 60.0;
        mWorld->stepSimulation(btScalar(dt), 8, btScalar(1.0 / 60.0));
        mLastTime = now;
    }

    writeOutputs(dataBlock);

    if (plug == aOutTranslate || plug.isElement() || plug.isChild())
    {
        // mark translate output clean (already handled by writeOutputs)
    }
    dataBlock.outputValue(aOutTranslate).setClean();
    dataBlock.outputValue(aOutRotate).setClean();
    return MS::kSuccess;
}
