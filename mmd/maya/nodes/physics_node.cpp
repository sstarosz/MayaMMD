/*
 * SPDX-License-Identifier: MIT
 *
 * physics_node.cpp
 *
 * PhysicsNode — native rigid-body physics node (embedded Bullet 3.25).
 *
 * See physics_node.h for the design rationale (replaces the mayaBullet
 * dynamic layer which froze under Cached Playback because its solver is a
 * stateful node the evaluation cache does not re-step).
 *
 * The Bullet world lives inside this node and advances in compute() every time
 * `time1.outTime` changes — the same evaluation path as a parentConstraint, so
 * it runs under Cached Playback / the evaluation manager.
 */

#include "physics_node.h"

#include "maya_utils.hpp"

#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnData.h>
#include <maya/MFnEnumAttribute.h>
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

#include "bullet_bridge.hpp"
#include "physics_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

// The pure math (Euler <-> quaternion, row/column matrix transpose) lives in
// the Maya-free physics_math.hpp so it can be unit-tested without the Maya
// SDK; the Bullet-facing conversions live in bullet_bridge.hpp (the only
// core header that exposes Bullet types).
using namespace mmd::core::physics_math;
using mmd::core::applyShapeSize;
using mmd::core::Double3;
using mmd::core::Double4;
using mmd::core::Matrix4;
using mmd::core::shapeSizeFromBodyDefinition;
using mmd::core::Simulation;

// ===========================================================================
// Constants
// ===========================================================================
const MTypeId PhysicsNode::kTypeId(0x0011C105); // unique Maya node type id for pmxPhysicsNode

// Simulation constants + joint-type mapping moved into the Maya-free engine
// (mmd_simulation.cpp) — the node only builds a Definition, steps the sim and
// reads solved poses.

// ===========================================================================
// Attribute declarations
// ===========================================================================
MObject PhysicsNode::aTime;
MObject PhysicsNode::aGravity;
MObject PhysicsNode::aConfigVersion;
MObject PhysicsNode::aAnchorWorldMatrix;
MObject PhysicsNode::aGroupInverseWorldMatrix;
MObject PhysicsNode::aAnchorOffset;
MObject PhysicsNode::aGroupWorldMatrix;
MObject PhysicsNode::aBodyWriteBackOffset;
MObject PhysicsNode::aBodyParentInverseMatrix;

MObject PhysicsNode::aBodies;
MObject PhysicsNode::aBodyEnabled;
MObject PhysicsNode::aBodyNameLocal;
MObject PhysicsNode::aBodyNameUniversal;
MObject PhysicsNode::aBodyGroupId;
std::array<MObject, 16> PhysicsNode::aBodyMaskGroup;
MObject PhysicsNode::aBodyColliderType;
MObject PhysicsNode::aBodyShapeSize;
MObject PhysicsNode::aBodyRestTranslate;
MObject PhysicsNode::aBodyRestRotate;
MObject PhysicsNode::aBodyMass;
MObject PhysicsNode::aBodyLinearDamping;
MObject PhysicsNode::aBodyAngularDamping;
MObject PhysicsNode::aBodyRestitution;
MObject PhysicsNode::aBodyFriction;
MObject PhysicsNode::aBodyPhysicsMode;
MObject PhysicsNode::aBodyParentBodyIndex;
MObject PhysicsNode::aBodyResetAnchorIndex;

MObject PhysicsNode::aJoints;
MObject PhysicsNode::aJointNameLocal;
MObject PhysicsNode::aJointNameUniversal;
MObject PhysicsNode::aJointBodyA;
MObject PhysicsNode::aJointBodyB;
MObject PhysicsNode::aJointType;
MObject PhysicsNode::aJointFrameTranslate;
MObject PhysicsNode::aJointFrameRotate;
MObject PhysicsNode::aJointLinearMin;
MObject PhysicsNode::aJointLinearMax;
MObject PhysicsNode::aJointAngularMin;
MObject PhysicsNode::aJointAngularMax;
MObject PhysicsNode::aJointLinearSpring;
MObject PhysicsNode::aJointAngularSpring;

MObject PhysicsNode::aOutTranslate;
MObject PhysicsNode::aOutTranslateValue;
MObject PhysicsNode::aOutRotate;
MObject PhysicsNode::aOutRotateValue;

// ===========================================================================
// Maya-specific conversion (the shared pure math is in physics_math.hpp)
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
    // itself is the (unit-tested) mmd::core::physics_math::doubleMatrixToBtTransform;
    // this wrapper only adapts MMatrix's accessor.
    Matrix4 mm;
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            mm(r, c) = m(r, c);
        }
    }
    return doubleMatrixToBtTransform(mm);
}

// MMatrix has no constructor from the core Matrix4 (only from a C array) — the
// C array is required by the Maya API boundary, so the bounds checks on the
// two lines below (loop-indexed subscript + array-to-pointer decay) do not
// apply to this bridge.
MMatrix matrix4ToMMatrix(const Matrix4& m)
{
    double tmp[4][4] = {};
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            tmp[r][c] = m(r, c);
        }
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    return MMatrix(tmp);
}

// Read a k3Double child attribute into a core Double3.  The Maya API returns a
// `const double*` to three elements; we write into the named members — never
// past them (the old .data() + out[0..2] pattern was out-of-bounds access).
// (asDouble3() is flagged by the decay check because the decay happens inside
// the Maya SDK header — NOLINT on that single line.)
void readDouble3(MDataHandle& hd, const MObject& attr, Double3& out)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* v = hd.child(attr).asDouble3();
    out.x = v[0];
    out.y = v[1];
    out.z = v[2];
}

// Map the node's persisted attribute enum (kColliderBox=1..kColliderCapsule=3)
// to the engine's PMX-aligned enum (eSphere=0..eCapsule=2).  The attribute
// values are stored in scenes, so they cannot change; the engine enum matches
// the PMX ShapeType byte instead — casting would silently swap sphere/capsule.
Simulation::ColliderType colliderToEngine(short v)
{
    switch (v)
    {
    case PhysicsNode::kColliderBox:
        return Simulation::ColliderType::eBox;
    case PhysicsNode::kColliderSphere:
        return Simulation::ColliderType::eSphere;
    default:
        return Simulation::ColliderType::eCapsule; // kColliderCapsule
    }
}

// Inverse of colliderToEngine — engine enum -> node attribute enum.
PhysicsNode::ColliderType colliderFromEngine(Simulation::ColliderType v)
{
    switch (v)
    {
    case Simulation::ColliderType::eSphere:
        return PhysicsNode::kColliderSphere;
    case Simulation::ColliderType::eBox:
        return PhysicsNode::kColliderBox;
    default:
        return PhysicsNode::kColliderCapsule; // eCapsule
    }
}

// Read one body element's attributes into a DrawBody.  Used by
// collectDrawData to draw the REST guides even before the first compute()
// (mBodies is only filled lazily on first evaluation) — so the colliders are
// visible immediately after import and whenever the solver is not being pulled
// by the DG.
// (The asDouble3() calls are NOLINT'd — the decay happens inside the Maya SDK
// header, not in this file.)
void readDrawBodyFromPlug(const MPlug& el, PhysicsNode::DrawBody& db)
{
    db.colliderType =
        static_cast<PhysicsNode::ColliderType>(el.child(PhysicsNode::aBodyColliderType).asShort());
    // PMX shape_size VERBATIM (full size) — the draw contract reads it
    // directly and derives the primitive by collider type.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* s = el.child(PhysicsNode::aBodyShapeSize).asMDataHandle().asDouble3();
    db.shapeSize[0] = s[0];
    db.shapeSize[1] = s[1];
    db.shapeSize[2] = s[2];
    db.kinematic = (el.child(PhysicsNode::aBodyPhysicsMode).asShort() ==
                    static_cast<short>(Simulation::PhysicsMode::eFollowBone));
    // group id straight from the raw PMX id (the Bullet group bit is derived
    // from it in buildWorld); clamp legacy scenes where it is -1.
    db.groupId = el.child(PhysicsNode::aBodyGroupId).asShort();
    db.groupId = std::max(db.groupId, 0); // clamp legacy -1
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* p = el.child(PhysicsNode::aBodyRestTranslate).asMDataHandle().asDouble3();
    db.pos[0] = p[0];
    db.pos[1] = p[1];
    db.pos[2] = p[2];
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* r = el.child(PhysicsNode::aBodyRestRotate).asMDataHandle().asDouble3();
    const Double4 q = eulerDegreesToQuat(r[0], r[1], r[2]);
    db.quat[0] = q.x;
    db.quat[1] = q.y;
    db.quat[2] = q.z;
    db.quat[3] = q.w;
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

uint64_t hashDouble3(uint64_t h, const Double3& v)
{
    h = hashValue(h, v.x);
    h = hashValue(h, v.y);
    return hashValue(h, v.z);
}

} // namespace

// ===========================================================================
// Node lifecycle
// ===========================================================================
PhysicsNode::PhysicsNode() = default;

// Defaulted: the node is destroyed polymorphically through its MPxNode base
// (Maya deletes it via the base pointer).  The default teardown is exactly
// what we want — mSim (Simulation) tears down the Bullet world in its own
// PIMPL destructor, mBodies/mJoints are plain vectors, and the scalars are
// trivial.  The only explicit teardown is destroyWorld() below, used to reset
// to the unbuilt state for a rebuild.
PhysicsNode::~PhysicsNode() = default;

void PhysicsNode::destroyWorld()
{
    // Reset to the unbuilt state for an in-place rebuild (see
    // rebuildSimulationAtCurrentPose): clear the Bullet world + the cached
    // body/joint data.  The engine owns the Bullet teardown order (world
    // before bodies).
    mSim.clear();
    mLastTime = -1.0;
    mLastTimeUnit = MTime::kFilm;
    mBodies.clear();
    mJoints.clear();
}

void* PhysicsNode::creator()
{
    return new PhysicsNode();
}

// ===========================================================================
// Attribute registration
// ===========================================================================
MStatus PhysicsNode::initialize()
{
    MFnNumericAttribute nAttr;
    MFnCompoundAttribute cAttr;
    MFnMatrixAttribute mAttr;
    MFnUnitAttribute uAttr;
    MStatus stat;

    // --- time ---
    aTime = uAttr.create("time", "tm", MFnUnitAttribute::kTime, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    uAttr.setStorable(true);
    uAttr.setKeyable(true);
    uAttr.setHidden(true);

    // --- gravity ---
    aGravity = nAttr.create("gravity", "grav", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    nAttr.setDefault(0.0, -9.8, 0.0); // MMD's physics engine uses exactly -9.8
    nAttr.setStorable(true);
    nAttr.setKeyable(false);

    // --- configVersion ---
    // Hidden forced-rebuild trigger (see physics_node.h).  Bumping it changes
    // the config signature, so compute() rebuilds the Bullet world even when
    // no other input changed.  dt is derived from the scene's time unit via
    // MTime, so the old `fps` attribute (which only ever served as this
    // trigger) is gone.
    aConfigVersion = nAttr.create("configVersion", "cfgv", MFnNumericData::kLong, 0, &stat);
    MMD_CHECK_MSTATUS(stat);
    nAttr.setStorable(true);
    nAttr.setHidden(true);
    nAttr.setKeyable(false);

    // --- anchor world matrices ---
    aAnchorWorldMatrix =
        mAttr.create("anchorWorldMatrix", "awm", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    // The physics group's world inverse (single) — the SAME matrix every
    // kinematic anchor used to receive per-anchor as anchorParentInverseMatrix.
    // local = world * groupInverseWorldMatrix puts each anchor in the group's
    // local space (the Bullet world frame); leaving it unconnected treats the
    // anchors as world space (identity).
    aGroupInverseWorldMatrix =
        mAttr.create("groupInverseWorldMatrix", "giwm", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setKeyable(false);

    // Phase 3: baked world-frame offset per kinematic anchor
    // (colliderRestWorld * jointRestWorld^-1) so the collider tracks the JOINT
    // with the PMX body<->bone offset preserved.  Indexed by kinematic order,
    // 1:1 with anchorWorldMatrix.
    aAnchorOffset = mAttr.create("anchorOffset", "aof", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    // Phase 3 direct write-back: the physics group's world matrix (single) and
    // the per-dynamic-body baked offset + related-joint parent inverse.  These
    // are TOP-LEVEL matrix arrays (compound matrix children are awkward), so
    // the node reads them by body index in writeOutputs.
    aGroupWorldMatrix = mAttr.create("groupWorldMatrix", "gwm", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setKeyable(false);

    aBodyWriteBackOffset =
        mAttr.create("bodyWriteBackOffset", "bwo", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    aBodyParentInverseMatrix =
        mAttr.create("bodyParentInverseMatrix", "bpim", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    // --- body compound ---
    // Body-compound children are created in PMX order (mirrors the
    // rigid_bodies.json fields); aBodyEnabled (a Maya-only custom attribute)
    // comes first.  The node reads everything by attribute name, so the order
    // is purely for the Attribute Editor / listAttr readability.
    aBodyEnabled = nAttr.create("bodyEnabled", "ben", MFnNumericData::kBoolean, 1.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    // PMX body names (local + universal — Query/UI display; the node itself
    // never reads them, they just need to be storable attributes).
    MFnTypedAttribute tAttr;
    aBodyNameLocal =
        tAttr.create("bodyNameLocal", "bnml", MFnData::kString, MObject::kNullObj, &stat);
    MMD_CHECK_MSTATUS(stat);
    tAttr.setStorable(true);
    tAttr.setKeyable(false);
    aBodyNameUniversal =
        tAttr.create("bodyNameUniversal", "bnmu", MFnData::kString, MObject::kNullObj, &stat);
    MMD_CHECK_MSTATUS(stat);
    tAttr.setStorable(true);
    tAttr.setKeyable(false);

    // PMX collision group — enum: one field per group (0..15, "Group 0"..
    // "Group 15").  The Bullet group bit (1 << groupId) is derived in
    // buildWorld.  Read back via .asShort() like any other numeric attribute.
    {
        MFnEnumAttribute eAttr;
        aBodyGroupId = eAttr.create("bodyGroupId", "bgid", 0, &stat);
        MMD_CHECK_MSTATUS(stat);
        for (int g = 0; g < 16; ++g)
        {
            MString name(("Group " + std::to_string(g)).c_str());
            eAttr.addField(name, static_cast<short>(g));
        }
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

    // THE collision mask: one boolean toggle per collision group (0..15),
    // True = collides with that group (default 0xFFFF = everything).  This is
    // the PMX non_collision_group field stored VERBATIM (bit i set = the body
    // collides with group i — MMD feeds it to Bullet directly, no inversion),
    // and the node uses it exactly as read.
    for (int g = 0; g < 16; ++g)
    {
        MString lname(("bodyMaskGroup" + std::to_string(g)).c_str());
        MString sname(("bmg" + std::to_string(g)).c_str());
        aBodyMaskGroup.at(g) = nAttr.create(lname, sname, MFnNumericData::kBoolean, 1.0, &stat);
        MMD_CHECK_MSTATUS(stat);
    }

    // PMX collider type — enum: box / sphere / capsule (field values match
    // PhysicsNode::ColliderType).  Field names mirror the enumerators.
    // Read back via .asShort() like any other numeric attribute.
    {
        MFnEnumAttribute eAttr;
        aBodyColliderType = eAttr.create("bodyColliderType", "bct", kColliderBox, &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("Box", kColliderBox);
        eAttr.addField("Sphere", kColliderSphere);
        eAttr.addField("Capsule", kColliderCapsule);
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }
    // PMX shape_size VERBATIM (3 doubles, full size).  The node derives the
    // engine's radius / box half-extents / capsule length by collider type
    // (mmd::core::applyShapeSize) wherever a body is read.
    aBodyShapeSize = nAttr.create("bodyShapeSize", "bss", MFnNumericData::k3Double, 1.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    aBodyRestTranslate =
        nAttr.create("bodyRestTranslate", "brt", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aBodyRestRotate = nAttr.create("bodyRestRotate", "brr", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    aBodyMass = nAttr.create("bodyMass", "bm", MFnNumericData::kDouble, 1.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aBodyLinearDamping =
        nAttr.create("bodyLinearDamping", "bld", MFnNumericData::kDouble, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aBodyAngularDamping =
        nAttr.create("bodyAngularDamping", "bad", MFnNumericData::kDouble, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aBodyRestitution = nAttr.create("bodyRestitution", "bre", MFnNumericData::kDouble, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aBodyFriction = nAttr.create("bodyFriction", "bfr", MFnNumericData::kDouble, 0.5, &stat);
    MMD_CHECK_MSTATUS(stat);

    // PMX physics mode — enum: followBone / physics / physicsBone (field
    // values match Simulation::PhysicsMode).  Field names mirror the
    // enumerators.  The node writes the joint-local pose for mode 1/2 (mode 2
    // = rotation only — Python connects only outRotate for those bodies).
    {
        MFnEnumAttribute eAttr;
        aBodyPhysicsMode = eAttr.create(
            "bodyPhysicsMode", "bpm", static_cast<short>(Simulation::PhysicsMode::ePhysics), &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("FollowBone", static_cast<short>(Simulation::PhysicsMode::eFollowBone));
        eAttr.addField("Physics", static_cast<short>(Simulation::PhysicsMode::ePhysics));
        eAttr.addField("PhysicsBone", static_cast<short>(Simulation::PhysicsMode::ePhysicsBone));
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

    // Derived / wiring fields (no PMX JSON counterpart).
    // Rigid-body index of the related joint's PARENT joint's body (the
    // write-back derives the parent inverse from that body's solved Bullet
    // transform); -1 = parent bone has no body (DG parentInverse fallback).
    aBodyParentBodyIndex =
        nAttr.create("bodyParentBodyIndex", "bpbi", MFnNumericData::kShort, -1, &stat);
    MMD_CHECK_MSTATUS(stat);
    aBodyResetAnchorIndex =
        nAttr.create("bodyResetAnchorIndex", "brai", MFnNumericData::kLong, -1, &stat);
    MMD_CHECK_MSTATUS(stat);

    for (MObject* a : {&aBodyEnabled, &aBodyShapeSize, &aBodyRestTranslate, &aBodyRestRotate,
                       &aBodyMass, &aBodyLinearDamping, &aBodyAngularDamping, &aBodyRestitution,
                       &aBodyFriction, &aBodyParentBodyIndex, &aBodyResetAnchorIndex})
    {
        MFnNumericAttribute fn(*a);
        fn.setStorable(true);
        fn.setKeyable(false);
    }
    for (int g = 0; g < 16; ++g)
    {
        MFnNumericAttribute fn(aBodyMaskGroup.at(g));
        fn.setStorable(true);
        fn.setKeyable(false);
    }

    aBodies = cAttr.create("bodies", "bds", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setStorable(true);
    cAttr.setKeyable(false);
    cAttr.addChild(aBodyEnabled);
    cAttr.addChild(aBodyNameLocal);
    cAttr.addChild(aBodyNameUniversal);
    cAttr.addChild(aBodyGroupId);
    for (int g = 0; g < 16; ++g)
        cAttr.addChild(aBodyMaskGroup.at(g));
    cAttr.addChild(aBodyColliderType);
    cAttr.addChild(aBodyShapeSize);
    cAttr.addChild(aBodyRestTranslate);
    cAttr.addChild(aBodyRestRotate);
    cAttr.addChild(aBodyMass);
    cAttr.addChild(aBodyLinearDamping);
    cAttr.addChild(aBodyAngularDamping);
    cAttr.addChild(aBodyRestitution);
    cAttr.addChild(aBodyFriction);
    cAttr.addChild(aBodyPhysicsMode);
    cAttr.addChild(aBodyParentBodyIndex);
    cAttr.addChild(aBodyResetAnchorIndex);

    // --- joint compound ---
    aJointNameLocal =
        tAttr.create("jointNameLocal", "jnml", MFnData::kString, MObject::kNullObj, &stat);
    MMD_CHECK_MSTATUS(stat);
    tAttr.setStorable(true);
    tAttr.setKeyable(false);
    aJointNameUniversal =
        tAttr.create("jointNameUniversal", "jnmu", MFnData::kString, MObject::kNullObj, &stat);
    MMD_CHECK_MSTATUS(stat);
    tAttr.setStorable(true);
    tAttr.setKeyable(false);
    aJointBodyA = nAttr.create("jointBodyA", "jba", MFnNumericData::kLong, 0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aJointBodyB = nAttr.create("jointBodyB", "jbb", MFnNumericData::kLong, 0, &stat);
    MMD_CHECK_MSTATUS(stat);
    // PMX joint type — enum: one field per PMX JointType (0..5, values match
    // the PMX JointType enum).  Field names mirror the enumerators.  Read
    // back via .asShort() like any other numeric attribute.
    {
        MFnEnumAttribute eAttr;
        aJointType = eAttr.create("jointType", "jt", 0, &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("Spring6Dof", 0);
        eAttr.addField("SixDof", 1);
        eAttr.addField("P2P", 2);
        eAttr.addField("ConeTwist", 3);
        eAttr.addField("Slider", 4);
        eAttr.addField("Hinge", 5);
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }
    aJointFrameTranslate =
        nAttr.create("jointFrameTranslate", "jft", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aJointFrameRotate =
        nAttr.create("jointFrameRotate", "jfr", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aJointLinearMin = nAttr.create("jointLinearMin", "jlmn", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aJointLinearMax = nAttr.create("jointLinearMax", "jlmx", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aJointAngularMin =
        nAttr.create("jointAngularMin", "jamn", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aJointAngularMax =
        nAttr.create("jointAngularMax", "jamx", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aJointLinearSpring =
        nAttr.create("jointLinearSpring", "jls", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aJointAngularSpring =
        nAttr.create("jointAngularSpring", "jas", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    for (MObject* a : {&aJointBodyA, &aJointBodyB, &aJointFrameTranslate, &aJointFrameRotate,
                       &aJointLinearMin, &aJointLinearMax, &aJointAngularMin, &aJointAngularMax,
                       &aJointLinearSpring, &aJointAngularSpring})
    {
        MFnNumericAttribute fn(*a);
        fn.setStorable(true);
        fn.setKeyable(false);
    }

    aJoints = cAttr.create("joints", "jns", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setStorable(true);
    cAttr.setKeyable(false);
    cAttr.addChild(aJointNameLocal);
    cAttr.addChild(aJointNameUniversal);
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
    MMD_CHECK_MSTATUS(stat);
    MFnNumericAttribute otFn(aOutTranslateValue);
    otFn.setWritable(false);
    otFn.setStorable(false);

    aOutTranslate = cAttr.create("outTranslate", "otr", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutTranslateValue);

    aOutRotateValue = nAttr.create("outRotateValue", "orv", MFnNumericData::k3Double, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    MFnNumericAttribute orFn(aOutRotateValue);
    orFn.setWritable(false);
    orFn.setStorable(false);

    aOutRotate = cAttr.create("outRotate", "ort", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutRotateValue);

    // --- node attribute registration ---
    stat = addAttribute(aTime);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aGravity);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aConfigVersion);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aAnchorWorldMatrix);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aGroupInverseWorldMatrix);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aAnchorOffset);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aGroupWorldMatrix);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyWriteBackOffset);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyParentInverseMatrix);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodies);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJoints);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutRotate);
    MMD_CHECK_MSTATUS(stat);

    // Make `time` drive the outputs.
    stat = attributeAffects(aTime, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aTime, aOutRotate);
    MMD_CHECK_MSTATUS(stat);

    // Phase 4: every config input drives the outputs too, so the node is
    // re-evaluated when a body/joint/gravity/anchor input changes — that is
    // what lets compute() detect the config edit and rebuild the Bullet world.
    // (The anchor matrix VALUES change every frame; declaring the dependency
    // also makes a kinematic bone dragged at a fixed time re-evaluate the node
    // so the attached chains follow immediately.)
    stat = attributeAffects(aGravity, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGravity, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aConfigVersion, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aConfigVersion, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodies, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodies, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorWorldMatrix, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorWorldMatrix, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGroupInverseWorldMatrix, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGroupInverseWorldMatrix, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorOffset, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorOffset, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGroupWorldMatrix, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGroupWorldMatrix, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyWriteBackOffset, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyWriteBackOffset, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyParentInverseMatrix, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyParentInverseMatrix, aOutRotate);
    MMD_CHECK_MSTATUS(stat);

    return MS::kSuccess;
}

// ===========================================================================
// Data reading
// ===========================================================================
bool PhysicsNode::readBodyData(MDataBlock& dataBlock)
{
    mBodies.clear();
    MArrayDataHandle bodiesHandle = dataBlock.inputArrayValue(aBodies);
    const unsigned int bodyCount = bodiesHandle.elementCount();
    for (unsigned int i = 0; i < bodyCount; ++i)
    {
        bodiesHandle.jumpToArrayElement(i);
        MDataHandle bodyHandle = bodiesHandle.inputValue();
        Simulation::BodyDefinition b;
        readDouble3(bodyHandle, aBodyRestTranslate, b.restPos);
        readDouble3(bodyHandle, aBodyRestRotate, b.restRot);
        b.mass = bodyHandle.child(aBodyMass).asDouble();
        b.linearDamping = bodyHandle.child(aBodyLinearDamping).asDouble();
        b.angularDamping = bodyHandle.child(aBodyAngularDamping).asDouble();
        b.friction = bodyHandle.child(aBodyFriction).asDouble();
        b.restitution = bodyHandle.child(aBodyRestitution).asDouble();
        b.colliderType = colliderToEngine(bodyHandle.child(aBodyColliderType).asShort());
        Double3 shapeSize;
        readDouble3(bodyHandle, aBodyShapeSize, shapeSize);
        applyShapeSize(b, shapeSize); // PMX shape_size -> engine radius/extents/length
        b.mask = 0;
        for (int g = 0; g < 16; ++g)
            if (bodyHandle.child(aBodyMaskGroup.at(g)).asBool())
                b.mask |= 1L << g;
        b.groupId = bodyHandle.child(aBodyGroupId).asShort();
        // KEEP the full PMX physics mode (0/1/2) — kinematic is a derived
        // property (BodyDefinition::isKinematic()) and PHYSICS vs
        // PHYSICS_BONE must stay distinguishable downstream.
        b.physicsMode =
            static_cast<Simulation::PhysicsMode>(bodyHandle.child(aBodyPhysicsMode).asShort());
        b.parentBodyIndex = bodyHandle.child(aBodyParentBodyIndex).asShort();
        b.resetAnchorIndex = bodyHandle.child(aBodyResetAnchorIndex).asInt();
        b.enabled = bodyHandle.child(aBodyEnabled).asBool();
        mBodies.push_back(b);
    }
    return !mBodies.empty();
}

bool PhysicsNode::readJointData(MDataBlock& dataBlock)
{
    mJoints.clear();
    MArrayDataHandle jointsHandle = dataBlock.inputArrayValue(aJoints);
    const unsigned int jointCount = jointsHandle.elementCount();
    for (unsigned int i = 0; i < jointCount; ++i)
    {
        jointsHandle.jumpToArrayElement(i);
        MDataHandle jointHandle = jointsHandle.inputValue();
        Simulation::JointDefinition j;
        j.bodyA = jointHandle.child(aJointBodyA).asInt();
        j.bodyB = jointHandle.child(aJointBodyB).asInt();
        j.type = jointHandle.child(aJointType).asInt();
        readDouble3(jointHandle, aJointFrameTranslate, j.frameT);
        readDouble3(jointHandle, aJointFrameRotate, j.frameR);
        readDouble3(jointHandle, aJointLinearMin, j.linearMin);
        readDouble3(jointHandle, aJointLinearMax, j.linearMax);
        readDouble3(jointHandle, aJointAngularMin, j.angularMin);
        readDouble3(jointHandle, aJointAngularMax, j.angularMax);
        readDouble3(jointHandle, aJointLinearSpring, j.linearSpring);
        readDouble3(jointHandle, aJointAngularSpring, j.angularSpring);
        mJoints.push_back(j);
    }
    return true;
}

// ===========================================================================
// Config signature (Phase 4)
// ===========================================================================
// The node rebuilds the Bullet world when the user edits any of the inputs
// that DEFINE it: gravity, configVersion, the bodies/joints arrays (values AND
// counts), and the number of kinematic anchors.  The anchor matrix VALUES are
// deliberately excluded — they change every frame — only their counts matter.
// Mass, damping, friction, restitution, collider size, group/mask, joint
// limits and springs are all baked into the Bullet construction info at build
// time, so an edit only takes effect after a rebuild; hashing them lets
// compute() detect the edit and rebuild in place.
uint64_t PhysicsNode::computeConfigSignature(MDataBlock& dataBlock)
{
    uint64_t h = 0xcbf29ce484222325ULL; // FNV-1a offset basis

    // gravity + configVersion
    MDataHandle grav = dataBlock.inputValue(aGravity);
    // asDouble3() decays to a C array inside the SDK; hashDouble3 reads it by
    // const pointer — the decay is unavoidable at the Maya API boundary.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    h = hashDouble3(h, grav.asDouble3());
    h = hashValue(h, dataBlock.inputValue(aConfigVersion).asLong());

    // bodies
    MArrayDataHandle bodiesHandle = dataBlock.inputArrayValue(aBodies);
    const unsigned int bodyCount = bodiesHandle.elementCount();
    h = hashValue(h, bodyCount);
    Double3 v3;
    for (unsigned int i = 0; i < bodyCount; ++i)
    {
        bodiesHandle.jumpToArrayElement(i);
        MDataHandle bh = bodiesHandle.inputValue();
        readDouble3(bh, aBodyRestTranslate, v3);
        h = hashDouble3(h, v3);
        readDouble3(bh, aBodyRestRotate, v3);
        h = hashDouble3(h, v3);
        h = hashValue(h, bh.child(aBodyMass).asDouble());
        h = hashValue(h, bh.child(aBodyLinearDamping).asDouble());
        h = hashValue(h, bh.child(aBodyAngularDamping).asDouble());
        h = hashValue(h, bh.child(aBodyFriction).asDouble());
        h = hashValue(h, bh.child(aBodyRestitution).asDouble());
        h = hashValue(h, bh.child(aBodyColliderType).asShort());
        readDouble3(bh, aBodyShapeSize, v3);
        h = hashDouble3(h, v3);
        // One hash input per collision-group toggle (any edit rebuilds).
        for (int g = 0; g < 16; ++g)
            h = hashValue(h, bh.child(aBodyMaskGroup.at(g)).asBool());
        h = hashValue(h, bh.child(aBodyGroupId).asShort());
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
        readDouble3(jh, aJointFrameTranslate, v3);
        h = hashDouble3(h, v3);
        readDouble3(jh, aJointFrameRotate, v3);
        h = hashDouble3(h, v3);
        readDouble3(jh, aJointLinearMin, v3);
        h = hashDouble3(h, v3);
        readDouble3(jh, aJointLinearMax, v3);
        h = hashDouble3(h, v3);
        readDouble3(jh, aJointAngularMin, v3);
        h = hashDouble3(h, v3);
        readDouble3(jh, aJointAngularMax, v3);
        h = hashDouble3(h, v3);
        readDouble3(jh, aJointLinearSpring, v3);
        h = hashDouble3(h, v3);
        readDouble3(jh, aJointAngularSpring, v3);
        h = hashDouble3(h, v3);
    }

    // anchor counts (the values are per-frame; the counts define the world
    // structure — adding/removing a kinematic anchor is a config change)
    MArrayDataHandle anchors = dataBlock.inputArrayValue(aAnchorWorldMatrix);
    h = hashValue(h, anchors.elementCount());
    // Phase 3 write-back arrays: only the COUNTS (the offset matrices are
    // baked constants; the parent-inverse matrices vary every frame).
    MArrayDataHandle anchorOffset = dataBlock.inputArrayValue(aAnchorOffset);
    h = hashValue(h, anchorOffset.elementCount());
    MArrayDataHandle wbOffset = dataBlock.inputArrayValue(aBodyWriteBackOffset);
    h = hashValue(h, wbOffset.elementCount());
    MArrayDataHandle wbParentInv = dataBlock.inputArrayValue(aBodyParentInverseMatrix);
    h = hashValue(h, wbParentInv.elementCount());

    return h;
}

// ===========================================================================
// World construction
// ===========================================================================
bool PhysicsNode::buildWorld(MDataBlock& dataBlock)
{
    if (mSim.initialized())
        return true;
    // An EMPTY node (no bodies) is a valid no-op state — the node exists in the
    // scene but has nothing to simulate.  Treat it as success so compute() stays
    // inert instead of failing on every evaluation (this is the normal state for
    // a freshly-created node before the rigid-body commands populate the bodies
    // array).
    if (mBodies.empty())
        return true;

    // NOTE: do NOT call destroyWorld() here — it clears mBodies/mJoints which
    // were just read from the datablock.  The caller guarantees the world is
    // not built when this runs.

    // The engine owns every Bullet object — the node only hands it the PMX
    // definition (gravity + bodies + joints) read from the attributes.
    Simulation::Definition definition;
    MDataHandle gravHandle = dataBlock.inputValue(aGravity);
    // asDouble3() decays inside the SDK header (same as readDouble3).
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* g = gravHandle.asDouble3();
    definition.gravity = Double3(g[0], g[1], g[2]);
    definition.bodies = mBodies;
    definition.joints = mJoints;
    return mSim.initialize(definition);
}

// ===========================================================================
// Per-frame update
// ===========================================================================
bool PhysicsNode::updateKinematicAnchors(MDataBlock& dataBlock)
{
    if (!mSim.initialized())
        return false;
    // anchorWorldMatrix[i] maps 1:1 to the kinematic bodies in body order.
    // local = world * groupInverseWorldMatrix (row-vector convention) — the
    // Bullet world runs in the physics group's local space.  The group-inverse
    // is a SINGLE matrix applied to every anchor (previously each anchor
    // carried its own parentInverse); unconnected = identity = world space.
    bool anchorsMoved = false;
    MArrayDataHandle anchors = dataBlock.inputArrayValue(aAnchorWorldMatrix);
    MArrayDataHandle anchorOffset = dataBlock.inputArrayValue(aAnchorOffset);
    const unsigned int anchorCount = anchors.elementCount();
    const unsigned int offsetCount = anchorOffset.elementCount();
    MMatrix groupInverse;
    MPlug groupInversePlug(thisMObject(), aGroupInverseWorldMatrix);
    if (groupInversePlug.isConnected())
    {
        groupInverse = dataBlock.inputValue(aGroupInverseWorldMatrix).asMatrix();
    }
    else
    {
        groupInverse.setToIdentity();
    }
    int anchorIndex = 0;
    for (size_t i = 0; i < mBodies.size() && anchorIndex < (int) anchorCount; ++i)
    {
        const Simulation::BodyDefinition& b = mBodies[i];
        if (!b.isKinematic() || !b.enabled)
            continue;
        anchors.jumpToArrayElement(anchorIndex);
        MMatrix w = anchors.inputValue().asMatrix();
        w *= groupInverse;
        // Phase 3: apply the baked world-frame offset (colliderRestWorld *
        // jointRestWorld^-1) so the kinematic collider tracks the JOINT with
        // the PMX body<->bone offset preserved (this is exactly what the old
        // parentConstraint(joint, guide, maintainOffset) maintained — verified
        // empirically: targetWorld = K * sourceWorld, K constant).  world here
        // is the JOINT's world matrix and groupInverse is the physics GROUP's
        // world inverse, so world * groupInverse is the joint in group space.
        if (anchorIndex < (int) offsetCount)
        {
            anchorOffset.jumpToArrayElement(anchorIndex);
            w = anchorOffset.inputValue().asMatrix() * w;
        }
        // Convert the anchor's group-local pose to the engine's Pose — the sim
        // sets the Bullet transform, tracks the current pose and detects
        // movement (a bone dragged at the current frame).
        Simulation::Pose pose;
        const btTransform t = mayaMatrixToBtTransform(w);
        const btVector3& o = t.getOrigin();
        const btQuaternion& q = t.getRotation();
        pose.pos = Double3(o.x(), o.y(), o.z());
        pose.quat = Double4(q.x(), q.y(), q.z(), q.w());
        if (mSim.setKinematicPose(anchorIndex, pose))
            anchorsMoved = true;
        ++anchorIndex;
    }
    return anchorsMoved;
}

void PhysicsNode::resetDynamicBodies(MDataBlock& dataBlock)
{
    // Teleport every dynamic body (that has a reset anchor) to its rest pose
    // transformed by the CURRENT skeleton pose, zeroing velocities — the
    // engine owns the reset math (Simulation::resetDynamicBodies).
    (void) dataBlock;
    mSim.resetDynamicBodies();
}

void PhysicsNode::getCacheSetup(const MEvaluationNode& evalNode,
                                MNodeCacheDisablingInfo& disablingInfo,
                                MNodeCacheSetupInfo& setupInfo,
                                MObjectArray& monitoredAttributes) const
{
    // This node advances an internal Bullet world in compute(), so its outputs
    // are NOT a pure function of its inputs.  Cached Playback must re-evaluate
    // it every frame, exactly like a scripted/expression node.
    MString category("pmxPhysicsNode: stateful Bullet solver (steps every frame)");
    MNodeCacheDisablingInfoHelper::setUnsafeNode(disablingInfo, evalNode, &category);
    MPxLocatorNode::getCacheSetup(evalNode, disablingInfo, setupInfo, monitoredAttributes);
}

bool PhysicsNode::writeOutputs(MDataBlock& dataBlock)
{
    // Phase 3 direct write-back: the node outputs the JOINT-LOCAL pose so
    // Python can connect outTranslate/outRotate straight into the joints (no
    // guide transforms, no parent/orientConstraints).  The primary transform
    // is
    //   boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1
    // where K = jointRestWorld * bodyRestWorld^-1 (baked by pmxRigidBody
    // -create) and the parent inverse is derived from the PARENT BODY's
    // solved Bullet transform (M_parent * B_parent * groupWorld =
    // parentJointWorld, M_parent = parentJointRestWorld * parentBodyRestWorld^-1
    // — the SAME constant as K[parentBodyIndex]).  This is
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

    // Dynamic bodies → outTranslate[i] / outRotate[i] keyed by BODY index
    // (kinematic bodies get no output element; reading them yields defaults).
    MArrayDataBuilder tBuilder(&dataBlock, aOutTranslate, (unsigned int) mBodies.size());
    MArrayDataBuilder rBuilder(&dataBlock, aOutRotate, (unsigned int) mBodies.size());

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        const Simulation::BodyDefinition& bd = mBodies[i];
        if (bd.isKinematic() || !bd.enabled)
            continue;
        // Start from the solved group-space body pose (Maya row-vector matrix).
        const Simulation::Pose wp = mSim.bodyPose(i);
        Matrix4 outRow;
        btTransformToRowMatrix(poseToTransform(wp.pos, wp.quat), outRow);

        // PRIMARY write-back path (Phase 3 cycle fix): the parent inverse is
        // derived from the PARENT BODY's solved Bullet transform, never from
        // the DG.  For a body whose parent JOINT is also node-driven the old
        // `joint.parentInverseMatrix` dependency created a DG feedback cycle
        // (parentJoint.worldMatrix <- node.outRotate <- node.compute <- ...
        // <- parentJoint.parentInverseMatrix) that made the simulation
        // explode.  Here:
        //   boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1
        // with K = bodyWriteBackOffset[i] (jointRestWorld * bodyRestWorld^-1)
        // and M_parent = K[parentBodyIndex] (parentJointRestWorld *
        // parentBodyRestWorld^-1 — the same constant as the parent body's K,
        // for kinematic and dynamic parents).  Because parentJointWorld =
        // M_parent * B_parent * groupWorld, the groupWorld term cancels and
        // boneLocal is EXACT at rest for both parent kinds (verified
        // algebraically).
        const int parentIdx = bd.parentBodyIndex;
        if (parentIdx >= 0 && (size_t) parentIdx < mBodies.size() && mBodies[parentIdx].enabled &&
            offsetHandle.jumpToArrayElement((unsigned int) i) == MS::kSuccess &&
            offsetHandle.jumpToArrayElement((unsigned int) parentIdx) == MS::kSuccess)
        {
            // The two jumps in the condition above leave the handle at the
            // LAST element (parentIdx) — re-position explicitly before each
            // read so k = K[i] and mp = K[parentIdx].  (Reading K[parentIdx]
            // for BOTH was the bug that displaced every dynamic bone: the
            // joint-world terms cancelled and boneLocal collapsed to
            // bodyLocal * B_parent^-1.)
            offsetHandle.jumpToArrayElement((unsigned int) i);
            MMatrix k = offsetHandle.inputValue().asMatrix();
            // M_parent = parentJointRestWorld * parentBodyRestWorld^-1 is the
            // SAME constant as K[parentIdx] (bodyWriteBackOffset of the parent
            // body — baked at create for kinematic AND dynamic parents), so a
            // separate parent-offset array is not needed.
            offsetHandle.jumpToArrayElement((unsigned int) parentIdx);
            MMatrix mp = offsetHandle.inputValue().asMatrix();
            // B_parent = the PARENT BODY's solved Bullet transform (never the
            // DG joint matrix — that was the feedback-cycle fix).
            const Simulation::Pose pp = mSim.bodyPose(parentIdx);
            Matrix4 bpRow;
            btTransformToRowMatrix(poseToTransform(pp.pos, pp.quat), bpRow);
            MMatrix bParent(matrix4ToMMatrix(bpRow));
            MMatrix bodyLocal(matrix4ToMMatrix(outRow));
            MMatrix result = k * bodyLocal * bParent.inverse() * mp.inverse();
            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    outRow(r, c) = result(r, c);
                }
            }
        }
        else if (haveGroupWorld &&
                 offsetHandle.jumpToArrayElement((unsigned int) i) == MS::kSuccess)
        {
            // FALLBACK (parent bone has no rigid body, or an old scene): the
            // original formula with the DG parent-inverse input.  Only used
            // when the parent joint is NOT node-driven (its bone has no body
            // and no dynamic ancestor), so it cannot feed back into the node.
            MMatrix k = offsetHandle.inputValue().asMatrix();
            Matrix4 kRow;
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    kRow(r, c) = k(r, c);
            Matrix4 tmp;
            rowMatrixMultiply(kRow, outRow, tmp);
            Matrix4 gw;
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    gw(r, c) = groupWorld(r, c);
            Matrix4 tmp2;
            rowMatrixMultiply(tmp, gw, tmp2);
            if (parentInvHandle.jumpToArrayElement((unsigned int) i) == MS::kSuccess)
            {
                MMatrix pi = parentInvHandle.inputValue().asMatrix();
                Matrix4 piRow;
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c)
                        piRow(r, c) = pi(r, c);
                rowMatrixMultiply(tmp2, piRow, outRow);
            }
            else
            {
                outRow = tmp2;
            }
        }

        const double ox = outRow(3, 0); // row-vector translation
        const double oy = outRow(3, 1);
        const double oz = outRow(3, 2);
        Double3 rot;
        const btTransform boneLocal = doubleMatrixToBtTransform(outRow);
        const btQuaternion& bq = boneLocal.getRotation();
        quatToEulerXYZDegrees(Double4(bq.x(), bq.y(), bq.z(), bq.w()), rot);

        // Mode-aware write-back — the MMD reference (blender_mmd_tools' rigid
        // track = "COPY_TRANSFORMS"/"COPY_ROTATION" per mode):
        //   PHYSICS (1)      -> the bone follows the body fully (translate + rotate)
        //   PHYSICS_BONE (2) -> the bone keeps its animated position and receives
        //                      only the body's rotation (rotate only)
        if (bd.physicsMode != Simulation::PhysicsMode::ePhysicsBone)
        {
            MDataHandle tEl = tBuilder.addElement((unsigned int) i);
            MDataHandle tChild = tEl.child(aOutTranslateValue);
            tChild.set3Double(ox, oy, oz);
        }

        MDataHandle rEl = rBuilder.addElement((unsigned int) i);
        MDataHandle rChild = rEl.child(aOutRotateValue);
        rChild.set3Double(rot.x, rot.y, rot.z);
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
void PhysicsNode::collectDrawData(std::vector<DrawBody>& out) const
{
    out.clear();
    // Before the first compute() the internal body state is empty — draw the
    // REST guides straight from the node's attributes so the colliders are
    // visible immediately after import (and whenever nothing pulls the DG).
    if (mBodies.empty())
    {
        // Fallback reads straight from the attributes (before the first
        // compute) — mirror the solved path by skipping disabled bodies.
        MPlug bodiesPlug(thisMObject(), aBodies);
        const unsigned int n = bodiesPlug.evaluateNumElements();
        for (unsigned int i = 0; i < n; ++i)
        {
            MPlug el = bodiesPlug.elementByLogicalIndex(i);
            if (!el.child(aBodyEnabled).asBool())
                continue;
            DrawBody db;
            readDrawBodyFromPlug(el, db);
            out.push_back(db);
        }
        return;
    }
    out.reserve(mBodies.size());
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        const Simulation::BodyDefinition& b = mBodies[i];
        if (!b.enabled)
            continue;
        DrawBody db;
        db.colliderType = colliderFromEngine(b.colliderType);
        const Double3 size = shapeSizeFromBodyDefinition(b);
        db.shapeSize[0] = size.x;
        db.shapeSize[1] = size.y;
        db.shapeSize[2] = size.z;
        db.kinematic = b.isKinematic();
        // group id straight from the raw PMX group id (clamp legacy -1).
        db.groupId = b.groupId >= 0 ? b.groupId : 0;
        if (mSim.initialized())
        {
            // Solved pose — what the simulation actually has right now.
            const Simulation::Pose p = mSim.bodyPose(i);
            db.pos[0] = p.pos.x;
            db.pos[1] = p.pos.y;
            db.pos[2] = p.pos.z;
            db.quat[0] = p.quat.x;
            db.quat[1] = p.quat.y;
            db.quat[2] = p.quat.z;
            db.quat[3] = p.quat.w;
        }
        else
        {
            // World not built yet — draw the PMX rest pose.
            db.pos[0] = b.restPos.x;
            db.pos[1] = b.restPos.y;
            db.pos[2] = b.restPos.z;
            const Double4 q = eulerDegreesToQuat(b.restRot.x, b.restRot.y, b.restRot.z);
            db.quat[0] = q.x;
            db.quat[1] = q.y;
            db.quat[2] = q.z;
            db.quat[3] = q.w;
        }
        out.push_back(db);
    }
}

MBoundingBox PhysicsNode::boundingBox() const
{
    MBoundingBox box;
    bool any = false;
    for (const Simulation::BodyDefinition& b : mBodies)
    {
        double r = 0.0;
        if (b.colliderType == Simulation::ColliderType::eSphere)
            r = b.radius;
        else if (b.colliderType == Simulation::ColliderType::eBox)
            r = std::max({b.extents.x, b.extents.y, b.extents.z});
        else
            r = b.radius + (b.length * 0.5);
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
// Timeline/state helpers — see SimulationTransition in physics_node.h.
// ===========================================================================
PhysicsNode::SimulationTransition PhysicsNode::classifyTransition(uint64_t configSignature,
                                                                  const MTime& nowTime, double now,
                                                                  bool anchorsMoved) const
{
    if (!mSim.initialized())
        return SimulationTransition::Initialize;
    if (configSignature != mConfigSignature)
        return SimulationTransition::ConfigurationChanged;
    if (mLastTime < 0.0 || now == mLastTime)
        return anchorsMoved ? SimulationTransition::PoseChanged : SimulationTransition::NoChange;
    // Time moved — forwards (Advance) or backwards (Rewind)?
    const double dt = (nowTime - MTime(mLastTime, mLastTimeUnit)).as(MTime::kSeconds);
    return dt < 0.0 ? SimulationTransition::Rewind : SimulationTransition::Advance;
}

bool PhysicsNode::initializeSimulation(MDataBlock& dataBlock, uint64_t configSignature,
                                       const MTime& nowTime)
{
    readBodyData(dataBlock);
    readJointData(dataBlock);
    if (!buildWorld(dataBlock))
        return false;
    mConfigSignature = configSignature;
    mLastTime = nowTime.value();
    mLastTimeUnit = nowTime.unit();
    return true;
}

bool PhysicsNode::rebuildSimulationAtCurrentPose(MDataBlock& dataBlock, uint64_t configSignature,
                                                 const MTime& nowTime)
{
    // Shared by ConfigurationChanged and Rewind: rebuild the Bullet world from
    // the CURRENT skeleton pose — an in-place config edit must not teleport the
    // chains to the PMX rest pose, and a rewind must not carry stale solver
    // warm-start state into the reset.  (The old world is discarded entirely,
    // so its anchor state is not "captured" — the anchors are re-applied to
    // the fresh world below.)
    destroyWorld();
    readBodyData(dataBlock);
    readJointData(dataBlock);
    if (!buildWorld(dataBlock))
        return false;
    mConfigSignature = configSignature;
    updateKinematicAnchors(dataBlock); // re-apply current anchors to the fresh world
    resetDynamicBodies(dataBlock);     // chains stay at the current pose
    mLastTime = nowTime.value();       // no time-step on the rebuild frame
    mLastTimeUnit = nowTime.unit();
    return true;
}

// ===========================================================================
// compute()
// ===========================================================================
MStatus PhysicsNode::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    if (plug != aOutTranslate && plug != aOutRotate && !plug.isElement() && !plug.isChild())
    {
        return MS::kUnknownParameter;
    }

    MDataHandle timeHandle = dataBlock.inputValue(aTime);
    const MTime nowTime = timeHandle.asTime();
    const double now = nowTime.value();
    const uint64_t configSignature = computeConfigSignature(dataBlock);

    // Refresh the kinematic anchors every evaluation (so the colliders track
    // their bones even at a fixed time) and detect whether any moved since the
    // previous step.  No-op while the world is not built (on the very first
    // eval the anchor inputs may not be ready yet).
    const bool anchorsMoved = updateKinematicAnchors(dataBlock);

    // Timeline/state machine: classify this evaluation into exactly one
    // transition and act on it (see SimulationTransition).
    switch (classifyTransition(configSignature, nowTime, now, anchorsMoved))
    {
    case SimulationTransition::Initialize:
        // First evaluation — read the attributes and build the world.
        if (!initializeSimulation(dataBlock, configSignature, nowTime))
            return MS::kFailure;
        break;

    case SimulationTransition::ConfigurationChanged:
    case SimulationTransition::Rewind:
        // A config edit (gravity/fps/bodies/joints/anchor counts) or a scrub
        // backwards — rebuild in place, keeping the dynamic chains glued to
        // the CURRENT skeleton pose (a fresh world carries no stale solver
        // warm-start state, so the chains do not get yanked after a rewind).
        if (!rebuildSimulationAtCurrentPose(dataBlock, configSignature, nowTime))
            return MS::kFailure;
        break;

    case SimulationTransition::Advance:
        // Time moved forward — step by the frame span in SECONDS (via MTime,
        // which adapts to the scene's playback unit — no fps attribute needed).
        mSim.step((nowTime - MTime(mLastTime, mLastTimeUnit)).as(MTime::kSeconds));
        mLastTime = now;
        mLastTimeUnit = nowTime.unit();
        break;

    case SimulationTransition::PoseChanged:
        // A bone was dragged at the current frame — run one fixed tick so the
        // attached chains follow immediately (MMD reacts to bone changes at
        // once, not on the next frame).
        mSim.step(Simulation::kFixedDt);
        break;

    case SimulationTransition::NoChange:
        break;
    }

    writeOutputs(dataBlock);

    dataBlock.outputValue(aOutTranslate).setClean();
    dataBlock.outputValue(aOutRotate).setClean();
    return MS::kSuccess;
}
