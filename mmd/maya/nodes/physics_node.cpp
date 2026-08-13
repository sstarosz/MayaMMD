/*
 * SPDX-License-Identifier: MIT
 *
 * physics_node.cpp
 *
 * PhysicsNode — native rigid-body physics node.  An MPxLocatorNode that owns
 * a Maya-free Bullet world (mmd::core::Simulation) and advances it in
 * compute() whenever `time1.outTime` changes (the same evaluation path as a
 * parentConstraint, so it runs under Cached Playback).
 *
 * The node is an adapter: it reads the PMX body/joint/gravity attributes into
 * a Simulation::Definition, rebuilds the world when those inputs change or
 * time is scrubbed backwards, steps it when time advances or a kinematic
 * anchor moves, and writes each dynamic body's solved local pose to the
 * outTranslate/outRotate outputs.
 */

#include "physics_node.h"

#include "maya_utils.hpp"

#include <maya/MAngle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDagPath.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MDistance.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MNodeCacheDisablingInfo.h>
#include <maya/MNodeCacheDisablingInfoHelper.h>
#include <maya/MNodeCacheSetupInfo.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MPoint.h>

#include "bullet_bridge.hpp"
#include "physics_math.hpp"

#include <algorithm>
#include <map>
#include <string>

// Pure math (Euler <-> quaternion, row/column transpose) comes from the
// Maya-free physics_math.hpp; the Bullet conversions from bullet_bridge.hpp.
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
MObject PhysicsNode::aAnchorWorldMatrix;
MObject PhysicsNode::aBodyWriteBackOffset;

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
MObject PhysicsNode::aBodyJoint;

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
MObject PhysicsNode::aOutTranslateX;
MObject PhysicsNode::aOutTranslateY;
MObject PhysicsNode::aOutTranslateZ;
MObject PhysicsNode::aOutRotate;
MObject PhysicsNode::aOutRotateX;
MObject PhysicsNode::aOutRotateY;
MObject PhysicsNode::aOutRotateZ;

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
    // Reset to the unbuilt state for an in-place rebuild (see compute()):
    // clear the Bullet world + the cached body/joint data.  The engine owns
    // the Bullet teardown order (world before bodies).
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
    MFnMessageAttribute mMsgAttr;
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

    // --- anchor world matrices ---
    aAnchorWorldMatrix =
        mAttr.create("anchorWorldMatrix", "awm", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setArray(true);
    mAttr.setUsesArrayDataBuilder(true);
    mAttr.setKeyable(false);

    // --- write-back inputs ---
    // The per-body baked write-back offset (K).  This is a TOP-LEVEL matrix
    // array (compound matrix children are awkward), so the node reads it by
    // body index in writeOutputs.
    aBodyWriteBackOffset =
        mAttr.create("bodyWriteBackOffset", "bwo", MFnMatrixAttribute::kDouble, &stat);
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
    // (mmd::core::applyShapeSize) in readBodyData; the draw fallback reads it
    // verbatim.
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
    // = rotation only — the command wires only outRotate for those bodies).
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

    // The body's related joint — a MESSAGE child (mirrors the PMX per-body
    // related_bone_index): pmxRigidBody connects it to the joint.  The node
    // resolves the bone index and the hierarchy from it + the joint DAG.
    // Unconnected = a static collider (no write-back).
    aBodyJoint = mMsgAttr.create("bodyJoint", "bjnt", &stat);
    MMD_CHECK_MSTATUS(stat);
    mMsgAttr.setStorable(true);
    mMsgAttr.setKeyable(false);

    for (MObject* a :
         {&aBodyEnabled, &aBodyShapeSize, &aBodyRestTranslate, &aBodyRestRotate, &aBodyMass,
          &aBodyLinearDamping, &aBodyAngularDamping, &aBodyRestitution, &aBodyFriction})
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
    cAttr.addChild(aBodyJoint);

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
    // Unit-typed compound children (MFnUnitAttribute), exactly like
    // transform.translate/rotate — so the write-back connections to
    // joint.translate / joint.rotate are DIRECT.  A unitless k3Double forced
    // Maya to auto-insert a unitConversion between the float3 output and the
    // joint's angle/linear attributes.
    MFnUnitAttribute uOutAttr;
    aOutTranslateX =
        uOutAttr.create("outTranslateX", "otx", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutTranslateY =
        uOutAttr.create("outTranslateY", "oty", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutTranslateZ =
        uOutAttr.create("outTranslateZ", "otz", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    aOutTranslate = cAttr.create("outTranslate", "otr", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutTranslateX);
    cAttr.addChild(aOutTranslateY);
    cAttr.addChild(aOutTranslateZ);

    aOutRotateX = uOutAttr.create("outRotateX", "orx", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutRotateY = uOutAttr.create("outRotateY", "ory", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutRotateZ = uOutAttr.create("outRotateZ", "orz", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    aOutRotate = cAttr.create("outRotate", "ort", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutRotateX);
    cAttr.addChild(aOutRotateY);
    cAttr.addChild(aOutRotateZ);

    // --- node attribute registration ---
    stat = addAttribute(aTime);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aGravity);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aAnchorWorldMatrix);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyWriteBackOffset);
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

    // Every config input drives the outputs too, so the node is re-evaluated
    // when a body/joint/gravity/anchor input changes (this is what lets
    // compute() detect a config edit) and when a kinematic bone is dragged at
    // a fixed time (the attached chains follow immediately).
    stat = attributeAffects(aGravity, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGravity, aOutRotate);
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
    stat = attributeAffects(aBodyWriteBackOffset, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyWriteBackOffset, aOutRotate);
    MMD_CHECK_MSTATUS(stat);

    return MS::kSuccess;
}

// ===========================================================================
// Data reading
// ===========================================================================
std::vector<Simulation::BodyDefinition> PhysicsNode::readBodyData(MDataBlock& dataBlock)
{
    std::vector<Simulation::BodyDefinition> out;
    MArrayDataHandle bodiesHandle = dataBlock.inputArrayValue(aBodies);
    const unsigned int bodyCount = bodiesHandle.elementCount();
    out.reserve(bodyCount);

    // Per-body related joint paths, resolved from the bodyJoint MESSAGES
    // (kept for the reset-anchor ancestor walk below).
    std::vector<MDagPath> jointPaths(bodyCount);
    MPlug bodiesPlug(thisMObject(), aBodies);

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
        // Keep the full PMX physics mode (0/1/2) — kinematic is a derived
        // property and PHYSICS vs PHYSICS_BONE must stay distinguishable.
        b.physicsMode =
            static_cast<Simulation::PhysicsMode>(bodyHandle.child(aBodyPhysicsMode).asShort());
        b.enabled = bodyHandle.child(aBodyEnabled).asBool();

        // Related joint from the bodyJoint MESSAGE (bodies[i].bodyJoint ->
        // joint.message).  The bone index comes from the joint's pmxBoneIndex;
        // the parent bone from its DAG parent — the DAG IS the hierarchy (the
        // bone builder parents each joint directly under its PMX parent).
        MPlugArray sources;
        bodiesPlug.elementByLogicalIndex(i).child(aBodyJoint).connectedTo(sources, true, false);
        if (sources.length() > 0 &&
            MDagPath::getAPathTo(sources[0].node(), jointPaths[i]) == MS::kSuccess)
        {
            b.relatedBoneIndex = mmd::maya::jointPmxBoneIndex(jointPaths[i]);
            MDagPath parentPath = jointPaths[i];
            if (parentPath.pop() == MS::kSuccess)
                b.parentBoneIndex = mmd::maya::jointPmxBoneIndex(parentPath);
        }
        out.push_back(b);
    }

    // Derive each dynamic body's scrub-back reset anchor: the kinematic-order
    // index of the body on its NEAREST KINEMATIC ANCESTOR bone (walking the
    // joint DAG parents), so hair uses the head anchor, skirt uses the pelvis
    // anchor, etc.  Dynamic bodies without a kinematic ancestor keep -1 (no
    // reset).  This is derived here — there is no per-body reset-anchor input.
    {
        std::map<int, int> boneToAnchor; // bone -> kinematic-order index (first body wins)
        int kinOrder = 0;
        for (const Simulation::BodyDefinition& b : out)
        {
            if (b.isKinematic() && b.enabled && b.relatedBoneIndex >= 0)
            {
                boneToAnchor.emplace(b.relatedBoneIndex, kinOrder);
                ++kinOrder;
            }
        }
        for (size_t i = 0; i < out.size(); ++i)
        {
            Simulation::BodyDefinition& b = out[i];
            if (b.isKinematic() || !b.enabled || b.relatedBoneIndex < 0)
                continue;
            MDagPath jp = jointPaths[i];
            if (!jp.isValid())
                continue;
            std::size_t steps = 0;
            while (steps++ <= 256)
            {
                const int bone = mmd::maya::jointPmxBoneIndex(jp);
                const auto it = boneToAnchor.find(bone);
                if (it != boneToAnchor.end())
                {
                    b.resetAnchorIndex = it->second;
                    break;
                }
                if (jp.pop() != MS::kSuccess)
                    break;
            }
        }
    }

    return out;
}

std::vector<Simulation::JointDefinition> PhysicsNode::readJointData(MDataBlock& dataBlock)
{
    std::vector<Simulation::JointDefinition> out;
    MArrayDataHandle jointsHandle = dataBlock.inputArrayValue(aJoints);
    const unsigned int jointCount = jointsHandle.elementCount();
    out.reserve(jointCount);
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
        out.push_back(j);
    }
    return out;
}

// ===========================================================================
// World construction
// ===========================================================================
bool PhysicsNode::buildWorld(const Double3& gravity,
                             const std::vector<Simulation::BodyDefinition>& bodies,
                             const std::vector<Simulation::JointDefinition>& joints)
{
    // An EMPTY node (no bodies) is a valid no-op — a freshly created node
    // before the commands populate the bodies array.  The callers skip
    // building in that case, so this guard is defensive.
    if (bodies.empty())
        return true;
    // The engine owns every Bullet object — the node only hands it the PMX
    // definition (gravity + bodies + joints) read from the attributes.
    Simulation::Definition definition;
    definition.gravity = gravity;
    definition.bodies = bodies;
    definition.joints = joints;
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
    // The Bullet world runs in WORLD space: the anchor world IS the body's
    // world pose, and the body<->joint rest offset (K^-1) moves the collider
    // onto its bone.
    bool anchorsMoved = false;
    MArrayDataHandle anchors = dataBlock.inputArrayValue(aAnchorWorldMatrix);
    MArrayDataHandle offsetHandle = dataBlock.inputArrayValue(aBodyWriteBackOffset);
    const unsigned int anchorCount = anchors.elementCount();
    int anchorIndex = 0;
    for (size_t i = 0; i < mBodies.size() && anchorIndex < (int) anchorCount; ++i)
    {
        const Simulation::BodyDefinition& b = mBodies[i];
        if (!b.isKinematic() || !b.enabled)
            continue;
        anchors.jumpToArrayElement(anchorIndex);
        MMatrix w = anchors.inputValue().asMatrix();
        // K = bodyWriteBackOffset[i] = jointRestWorld * bodyRestWorld^-1
        // (baked by the command), so the kinematic offset is K^-1.
        if (offsetHandle.jumpToArrayElement((unsigned int) i) == MS::kSuccess)
        {
            w = offsetHandle.inputValue().asMatrix().inverse() * w;
        }
        Simulation::Pose pose;
        const btTransform t = mayaMatrixToBtTransform(w);
        storePose(pose.pos, pose.quat, t);
        if (mSim.setKinematicPose(anchorIndex, pose))
            anchorsMoved = true;
        ++anchorIndex;
    }
    return anchorsMoved;
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
    // Two-pass bone-world write-back — the node builds its OWN temporary
    // skeleton hierarchy and never reads the driven joints from the DG (that
    // was the feedback cycle that exploded the sim):
    //
    //   pass 1 — solved bone world per bone:
    //       solvedBoneWorld[bone] = bodyPose(i) * K_i
    //     for every enabled body with a related bone (kinematic AND dynamic).
    //     A kinematic body's Bullet transform tracks its joint (set to
    //     K^-1 * jointWorld), so its bone world IS the animated joint world —
    //     dynamic chains get their root for free.
    //
    //   pass 2 — joint-local pose via the BONE hierarchy (resolved from the
    //       body's joint message + DAG in readBodyData -> bd.parentBoneIndex):
    //       boneLocal = solvedBoneWorld[parentBone]^-1 * solvedBoneWorld[bone]
    //     At rest this telescopes to jointRest * parentJointRest^-1 (the
    //     joint's exact rest-local pose).
    //
    // Bodies whose parent bone has no body (or no related bone at all) get
    // the RAW solved world pose — matching the old behaviour where a missing
    // parent meant "no write-back, output the body pose".  The command always
    // connects those outputs too; the node falls back to the raw pose for
    // them (rare in well-formed chains).
    //
    // Bullet/btTransform is COLUMN-vector: `bodyPose * K` is the joint world
    // (the transpose of the row-vector K * bodyPose).
    MArrayDataHandle offsetHandle = dataBlock.inputArrayValue(aBodyWriteBackOffset);

    // Dynamic bodies → outTranslate[i] / outRotate[i] keyed by BODY index
    // (kinematic bodies get no output element; reading them yields defaults).
    MArrayDataBuilder tBuilder(&dataBlock, aOutTranslate, (unsigned int) mBodies.size());
    MArrayDataBuilder rBuilder(&dataBlock, aOutRotate, (unsigned int) mBodies.size());

    // Pass 1 — solved bone world per bone.  First body on a bone wins (bodies
    // are created in PMX order, so the lowest body index on a bone drives it).
    // Bodies without K (no command-baked offset) or without a related bone are
    // skipped.
    std::map<int, btTransform> solvedBoneWorld;
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        const Simulation::BodyDefinition& bd = mBodies[i];
        if (!bd.enabled || bd.relatedBoneIndex < 0)
            continue;
        if (solvedBoneWorld.find(bd.relatedBoneIndex) != solvedBoneWorld.end())
            continue; // first body on the bone wins
        if (offsetHandle.jumpToArrayElement((unsigned int) i) != MS::kSuccess)
            continue; // no K baked -> this bone has no derived world
        const btTransform k = mayaMatrixToBtTransform(offsetHandle.inputValue().asMatrix());
        const Simulation::Pose wp = mSim.bodyPose(i);
        solvedBoneWorld.emplace(bd.relatedBoneIndex, poseToTransform(wp.pos, wp.quat) * k);
    }

    // Pass 2 — write the joint-local pose, or the raw solved world pose when
    // the parent bone has no body (the command still connects those outputs;
    // the joint just receives the body's raw world pose — rare in well-formed
    // chains, where every bone in a physics chain has a body).
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        const Simulation::BodyDefinition& bd = mBodies[i];
        if (bd.isKinematic() || !bd.enabled)
            continue;

        const Simulation::Pose wp = mSim.bodyPose(i);
        btTransform boneLocal = poseToTransform(wp.pos, wp.quat);

        const int bone = bd.relatedBoneIndex;
        if (bone >= 0)
        {
            const int parentBone = bd.parentBoneIndex;
            const auto it = solvedBoneWorld.find(bone);
            const auto pit = solvedBoneWorld.find(parentBone);
            if (it != solvedBoneWorld.end() && pit != solvedBoneWorld.end())
            {
                // boneLocal = solvedBoneWorld[parentBone]^-1 * solvedBoneWorld[bone]
                boneLocal = pit->second.inverse() * it->second;
            }
        }
        // No parent bone world -> boneLocal stays the raw solved world pose.

        const btVector3& o = boneLocal.getOrigin();
        Double3 rot;
        const btQuaternion& bq = boneLocal.getRotation();
        quatToEulerXYZDegrees(Double4(bq.x(), bq.y(), bq.z(), bq.w()), rot);

        // PHYSICS writes translate+rotate; PHYSICS_BONE is rotation-only.
        if (bd.physicsMode != Simulation::PhysicsMode::ePhysicsBone)
        {
            MDataHandle tEl = tBuilder.addElement((unsigned int) i);
            tEl.child(aOutTranslateX).setMDistance(MDistance(o.x()));
            tEl.child(aOutTranslateY).setMDistance(MDistance(o.y()));
            tEl.child(aOutTranslateZ).setMDistance(MDistance(o.z()));
        }

        MDataHandle rEl = rBuilder.addElement((unsigned int) i);
        // Written in DEGREES (quatToEulerXYZDegrees output; MAngle's default
        // unit is radians, so the unit must be explicit).
        rEl.child(aOutRotateX).setMAngle(MAngle(rot.x, MAngle::kDegrees));
        rEl.child(aOutRotateY).setMAngle(MAngle(rot.y, MAngle::kDegrees));
        rEl.child(aOutRotateZ).setMAngle(MAngle(rot.z, MAngle::kDegrees));
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
    // rest guides straight from the node's attributes.
    if (mBodies.empty())
    {
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
// Config change detection
// ===========================================================================
Double3 PhysicsNode::readGravity(MDataBlock& dataBlock)
{
    MDataHandle gravHandle = dataBlock.inputValue(aGravity);
    // asDouble3() decays inside the SDK header (same as readDouble3).
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* g = gravHandle.asDouble3();
    return Double3(g[0], g[1], g[2]);
}

std::size_t PhysicsNode::arrayElementCount(MDataBlock& dataBlock, const MObject& attr)
{
    return dataBlock.inputArrayValue(attr).elementCount();
}

bool PhysicsNode::configChanged(const std::vector<Simulation::BodyDefinition>& bodies,
                                const std::vector<Simulation::JointDefinition>& joints,
                                const Double3& gravity, std::size_t anchorCount,
                                std::size_t wbOffsetCount) const
{
    // The anchor/write-back matrix VALUES are per-frame (read fresh in
    // updateKinematicAnchors / writeOutputs), so only their counts define the
    // world structure.
    return bodies != mBodies || joints != mJoints || gravity.x != mGravity.x ||
           gravity.y != mGravity.y || gravity.z != mGravity.z || anchorCount != mAnchorCount ||
           wbOffsetCount != mWriteBackOffsetCount;
}

void PhysicsNode::storeConfig(const std::vector<Simulation::BodyDefinition>& bodies,
                              const std::vector<Simulation::JointDefinition>& joints,
                              const Double3& gravity, std::size_t anchorCount,
                              std::size_t wbOffsetCount)
{
    mBodies = bodies;
    mJoints = joints;
    mGravity = gravity;
    mAnchorCount = anchorCount;
    mWriteBackOffsetCount = wbOffsetCount;
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

    const MTime nowTime = dataBlock.inputValue(aTime).asTime();
    const double now = nowTime.value();

    // Read the config inputs once per evaluation.  They define the world
    // (gravity, bodies, joints) and, compared against what the world was
    // built with, drive the rebuild-vs-step decision below.
    const std::vector<Simulation::BodyDefinition> bodies = readBodyData(dataBlock);
    const std::vector<Simulation::JointDefinition> joints = readJointData(dataBlock);
    const Double3 gravity = readGravity(dataBlock);
    const std::size_t anchorCount = arrayElementCount(dataBlock, aAnchorWorldMatrix);
    const std::size_t wbOffsetCount = arrayElementCount(dataBlock, aBodyWriteBackOffset);

    // Refresh the kinematic anchors every evaluation (the colliders track
    // their bones even at a fixed time) and report whether any moved.  No-op
    // while the world is not built.
    const bool anchorsMoved = updateKinematicAnchors(dataBlock);

    if (!mSim.initialized())
    {
        // First evaluation: build the world from the current config at rest.
        // An empty node (no bodies) is a valid no-op.
        if (!bodies.empty() && !buildWorld(gravity, bodies, joints))
            return MS::kFailure;
        storeConfig(bodies, joints, gravity, anchorCount, wbOffsetCount);
        mLastTime = now;
        mLastTimeUnit = nowTime.unit();
    }
    else
    {
        const double dt = (nowTime - MTime(mLastTime, mLastTimeUnit)).as(MTime::kSeconds);
        if (configChanged(bodies, joints, gravity, anchorCount, wbOffsetCount) || dt < 0.0)
        {
            // A config edit (a body/joint/gravity input changed, or a changed
            // anchor/write-back count) or a scrub backwards — rebuild in place
            // at the CURRENT skeleton pose.  A fresh world carries no stale
            // solver warm-start state, so the chains are not yanked.
            destroyWorld();
            if (!bodies.empty() && !buildWorld(gravity, bodies, joints))
                return MS::kFailure;
            storeConfig(bodies, joints, gravity, anchorCount, wbOffsetCount);
            updateKinematicAnchors(dataBlock); // re-apply current anchors
            mSim.resetDynamicBodies();         // chains stay at the current pose
            mLastTime = now;                   // no time-step on the rebuild frame
            mLastTimeUnit = nowTime.unit();
        }
        else if (dt > 0.0)
        {
            // Time advanced — step by the frame span in seconds (via MTime,
            // which adapts to the scene's playback unit).
            mSim.step(dt);
            mLastTime = now;
            mLastTimeUnit = nowTime.unit();
        }
        else if (anchorsMoved)
        {
            // A bone was dragged at the current frame — one fixed tick so the
            // attached chains follow immediately.
            mSim.step(Simulation::kFixedDt);
        }
    }

    writeOutputs(dataBlock);

    dataBlock.outputValue(aOutTranslate).setClean();
    dataBlock.outputValue(aOutRotate).setClean();
    return MS::kSuccess;
}
