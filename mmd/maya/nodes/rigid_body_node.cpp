/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_node.cpp
 *
 * RigidBodyNode — native rigid-body physics node.  An MPxLocatorNode that owns
 * a Maya-free Bullet world (mmd::core::RigidBodySimulation) and advances it in
 * compute() whenever `time1.outTime` changes (the same evaluation path as a
 * parentConstraint, so it runs under Cached Playback).
 *
 * The node is an adapter: it reads the PMX body/joint/gravity attributes into
 * a RigidBodySimulation::Definition, rebuilds the world when those inputs
 * change or time is scrubbed backwards, steps it when time advances or a
 * kinematic anchor moves, and writes each dynamic body's solved local pose to
 * the outTranslate/outRotate outputs.
 */

#include "rigid_body_node.hpp"

#include "maya_utils.hpp"

#include <maya/MAngle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDagPath.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MDistance.h>
#include <maya/MEulerRotation.h>
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
#include <maya/MQuaternion.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>

#include "bullet_bridge.hpp"
#include "physics_math.hpp"
#include "rigid_body_simulation.hpp"

#include <map>
#include <optional>
#include <string>

// Pure math (Euler <-> quaternion, row/column transpose) comes from the
// Maya-free physics_math.hpp; the Bullet conversions from bullet_bridge.hpp.
using namespace mmd::core::physics_math;
using mmd::core::applyShapeSize;
using mmd::core::Double3;
using mmd::core::Double4;
using mmd::core::Matrix4;
using mmd::core::RigidBodySimulation;
using World = RigidBodyNode::World; // the node's state record (see the header)

// ===========================================================================
// Node type + attribute storage
// ===========================================================================
const MTypeId RigidBodyNode::kTypeId(0x0011C105); // unique Maya node type id for pmxRigidBodyNode

// ===========================================================================
// Attribute declarations
// ===========================================================================
MObject RigidBodyNode::aTime;
MObject RigidBodyNode::aGravity;

MObject RigidBodyNode::aBodies;
MObject RigidBodyNode::aBodyEnabled;
MObject RigidBodyNode::aBodyNameLocal;
MObject RigidBodyNode::aBodyNameUniversal;
MObject RigidBodyNode::aBodyGroupId;
std::array<MObject, 16> RigidBodyNode::aBodyMaskGroup;
MObject RigidBodyNode::aBodyColliderType;
MObject RigidBodyNode::aBodyShapeSize;
MObject RigidBodyNode::aBodyRestTranslate;
MObject RigidBodyNode::aBodyRestRotate;
MObject RigidBodyNode::aBodyMass;
MObject RigidBodyNode::aBodyLinearDamping;
MObject RigidBodyNode::aBodyAngularDamping;
MObject RigidBodyNode::aBodyRestitution;
MObject RigidBodyNode::aBodyFriction;
MObject RigidBodyNode::aBodyPhysicsMode;
MObject RigidBodyNode::aBodyJoint;
MObject RigidBodyNode::aBodyAnchorWorld;

MObject RigidBodyNode::aJoints;
MObject RigidBodyNode::aJointNameLocal;
MObject RigidBodyNode::aJointNameUniversal;
MObject RigidBodyNode::aJointBodyA;
MObject RigidBodyNode::aJointBodyB;
MObject RigidBodyNode::aJointType;
MObject RigidBodyNode::aJointFrameTranslate;
MObject RigidBodyNode::aJointFrameRotate;
MObject RigidBodyNode::aJointLinearMin;
MObject RigidBodyNode::aJointLinearMax;
MObject RigidBodyNode::aJointAngularMin;
MObject RigidBodyNode::aJointAngularMax;
MObject RigidBodyNode::aJointLinearSpring;
MObject RigidBodyNode::aJointAngularSpring;

MObject RigidBodyNode::aOutTranslate;
MObject RigidBodyNode::aOutTranslateX;
MObject RigidBodyNode::aOutTranslateY;
MObject RigidBodyNode::aOutTranslateZ;
MObject RigidBodyNode::aOutRotate;
MObject RigidBodyNode::aOutRotateX;
MObject RigidBodyNode::aOutRotateY;
MObject RigidBodyNode::aOutRotateZ;

// ===========================================================================
// Per-evaluation inputs (transient — not part of the node's state)
// ===========================================================================
// The PMX-verbatim inputs for one evaluation; World (RigidBodyNode::World,
// see the header) is the built state derived from them.
struct Inputs
{
    std::vector<mmd::core::RigidBodySimulation::BodyDefinition> bodies;
    std::vector<mmd::core::RigidBodySimulation::JointDefinition> joints;
    mmd::core::Double3 gravity;
};

// ===========================================================================
// File-local helpers (pure attribute/plugin reading — no node state)
// ===========================================================================
namespace
{

// Maya matrices are ROW-vector (p' = p * M): row r holds the image of the r-th
// basis vector and m(3, 0..2) is the translation.  Bullet uses COLUMN-vector
// matrices (v' = M * v), so the same orientation is the TRANSPOSE of Maya's —
// the (unit-tested) mmd::core::physics_math::doubleMatrixToBtTransform.  This
// wrapper only adapts MMatrix's accessor.
[[nodiscard]] btTransform mayaMatrixToBtTransform(const MMatrix& m)
{
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

// Read a k3Double attribute handle into a core Double3.  asDouble3() decays
// inside the Maya SDK header — hence the NOLINT on that single line.
[[nodiscard]] Double3 readDouble3(const MDataHandle& hd)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* v = hd.asDouble3();
    return Double3(v[0], v[1], v[2]);
}

// The joint's rest LOCAL matrix, reconstructed from the bone builder's
// captured rest pose (pmxRestTranslate/Rotate + static jointOrient) with
// MTransformationMatrix so the composition matches Maya's joint localMatrix
// exactly.  Missing attributes read as 0 (mock joints in tests carry none).
[[nodiscard]] MMatrix jointRestLocalMatrix(const MDagPath& jointPath)
{
    MFnDependencyNode fn(jointPath.node());
    const auto read = [&fn](const char* name) -> double
    {
        MStatus stat;
        const MPlug plug = fn.findPlug(name, true, &stat);
        return plug.isNull() ? 0.0 : plug.asDouble();
    };
    const double tx = read("pmxRestTranslateX");
    const double ty = read("pmxRestTranslateY");
    const double tz = read("pmxRestTranslateZ");
    const double ox = MAngle(read("jointOrientX"), MAngle::kDegrees).asRadians();
    const double oy = MAngle(read("jointOrientY"), MAngle::kDegrees).asRadians();
    const double oz = MAngle(read("jointOrientZ"), MAngle::kDegrees).asRadians();
    const double rx = MAngle(read("pmxRestRotateX"), MAngle::kDegrees).asRadians();
    const double ry = MAngle(read("pmxRestRotateY"), MAngle::kDegrees).asRadians();
    const double rz = MAngle(read("pmxRestRotateZ"), MAngle::kDegrees).asRadians();
    MTransformationMatrix tm;
    tm.setTranslation(MVector(tx, ty, tz), MSpace::kTransform);
    tm.setRotationOrientation(MEulerRotation(ox, oy, oz).asQuaternion());
    // Rotation order kXYZ matches the joints' default rotateOrder (0).
    const double rot[3] = {rx, ry, rz};
    tm.setRotation(&rot[0], MTransformationMatrix::kXYZ);
    return tm.asMatrix();
}

// The joint's rest WORLD matrix: compose the rest-local matrices up the DAG
// (row-vector: world = local * parentWorld); the model root contributes
// nothing (no captured rest attributes -> identity).  `cache` memoizes rest
// worlds per pmxBoneIndex so chains sharing ancestors are walked once.
[[nodiscard]] MMatrix jointRestWorldMatrix(const MDagPath& jointPath, std::map<int, MMatrix>& cache)
{
    std::vector<MDagPath> chain;
    MDagPath p = jointPath;
    MMatrix world; // identity — the model root sits at identity/origin
    while (true)
    {
        const int bone = mmd::maya::jointPmxBoneIndex(p);
        if (bone >= 0)
        {
            const auto it = cache.find(bone);
            if (it != cache.end())
            {
                world = it->second;
                break;
            }
        }
        chain.push_back(p);
        if (p.pop() != MS::kSuccess)
            break;
    }
    // Compose from the cached ancestor (or identity at the root) down to the
    // joint, caching every intermediate rest world by its bone index.
    for (auto it = chain.rbegin(); it != chain.rend(); ++it)
    {
        world = jointRestLocalMatrix(*it) * world;
        const int bone = mmd::maya::jointPmxBoneIndex(*it);
        if (bone >= 0)
            cache.emplace(bone, world);
    }
    return world;
}

// Map the node's persisted attribute enum (kColliderBox=1..kColliderCapsule=3)
// to the engine's PMX-aligned enum (eSphere=0..eCapsule=2).  The attribute
// values are stored in scenes, so they cannot change; the engine enum matches
// the PMX ShapeType byte instead — casting would silently swap sphere/capsule.
[[nodiscard]] RigidBodySimulation::ColliderType colliderToEngine(short v)
{
    switch (v)
    {
    case RigidBodyNode::kColliderBox:
        return RigidBodySimulation::ColliderType::eBox;
    case RigidBodyNode::kColliderSphere:
        return RigidBodySimulation::ColliderType::eSphere;
    default:
        return RigidBodySimulation::ColliderType::eCapsule; // kColliderCapsule
    }
}

// Resolve the body's related joint (bodyJoint message) into the joint's DAG
// path + PMX bone indices; returns false for a body with no connected joint
// (a static collider).  The DAG IS the PMX bone hierarchy — the bone builder
// parents each joint directly under its PMX parent.  Runs once per world
// build via resolveBodyWiring (~5 DG API calls per body); the result is
// cached in the world's bodies and not re-checked per frame — re-binding a
// body or re-parenting a joint needs a rebuild trigger (e.g. re-import).
bool resolveRelatedBones(const MPlug& bodiesPlug, unsigned int index, MDagPath& jointPath,
                         int& boneIndex, int& parentBoneIndex)
{
    boneIndex = -1;
    parentBoneIndex = -1;
    MPlugArray sources;
    bodiesPlug.elementByLogicalIndex(index)
        .child(RigidBodyNode::aBodyJoint)
        .connectedTo(sources, true, false);
    if (sources.length() == 0 || MDagPath::getAPathTo(sources[0].node(), jointPath) != MS::kSuccess)
        return false;
    boneIndex = mmd::maya::jointPmxBoneIndex(jointPath);
    MDagPath parentPath = jointPath;
    if (parentPath.pop() == MS::kSuccess)
        parentBoneIndex = mmd::maya::jointPmxBoneIndex(parentPath);
    return true;
}

// Derive each dynamic body's scrub-back reset anchor: the kinematic-order
// index of the body on its NEAREST KINEMATIC ANCESTOR bone (walking the joint
// DAG parents), so hair uses the head anchor, skirt uses the pelvis anchor,
// etc.  Bodies without a kinematic ancestor keep -1 (no reset).  The kinematic
// order counts EVERY enabled kinematic body — a boneless FOLLOW_BONE body pins
// its own rest world but still occupies a kinematic slot.
void deriveResetAnchors(std::vector<RigidBodySimulation::BodyDefinition>& bodies,
                        const std::vector<MDagPath>& jointPaths)
{
    std::map<int, int> boneToAnchor; // bone -> kinematic-order index (first body wins)
    int kinOrder = 0;
    for (const RigidBodySimulation::BodyDefinition& b : bodies)
    {
        if (!b.isKinematic() || !b.enabled)
            continue;
        if (b.relatedBoneIndex >= 0)
            boneToAnchor.emplace(b.relatedBoneIndex, kinOrder);
        ++kinOrder;
    }
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        RigidBodySimulation::BodyDefinition& b = bodies[i];
        if (b.isKinematic() || !b.enabled || b.relatedBoneIndex < 0)
            continue;
        MDagPath jp = jointPaths[i];
        if (!jp.isValid())
            continue;
        for (size_t steps = 0; steps < 256; ++steps) // cycle guard
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

// Read one body element's PMX-verbatim fields.  The DAG-derived wiring
// (relatedBoneIndex / parentBoneIndex / resetAnchorIndex) stays -1 here —
// resolveBodyWiring fills it at world build.
[[nodiscard]] RigidBodySimulation::BodyDefinition readBody(MDataHandle& hd)
{
    RigidBodySimulation::BodyDefinition b;
    b.restPos = readDouble3(hd.child(RigidBodyNode::aBodyRestTranslate));
    b.restRot = readDouble3(hd.child(RigidBodyNode::aBodyRestRotate));
    b.mass = hd.child(RigidBodyNode::aBodyMass).asDouble();
    b.linearDamping = hd.child(RigidBodyNode::aBodyLinearDamping).asDouble();
    b.angularDamping = hd.child(RigidBodyNode::aBodyAngularDamping).asDouble();
    b.friction = hd.child(RigidBodyNode::aBodyFriction).asDouble();
    b.restitution = hd.child(RigidBodyNode::aBodyRestitution).asDouble();
    b.colliderType = colliderToEngine(hd.child(RigidBodyNode::aBodyColliderType).asShort());
    const Double3 shapeSize = readDouble3(hd.child(RigidBodyNode::aBodyShapeSize));
    applyShapeSize(b, shapeSize); // PMX shape_size -> engine radius/extents/length
    b.mask = 0;
    for (int g = 0; g < 16; ++g)
        if (hd.child(RigidBodyNode::aBodyMaskGroup.at(g)).asBool())
            b.mask |= 1L << g;
    b.groupId = hd.child(RigidBodyNode::aBodyGroupId).asShort();
    // Keep the full PMX physics mode (0/1/2) — PHYSICS vs PHYSICS_BONE must
    // stay distinguishable; kinematic is a derived property.
    b.physicsMode = static_cast<RigidBodySimulation::PhysicsMode>(
        hd.child(RigidBodyNode::aBodyPhysicsMode).asShort());
    b.enabled = hd.child(RigidBodyNode::aBodyEnabled).asBool();
    return b;
}

// Read the node's bodies array (PMX-verbatim fields only).  The array is
// written DENSELY by the commands (auto-append), so the element position
// (jumpToArrayElement) and the logical index line up.
[[nodiscard]] std::vector<RigidBodySimulation::BodyDefinition> readBodyData(MDataBlock& dataBlock)
{
    std::vector<RigidBodySimulation::BodyDefinition> out;
    MArrayDataHandle bodiesHandle = dataBlock.inputArrayValue(RigidBodyNode::aBodies);
    const unsigned int bodyCount = bodiesHandle.elementCount();
    out.reserve(bodyCount);
    for (unsigned int i = 0; i < bodyCount; ++i)
    {
        bodiesHandle.jumpToArrayElement(i);
        MDataHandle element = bodiesHandle.inputValue();
        out.push_back(readBody(element));
    }
    return out;
}

// Resolve each body's DAG-derived wiring: the related joint (bodyJoint
// message) into its DAG path + PMX bone indices, and the scrub-back reset
// anchor.  Called only when the world is (re)built — these are structural
// facts of the imported skeleton, computed once and cached in the world's
// bodies rather than re-resolved every evaluation (re-import to change them).
// `jointPaths` is also consumed by deriveWriteBackOffsets (one resolution
// pass per build).
void resolveBodyWiring(const MObject& node,
                       std::vector<RigidBodySimulation::BodyDefinition>& bodies,
                       std::vector<MDagPath>& jointPaths)
{
    jointPaths.assign(bodies.size(), MDagPath());
    const MPlug bodiesPlug(node, RigidBodyNode::aBodies);
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        resolveRelatedBones(bodiesPlug, (unsigned int) i, jointPaths[i], bodies[i].relatedBoneIndex,
                            bodies[i].parentBoneIndex);
    }
    deriveResetAnchors(bodies, jointPaths);
}

// PMX-verbatim field equality for config-change detection.  The DAG-derived
// wiring (relatedBoneIndex / parentBoneIndex / resetAnchorIndex) is resolved
// once per world build and cached in the world's bodies — it is deliberately
// NOT part of the comparison (re-binding a body or re-parenting a joint needs
// a rebuild trigger, e.g. a re-import).
bool sameBodyFields(const std::vector<RigidBodySimulation::BodyDefinition>& a,
                    const std::vector<RigidBodySimulation::BodyDefinition>& b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i)
    {
        const RigidBodySimulation::BodyDefinition& x = a[i];
        const RigidBodySimulation::BodyDefinition& y = b[i];
        if (x.restPos.x != y.restPos.x || x.restPos.y != y.restPos.y ||
            x.restPos.z != y.restPos.z || x.restRot.x != y.restRot.x ||
            x.restRot.y != y.restRot.y || x.restRot.z != y.restRot.z || x.mass != y.mass ||
            x.linearDamping != y.linearDamping || x.angularDamping != y.angularDamping ||
            x.friction != y.friction || x.restitution != y.restitution ||
            x.colliderType != y.colliderType || x.radius != y.radius ||
            x.extents.x != y.extents.x || x.extents.y != y.extents.y ||
            x.extents.z != y.extents.z || x.length != y.length || x.mask != y.mask ||
            x.groupId != y.groupId || x.physicsMode != y.physicsMode || x.enabled != y.enabled)
        {
            return false;
        }
    }
    return true;
}

// Read one joint element's fields into an engine JointDefinition.
[[nodiscard]] RigidBodySimulation::JointDefinition readJoint(MDataHandle& hd)
{
    RigidBodySimulation::JointDefinition j;
    j.bodyA = hd.child(RigidBodyNode::aJointBodyA).asInt();
    j.bodyB = hd.child(RigidBodyNode::aJointBodyB).asInt();
    j.type = hd.child(RigidBodyNode::aJointType).asInt();
    j.frameT = readDouble3(hd.child(RigidBodyNode::aJointFrameTranslate));
    j.frameR = readDouble3(hd.child(RigidBodyNode::aJointFrameRotate));
    j.linearMin = readDouble3(hd.child(RigidBodyNode::aJointLinearMin));
    j.linearMax = readDouble3(hd.child(RigidBodyNode::aJointLinearMax));
    j.angularMin = readDouble3(hd.child(RigidBodyNode::aJointAngularMin));
    j.angularMax = readDouble3(hd.child(RigidBodyNode::aJointAngularMax));
    j.linearSpring = readDouble3(hd.child(RigidBodyNode::aJointLinearSpring));
    j.angularSpring = readDouble3(hd.child(RigidBodyNode::aJointAngularSpring));
    return j;
}

[[nodiscard]] std::vector<RigidBodySimulation::JointDefinition> readJointData(MDataBlock& dataBlock)
{
    std::vector<RigidBodySimulation::JointDefinition> out;
    MArrayDataHandle jointsHandle = dataBlock.inputArrayValue(RigidBodyNode::aJoints);
    const unsigned int jointCount = jointsHandle.elementCount();
    out.reserve(jointCount);
    for (unsigned int i = 0; i < jointCount; ++i)
    {
        jointsHandle.jumpToArrayElement(i);
        MDataHandle element = jointsHandle.inputValue();
        out.push_back(readJoint(element));
    }
    return out;
}

[[nodiscard]] Double3 readGravity(MDataBlock& dataBlock)
{
    return readDouble3(dataBlock.inputValue(RigidBodyNode::aGravity));
}

// True when `plug` is one of the node's output compounds (or an element or
// child of one) — the only plugs compute() must service.
bool isOutputPlug(const MPlug& plug)
{
    return plug == RigidBodyNode::aOutTranslate || plug == RigidBodyNode::aOutRotate ||
           plug.isElement() || plug.isChild();
}

// ===========================================================================
// Frame policy — pure functions over World / Inputs (no node state)
// ===========================================================================
// Read the PMX-verbatim inputs for one evaluation.
[[nodiscard]] Inputs readInputs(MDataBlock& dataBlock)
{
    Inputs in;
    in.bodies = readBodyData(dataBlock);
    in.joints = readJointData(dataBlock);
    in.gravity = readGravity(dataBlock);
    return in;
}

// K = jointRestWorld * bodyRestWorld^-1 per body (identity for bodies without
// a related joint).  Derived once per build from the wiring pass's joint paths.
[[nodiscard]] std::vector<MMatrix>
deriveWriteBackOffsets(const std::vector<RigidBodySimulation::BodyDefinition>& bodies,
                       const std::vector<MDagPath>& jointPaths)
{
    std::vector<MMatrix> k(bodies.size(), MMatrix());
    std::map<int, MMatrix> restWorldCache; // pmxBoneIndex -> joint rest world
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (!bodies[i].enabled || bodies[i].relatedBoneIndex < 0)
            continue;
        const MDagPath& jointPath = jointPaths[i];
        if (!jointPath.isValid())
            continue;
        const MMatrix jointRestWorld = jointRestWorldMatrix(jointPath, restWorldCache);
        const MMatrix bodyWorld = mmd::maya::matrixFromTR(bodies[i].restPos, bodies[i].restRot);
        k[i] = jointRestWorld * bodyWorld.inverse();
    }
    return k;
}

// Refresh each kinematic anchor from its INPUT (bodies[i].bodyAnchorWorld =
// joint.worldMatrix[0], or the body's own rest world for a boneless pin).
// Returns true when any anchor moved (a bone dragged at the current frame).
bool updateKinematicAnchors(World& world, MDataBlock& dataBlock)
{
    if (!world.sim.initialized())
        return false;
    bool anchorsMoved = false;
    MArrayDataHandle bodiesHandle = dataBlock.inputArrayValue(RigidBodyNode::aBodies);
    int anchorIndex = 0;
    for (size_t i = 0; i < world.bodies.size(); ++i)
    {
        const RigidBodySimulation::BodyDefinition& b = world.bodies[i];
        if (!b.isKinematic() || !b.enabled)
            continue;
        bodiesHandle.jumpToArrayElement((unsigned int) i);
        MMatrix w = bodiesHandle.inputValue().child(RigidBodyNode::aBodyAnchorWorld).asMatrix();
        // K^-1 (K = jointRestWorld * bodyRestWorld^-1); identity for a
        // boneless pinned body.
        if (i < world.k.size())
            w = world.k[i].inverse() * w;
        RigidBodySimulation::Pose pose;
        const btTransform t = mayaMatrixToBtTransform(w);
        storePose(pose.pos, pose.quat, t);
        if (world.sim.setKinematicPose(anchorIndex, pose))
            anchorsMoved = true;
        ++anchorIndex;
    }
    return anchorsMoved;
}

// Read each kinematic anchor's RAW world matrix (bodyAnchorWorld, BEFORE the
// K^-1 rest offset) in kinematic order.  The whole-skeleton-move detector
// compares these — the per-body K would otherwise break the shared-move test.
[[nodiscard]] std::vector<MMatrix> readRawAnchorWorlds(World& world, MDataBlock& dataBlock)
{
    std::vector<MMatrix> out;
    MArrayDataHandle bodiesHandle = dataBlock.inputArrayValue(RigidBodyNode::aBodies);
    for (size_t i = 0; i < world.bodies.size(); ++i)
    {
        const RigidBodySimulation::BodyDefinition& b = world.bodies[i];
        if (!b.isKinematic() || !b.enabled)
            continue;
        bodiesHandle.jumpToArrayElement((unsigned int) i);
        out.push_back(
            bodiesHandle.inputValue().child(RigidBodyNode::aBodyAnchorWorld).asMatrix());
    }
    return out;
}

// Approximate rigid-transform equality (column-vector): same translation and
// same basis.  Used to decide whether every anchor moved by one shared move.
[[nodiscard]] bool transformsNear(const btTransform& a, const btTransform& b,
                                  btScalar distTol = 1e-3, btScalar rotTol = 1e-4)
{
    const btVector3 d = a.getOrigin() - b.getOrigin();
    if (d.length2() > distTol * distTol)
        return false;
    for (int c = 0; c < 3; ++c)
    {
        if (a.getBasis().getColumn(c).dot(b.getBasis().getColumn(c)) < 1.0 - rotTol)
            return false;
    }
    return true;
}

// Detect a WHOLE-SKELETON rigid move: every BONE-ATTACHED kinematic anchor
// moved by the SAME world-space rigid transform since the last evaluation —
// the user dragged the character as a unit (e.g. translating/rotating the
// model root at a paused frame).  Boneless pinned anchors are excluded (they
// live in the world, not on the skeleton).  Returns the shared move as a
// column-vector btTransform when detected, nullopt when anchors moved
// differently (a local bone drag / normal animation) or nothing moved.
[[nodiscard]] std::optional<btTransform>
detectWholeSkeletonMove(const World& world, const std::vector<MMatrix>& prev,
                        const std::vector<MMatrix>& cur)
{
    if (prev.size() != cur.size() || cur.empty())
        return std::nullopt;
    std::optional<btTransform> common;
    size_t anchor = 0;
    for (size_t i = 0; i < world.bodies.size(); ++i)
    {
        const RigidBodySimulation::BodyDefinition& b = world.bodies[i];
        if (!b.isKinematic() || !b.enabled)
            continue;
        if (b.relatedBoneIndex < 0) // boneless pin — not part of the skeleton
        {
            ++anchor;
            continue;
        }
        // Maya row-vector: cur = prev * M  =>  M = prev^-1 * cur.  Bullet is
        // the transpose (column-vector), so btMove = mayaMatrixToBtTransform(M).
        const MMatrix move = prev[anchor].inverse() * cur[anchor];
        const btTransform btMove = mayaMatrixToBtTransform(move);
        if (!common)
        {
            common = btMove;
        }
        else if (!transformsNear(btMove, *common))
        {
            return std::nullopt; // anchors moved differently — local drag
        }
        ++anchor;
    }
    if (!common)
        return std::nullopt;
    // The shared move must be a real move (not the identity transform).
    if (transformsNear(*common, btTransform::getIdentity()))
        return std::nullopt;
    return common;
}

// Build a fresh world from the PMX-verbatim inputs: resolve the DAG wiring,
// derive the write-back offsets K, initialize the Bullet engine, and pin the
// chains to the CURRENT skeleton pose (a posed skeleton must not snap to
// rest).  std::nullopt when there are no bodies — an empty node is a valid
// no-op.
[[nodiscard]] std::optional<World> buildWorld(const MObject& node, const Inputs& in,
                                              const MTime& now, MDataBlock& dataBlock)
{
    if (in.bodies.empty())
        return std::nullopt;

    std::optional<World> result(std::in_place);
    World& world = *result;
    world.bodies = in.bodies;
    world.joints = in.joints;
    world.gravity = in.gravity;

    std::vector<MDagPath> jointPaths;
    resolveBodyWiring(node, world.bodies, jointPaths);
    world.k = deriveWriteBackOffsets(world.bodies, jointPaths);

    RigidBodySimulation::Definition definition;
    definition.gravity = in.gravity;
    definition.bodies = world.bodies; // wired — the engine consumes the reset anchors
    definition.joints = in.joints;
    if (!world.sim.initialize(definition))
        return std::nullopt;

    updateKinematicAnchors(world, dataBlock); // apply anchors with the fresh K
    world.sim.resetDynamicBodies();           // chains stay at the current pose
    // Baseline the raw anchor worlds so the whole-skeleton-move detector has
    // a "previous" frame to compare the first drag against.
    world.lastAnchorWorld = readRawAnchorWorlds(world, dataBlock);

    world.lastTime = now.value(); // no time-step on the (re)build frame
    world.lastTimeUnit = now.unit();
    return result;
}

// The PMX-verbatim configs differ — rebuild.  The DAG-derived wiring (bone
// indices, reset anchors) is resolved at build and is deliberately NOT part of
// the comparison (re-binding a body or re-parenting a joint needs a rebuild
// trigger, e.g. re-import).  Anchor VALUES are per-frame reads, never compared.
[[nodiscard]] bool configChanged(const World& world, const Inputs& in)
{
    return !sameBodyFields(in.bodies, world.bodies) || in.joints != world.joints ||
           in.gravity.x != world.gravity.x || in.gravity.y != world.gravity.y ||
           in.gravity.z != world.gravity.z;
}

// The world must be (re)built when its config differs from the current inputs,
// or when time was scrubbed backwards (rebuild rather than re-simulate).
[[nodiscard]] bool needsRebuild(const World& world, const Inputs& in, const MTime& now)
{
    if (configChanged(world, in))
        return true;
    return (now - MTime(world.lastTime, world.lastTimeUnit)).as(MTime::kSeconds) < 0.0;
}

// Advance the simulation by the frame span when time moved, or by one fixed
// tick when a kinematic bone was dragged at the current frame.  The anchors
// are refreshed first so the colliders track their bones.  Returns the world
// with the updated timeline cursor (unchanged when nothing happened).
//
// Whole-skeleton drags (moving/rotating the character at a paused frame) do
// NOT run physics: every bone-attached anchor shares the same world-space
// move, so the dynamic chains ride along by that move instead of being yanked
// by teleported anchors (the old behaviour displaced the skirt/hair by the
// move, and the displacement was baked into the write-back).  A local bone
// drag moves the anchors differently and still gets the single fixed tick.
[[nodiscard]] World advance(World world, const MTime& now, MDataBlock& dataBlock)
{
    const bool anchorsMoved = updateKinematicAnchors(world, dataBlock);
    const std::vector<MMatrix> curAnchors = readRawAnchorWorlds(world, dataBlock);
    const double dt = (now - MTime(world.lastTime, world.lastTimeUnit)).as(MTime::kSeconds);
    if (dt > 0.0)
    {
        world.sim.step(dt);
    }
    else if (anchorsMoved)
    {
        if (const auto move = detectWholeSkeletonMove(world, world.lastAnchorWorld, curAnchors))
        {
            RigidBodySimulation::Pose movePose;
            storePose(movePose.pos, movePose.quat, *move);
            world.sim.rideDynamicBodiesAlong(movePose);
        }
        else
        {
            world.sim.step(RigidBodySimulation::kFixedDt);
        }
    }
    else
    {
        return world; // nothing moved — the anchor history is still current
    }
    world.lastAnchorWorld = std::move(curAnchors);
    world.lastTime = now.value();
    world.lastTimeUnit = now.unit();
    return world;
}

// The whole frame transition: (re)build the world when its config or time
// demands it, otherwise advance it.  The previous world is moved in and the
// result replaces it — state is never mutated piecemeal.
[[nodiscard]] std::optional<World> frame(std::optional<World> world, const Inputs& in,
                                         const MObject& node, const MTime& now,
                                         MDataBlock& dataBlock)
{
    if (!world || needsRebuild(*world, in, now))
        return buildWorld(node, in, now, dataBlock);
    return advance(std::move(*world), now, dataBlock);
}

// Write each dynamic body's solved local pose to outTranslate/outRotate.
// An empty world (no bodies) writes empty arrays.
MStatus writeOutputs(const std::optional<World>& world, MDataBlock& dataBlock)
{
    const unsigned int bodyCount = world ? static_cast<unsigned int>(world->bodies.size()) : 0u;
    MArrayDataBuilder tBuilder(&dataBlock, RigidBodyNode::aOutTranslate, bodyCount);
    MArrayDataBuilder rBuilder(&dataBlock, RigidBodyNode::aOutRotate, bodyCount);

    if (world)
    {
        // Two-pass bone-world write-back, so the driven joints are never read
        // back from the DG (that feedback cycle destabilized the sim):
        //   pass 1 — solvedBoneWorld[bone] = bodyPose(i) * K_i for the first
        //            enabled body on each bone (kinematic bodies track their
        //            joints, so their bone world IS the animated joint world);
        //   pass 2 — boneLocal = solvedBoneWorld[parentBone]^-1 * solvedBoneWorld[bone],
        //            or the raw solved world pose when the parent bone has no body.
        // Bullet/btTransform is COLUMN-vector: `bodyPose * K` is the joint world
        // (the transpose of the row-vector K * bodyPose).

        // Pass 1 — solved bone world per bone.  First body on a bone wins
        // (bodies are created in PMX order, so the lowest body index on a bone
        // drives it); bodies without a related bone are skipped.
        std::map<int, btTransform> solvedBoneWorld;
        for (size_t i = 0; i < world->bodies.size(); ++i)
        {
            const RigidBodySimulation::BodyDefinition& bd = world->bodies[i];
            if (!bd.enabled || bd.relatedBoneIndex < 0)
                continue;
            if (solvedBoneWorld.find(bd.relatedBoneIndex) != solvedBoneWorld.end())
                continue; // first body on the bone wins
            if (i >= world->k.size())
                continue; // defensive — K is derived for every body at build
            const btTransform kb = mayaMatrixToBtTransform(world->k[i]);
            const RigidBodySimulation::Pose wp = world->sim.bodyPose(i);
            solvedBoneWorld.emplace(bd.relatedBoneIndex, poseToTransform(wp.pos, wp.quat) * kb);
        }

        // Pass 2 — write the joint-local pose, or the raw solved world pose
        // when the parent bone has no body (rare in well-formed chains).
        for (size_t i = 0; i < world->bodies.size(); ++i)
        {
            const RigidBodySimulation::BodyDefinition& bd = world->bodies[i];
            if (bd.isKinematic() || !bd.enabled)
                continue;

            const RigidBodySimulation::Pose wp = world->sim.bodyPose(i);
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
                else if (parentBone == -1 && it != solvedBoneWorld.end())
                {
                    // Root bone: its parent is the model root at identity, so
                    // the joint-local pose IS the solved bone world (falling
                    // through to the raw body pose would bake the K offset
                    // into the joint).
                    boneLocal = it->second;
                }
            }
            // Parent bone without a body -> boneLocal stays the raw solved world pose.

            const btVector3& o = boneLocal.getOrigin();
            Double3 rot;
            const btQuaternion& bq = boneLocal.getRotation();
            quatToEulerXYZDegrees(Double4(bq.x(), bq.y(), bq.z(), bq.w()), rot);

            // PHYSICS writes translate+rotate; PHYSICS_BONE is rotation-only.
            if (bd.physicsMode != RigidBodySimulation::PhysicsMode::ePhysicsBone)
            {
                MDataHandle tEl = tBuilder.addElement((unsigned int) i);
                tEl.child(RigidBodyNode::aOutTranslateX).setMDistance(MDistance(o.x()));
                tEl.child(RigidBodyNode::aOutTranslateY).setMDistance(MDistance(o.y()));
                tEl.child(RigidBodyNode::aOutTranslateZ).setMDistance(MDistance(o.z()));
            }

            MDataHandle rEl = rBuilder.addElement((unsigned int) i);
            // Written in DEGREES (quatToEulerXYZDegrees output; MAngle's
            // default unit is radians, so the unit must be explicit).
            rEl.child(RigidBodyNode::aOutRotateX).setMAngle(MAngle(rot.x, MAngle::kDegrees));
            rEl.child(RigidBodyNode::aOutRotateY).setMAngle(MAngle(rot.y, MAngle::kDegrees));
            rEl.child(RigidBodyNode::aOutRotateZ).setMAngle(MAngle(rot.z, MAngle::kDegrees));
        }
    }

    MArrayDataHandle tOut = dataBlock.outputArrayValue(RigidBodyNode::aOutTranslate);
    tOut.set(tBuilder);
    tOut.setAllClean();
    MArrayDataHandle rOut = dataBlock.outputArrayValue(RigidBodyNode::aOutRotate);
    rOut.set(rBuilder);
    rOut.setAllClean();
    dataBlock.outputValue(RigidBodyNode::aOutTranslate).setClean();
    dataBlock.outputValue(RigidBodyNode::aOutRotate).setClean();
    return MS::kSuccess;
}

} // namespace

// ===========================================================================
// Node lifecycle
// ===========================================================================
RigidBodyNode::RigidBodyNode() = default;

// Defaulted out-of-line: the node is destroyed polymorphically through its
// MPxNode base, and the default teardown is exactly what we want — mWorld's
// optional destroys the World, whose RigidBodySimulation tears down the
// Bullet world in its own PIMPL destructor.
RigidBodyNode::~RigidBodyNode() = default;

void* RigidBodyNode::creator()
{
    return new RigidBodyNode();
}

// ===========================================================================
// Attribute registration
// ===========================================================================
namespace
{
// Create a numeric attribute with the standard storable + non-keyable flags,
// so each schema line stays a single readable declaration.
MObject makeNumeric(MFnNumericAttribute& fn, const MString& longName, const MString& shortName,
                    MFnNumericData::Type type, double def)
{
    MStatus stat;
    MObject attr = fn.create(longName, shortName, type, def, &stat);
    MMD_CHECK_MSTATUS(stat);
    fn.setStorable(true);
    fn.setKeyable(false);
    return attr;
}

// Same, for a PMX-name string attribute.
MObject makeString(MFnTypedAttribute& fn, const MString& longName, const MString& shortName)
{
    MStatus stat;
    MObject attr = fn.create(longName, shortName, MFnData::kString, MObject::kNullObj, &stat);
    MMD_CHECK_MSTATUS(stat);
    fn.setStorable(true);
    fn.setKeyable(false);
    return attr;
}
} // namespace

MStatus RigidBodyNode::initialize()
{
    MFnNumericAttribute nAttr;
    MFnCompoundAttribute cAttr;
    MFnMatrixAttribute mAttr;
    MFnMessageAttribute mMsgAttr;
    MFnTypedAttribute tAttr;
    MFnUnitAttribute uAttr;
    MStatus stat;

    // --- time (the frame driver; hidden + keyable) ---
    aTime = uAttr.create("time", "tm", MFnUnitAttribute::kTime, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    uAttr.setStorable(true);
    uAttr.setKeyable(true);
    uAttr.setHidden(true);

    // --- gravity ---
    aGravity = makeNumeric(nAttr, "gravity", "grav", MFnNumericData::k3Double, 0.0);
    nAttr.setDefault(0.0, -9.8, 0.0); // MMD's physics engine uses exactly -9.8

    // --- body compound (children mirror rigid_bodies.json; aBodyEnabled — a
    // Maya-only custom attribute — sits first) ---
    aBodyEnabled = makeNumeric(nAttr, "bodyEnabled", "ben", MFnNumericData::kBoolean, 1.0);
    aBodyNameLocal = makeString(tAttr, "bodyNameLocal", "bnml");
    aBodyNameUniversal = makeString(tAttr, "bodyNameUniversal", "bnmu");

    // PMX collision group (0..15).
    {
        MFnEnumAttribute eAttr;
        aBodyGroupId = eAttr.create("bodyGroupId", "bgid", 0, &stat);
        MMD_CHECK_MSTATUS(stat);
        for (int g = 0; g < 16; ++g)
            eAttr.addField(MString(("Group " + std::to_string(g)).c_str()), static_cast<short>(g));
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

    // PMX collision mask — one bool per group.  This is the
    // non_collision_group field stored VERBATIM (bit i set = collides with
    // group i; MMD feeds it to Bullet directly — no inversion).
    for (int g = 0; g < 16; ++g)
    {
        const MString longName(("bodyMaskGroup" + std::to_string(g)).c_str());
        const MString shortName(("bmg" + std::to_string(g)).c_str());
        aBodyMaskGroup.at(g) =
            makeNumeric(nAttr, longName, shortName, MFnNumericData::kBoolean, 1.0);
    }

    // PMX collider shape.
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
    aBodyShapeSize = makeNumeric(nAttr, "bodyShapeSize", "bss", MFnNumericData::k3Double, 1.0);
    aBodyRestTranslate =
        makeNumeric(nAttr, "bodyRestTranslate", "brt", MFnNumericData::k3Double, 0.0);
    aBodyRestRotate = makeNumeric(nAttr, "bodyRestRotate", "brr", MFnNumericData::k3Double, 0.0);
    aBodyMass = makeNumeric(nAttr, "bodyMass", "bm", MFnNumericData::kDouble, 1.0);
    aBodyLinearDamping =
        makeNumeric(nAttr, "bodyLinearDamping", "bld", MFnNumericData::kDouble, 0.0);
    aBodyAngularDamping =
        makeNumeric(nAttr, "bodyAngularDamping", "bad", MFnNumericData::kDouble, 0.0);
    aBodyRestitution = makeNumeric(nAttr, "bodyRestitution", "bre", MFnNumericData::kDouble, 0.0);
    aBodyFriction = makeNumeric(nAttr, "bodyFriction", "bfr", MFnNumericData::kDouble, 0.5);

    // PMX physics mode — followBone / physics / physicsBone.
    {
        MFnEnumAttribute eAttr;
        aBodyPhysicsMode =
            eAttr.create("bodyPhysicsMode", "bpm",
                         static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysics), &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("FollowBone",
                       static_cast<short>(RigidBodySimulation::PhysicsMode::eFollowBone));
        eAttr.addField("Physics", static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysics));
        eAttr.addField("PhysicsBone",
                       static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysicsBone));
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

    // The related joint as a MESSAGE (mirrors PMX related_bone_index);
    // unconnected = a static collider (no write-back).
    aBodyJoint = mMsgAttr.create("bodyJoint", "bjnt", &stat);
    MMD_CHECK_MSTATUS(stat);
    mMsgAttr.setStorable(true);
    mMsgAttr.setKeyable(false);

    // Kinematic-anchor INPUT: the bone world the body follows
    // (joint.worldMatrix[0]; a boneless FOLLOW_BONE body pins its rest world).
    aBodyAnchorWorld = mAttr.create("bodyAnchorWorld", "baw", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(true);
    mAttr.setKeyable(false);

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
    cAttr.addChild(aBodyAnchorWorld);

    // --- joint compound (mirrors the PMX rigid-body constraint fields) ---
    aJointNameLocal = makeString(tAttr, "jointNameLocal", "jnml");
    aJointNameUniversal = makeString(tAttr, "jointNameUniversal", "jnmu");
    aJointBodyA = makeNumeric(nAttr, "jointBodyA", "jba", MFnNumericData::kLong, 0);
    aJointBodyB = makeNumeric(nAttr, "jointBodyB", "jbb", MFnNumericData::kLong, 0);
    // PMX joint type — one field per PMX JointType (0..5).
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
        makeNumeric(nAttr, "jointFrameTranslate", "jft", MFnNumericData::k3Double, 0.0);
    aJointFrameRotate =
        makeNumeric(nAttr, "jointFrameRotate", "jfr", MFnNumericData::k3Double, 0.0);
    aJointLinearMin = makeNumeric(nAttr, "jointLinearMin", "jlmn", MFnNumericData::k3Double, 0.0);
    aJointLinearMax = makeNumeric(nAttr, "jointLinearMax", "jlmx", MFnNumericData::k3Double, 0.0);
    aJointAngularMin = makeNumeric(nAttr, "jointAngularMin", "jamn", MFnNumericData::k3Double, 0.0);
    aJointAngularMax = makeNumeric(nAttr, "jointAngularMax", "jamx", MFnNumericData::k3Double, 0.0);
    aJointLinearSpring =
        makeNumeric(nAttr, "jointLinearSpring", "jls", MFnNumericData::k3Double, 0.0);
    aJointAngularSpring =
        makeNumeric(nAttr, "jointAngularSpring", "jas", MFnNumericData::k3Double, 0.0);

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
    // transform.translate/rotate, so the write-back connections to
    // joint.translate / joint.rotate are direct — a unitless k3Double would
    // make Maya insert a unitConversion.
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

    // Every config input drives the outputs too, so compute() re-runs on a
    // body/joint/gravity edit (config change detection) and on a kinematic
    // bone drag at a fixed time (the anchor lives on bodies[i].bodyAnchorWorld).
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

    return MS::kSuccess;
}

// ===========================================================================
// Per-frame update
// ===========================================================================
void RigidBodyNode::getCacheSetup(const MEvaluationNode& evalNode,
                                  MNodeCacheDisablingInfo& disablingInfo,
                                  MNodeCacheSetupInfo& setupInfo,
                                  MObjectArray& monitoredAttributes) const
{
    // The node advances an internal Bullet world in compute(), so its outputs
    // are NOT a pure function of its inputs — Cached Playback must re-evaluate
    // it every frame, exactly like a scripted/expression node.
    MString category("pmxRigidBodyNode: stateful Bullet solver (steps every frame)");
    MNodeCacheDisablingInfoHelper::setUnsafeNode(disablingInfo, evalNode, &category);
    MPxLocatorNode::getCacheSetup(evalNode, disablingInfo, setupInfo, monitoredAttributes);
}

// ===========================================================================
// compute()
// ===========================================================================
MStatus RigidBodyNode::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    if (!isOutputPlug(plug))
        return MS::kUnknownParameter;

    const MTime now = dataBlock.inputValue(aTime).asTime();

    // Frame transition: read the PMX-verbatim inputs, then (re)build or
    // advance the world.  The new world atomically replaces the old one.
    const Inputs in = readInputs(dataBlock);
    mWorld = frame(std::move(mWorld), in, thisMObject(), now, dataBlock);

    return writeOutputs(mWorld, dataBlock);
}
