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
#include "rigid_body_shape.hpp"

#include <algorithm>

#include <maya/MAngle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDagPath.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MDistance.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTransform.h>
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

MObject RigidBodyNode::aBodyShapes;

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

MObject RigidBodyNode::aOutGuideTranslate;
MObject RigidBodyNode::aOutGuideTranslateX;
MObject RigidBodyNode::aOutGuideTranslateY;
MObject RigidBodyNode::aOutGuideTranslateZ;
MObject RigidBodyNode::aOutGuideRotate;
MObject RigidBodyNode::aOutGuideRotateX;
MObject RigidBodyNode::aOutGuideRotateY;
MObject RigidBodyNode::aOutGuideRotateZ;

// ===========================================================================
// File-local helpers (pure attribute/plugin reading — no node state)
// ===========================================================================
namespace
{

// Per-evaluation inputs (transient — not part of the node's state).  The
// PMX-verbatim inputs for one evaluation; World (RigidBodyNode::World, see the
// header) is the built state derived from them.
struct Inputs
{
    std::vector<mmd::core::RigidBodySimulation::BodyDefinition> bodies;
    std::vector<mmd::core::RigidBodySimulation::JointDefinition> joints;
    mmd::core::Double3 gravity;
};

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
    Double3 out;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    std::copy_n(hd.asDouble3(), 3, out.data());
    return out;
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
    const std::array<double, 3> rot = {rx, ry, rz};
    tm.setRotation(rot.data(), MTransformationMatrix::kXYZ);
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

// Resolve the body's related joint (its shape's bodyJoint message) into the
// joint's DAG path + PMX bone indices; returns false for a body with no
// connected joint (a static collider).  The DAG IS the PMX bone hierarchy —
// the bone builder parents each joint directly under its PMX parent.  Runs
// once per world build via resolveBodyWiring (~5 DG API calls per body); the
// result is cached in the world's bodies and not re-checked per frame —
// re-binding a body or re-parenting a joint needs a rebuild trigger (e.g.
// re-import).
bool resolveRelatedBones(const MObject& shapeNode, MDagPath& jointPath, int& boneIndex,
                         int& parentBoneIndex)
{
    boneIndex = -1;
    parentBoneIndex = -1;
    if (shapeNode.isNull() || !shapeNode.hasFn(MFn::kDependencyNode))
        return false;
    MPlugArray sources;
    MPlug(shapeNode, RigidBodyShape::aBodyJoint).connectedTo(sources, true, false);
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

// Read the solver's body list from its bodyShapes[] message array (PMX
// order — written DENSELY by the builder): each connected pmxRigidBodyShape
// node provides one BodyDefinition (PMX-verbatim fields; the REST pose comes
// from the shape's bodyRestTranslate/bodyRestRotate attributes, not the
// guide transform, which holds the CURRENT pose).  Unconnected slots are
// skipped, keeping the body index == connected-shape order.
[[nodiscard]] std::vector<RigidBodySimulation::BodyDefinition> readBodyData(const MObject& node)
{
    std::vector<RigidBodySimulation::BodyDefinition> out;
    MPlug shapesPlug(node, RigidBodyNode::aBodyShapes);
    const unsigned int bodyCount = shapesPlug.evaluateNumElements();
    out.reserve(bodyCount);
    for (unsigned int i = 0; i < bodyCount; ++i)
    {
        MPlugArray sources;
        shapesPlug.elementByLogicalIndex(i).connectedTo(sources, true, false);
        if (sources.length() == 0)
            continue;
        RigidBodySimulation::BodyDefinition b;
        if (RigidBodyShape::readBodyDefinition(sources[0].node(), b))
            out.push_back(b);
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
    MPlug shapesPlug(node, RigidBodyNode::aBodyShapes);
    const unsigned int bodyCount = shapesPlug.evaluateNumElements();
    size_t bi = 0;
    for (unsigned int i = 0; i < bodyCount && bi < bodies.size(); ++i)
    {
        MPlugArray sources;
        shapesPlug.elementByLogicalIndex(i).connectedTo(sources, true, false);
        if (sources.length() == 0)
            continue;
        resolveRelatedBones(sources[0].node(), jointPaths[bi], bodies[bi].relatedBoneIndex,
                            bodies[bi].parentBoneIndex);
        ++bi;
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
[[nodiscard]] Inputs readInputs(const MObject& node, MDataBlock& dataBlock)
{
    Inputs in;
    in.bodies = readBodyData(node);
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

// The connected pmxRigidBodyShape node for body `index` (PMX order); null
// when the bodyShapes[] slot is unconnected.
[[nodiscard]] MObject bodyShapeNode(const MObject& node, unsigned int index)
{
    MPlugArray sources;
    MPlug(node, RigidBodyNode::aBodyShapes)
        .elementByLogicalIndex(index)
        .connectedTo(sources, true, false);
    return sources.length() ? sources[0].node() : MObject::kNullObj;
}

// Refresh each kinematic anchor from its INPUT (bodies[i].bodyAnchorWorld =
// joint.worldMatrix[0], or the body's own rest world for a boneless pin).
// Returns true when any anchor moved (a bone dragged at the current frame).
//
// The anchor is placed at the RAW world pose: bodyAnchorWorld (the joint
// world) transformed by the constant bodyRest^-1 * jointRest offset — i.e.
// the body's rest pose rigidly attached to the bone's CURRENT world pose.
// This is EXACT for a whole-skeleton move M (anchor -> M·jointWorld ⇒ body ->
// M·bodyRest) and for animation (body follows the posed bone).  The write-back
// (pass 1) therefore reads the kinematic body's pose directly (no K multiply),
// and the raw-anchor reset keeps the weld constraint exactly satisfied.
bool updateKinematicAnchors(World& world, const MObject& node)
{
    if (!world.sim.initialized())
        return false;
    bool anchorsMoved = false;
    int anchorIndex = 0;
    for (size_t i = 0; i < world.bodies.size(); ++i)
    {
        const RigidBodySimulation::BodyDefinition& b = world.bodies[i];
        if (!b.isKinematic() || !b.enabled)
            continue;
        const MObject shape = bodyShapeNode(node, (unsigned int) i);
        if (shape.isNull())
            continue;
        MPlug anchorPlug(shape, RigidBodyShape::aBodyAnchorWorld);
        MMatrix w = anchorPlug.asMDataHandle().asMatrix();
        // RAW rigid attachment (row-vector): the body is a child of its joint
        // at a constant local offset, so bodyWorld = bodyRest * anchorRest^-1 *
        // anchorWorld (bodyRest on the LEFT).  At rest this is bodyRest; under
        // a whole move M (anchor = anchorRest * M) it is bodyRest * M — EXACT,
        // no K conjugation.  (For a boneless pin the anchor IS the body's own
        // rest world, so w stays put.)  The engine reset uses the same form
        // (anchorCurrent * anchorRest^-1 * bodyRest in bt/column = bodyRest *
        // anchorRest^-1 * anchorCurrent in row), so kinematic and dynamic
        // bodies stay weld-consistent.
        if (anchorIndex < static_cast<int>(world.originalAnchorWorld.size()))
        {
            const MMatrix bodyRest = mmd::maya::matrixFromTR(b.restPos, b.restRot);
            w = bodyRest * world.originalAnchorWorld[anchorIndex].inverse() * w;
        }
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
[[nodiscard]] std::vector<MMatrix> readRawAnchorWorlds(World& world, const MObject& node)
{
    std::vector<MMatrix> out;
    for (size_t i = 0; i < world.bodies.size(); ++i)
    {
        const RigidBodySimulation::BodyDefinition& b = world.bodies[i];
        if (!b.isKinematic() || !b.enabled)
            continue;
        const MObject shape = bodyShapeNode(node, (unsigned int) i);
        if (shape.isNull())
            continue;
        MPlug anchorPlug(shape, RigidBodyShape::aBodyAnchorWorld);
        out.push_back(anchorPlug.asMDataHandle().asMatrix());
    }
    return out;
}

// Convert RAW (unconjugated) anchor MMatrices to engine Pose vectors, keeping
// kinematic order.  The scrub-back reset consumes these so a whole-skeleton
// move is not K-conjugated.
[[nodiscard]] std::vector<RigidBodySimulation::Pose>
anchorsToPoses(const std::vector<MMatrix>& anchors)
{
    std::vector<RigidBodySimulation::Pose> out;
    out.reserve(anchors.size());
    for (const MMatrix& m : anchors)
    {
        const btTransform t = mayaMatrixToBtTransform(m);
        RigidBodySimulation::Pose pose;
        storePose(pose.pos, pose.quat, t);
        out.push_back(pose);
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
[[nodiscard]] std::optional<btTransform> detectWholeSkeletonMove(const World& world,
                                                                 const std::vector<MMatrix>& prev,
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
//
// Pinning: the raw reset places each dynamic body at curRaw * (restRaw^-1 *
// bodyRest) — its rest pose rigidly attached to its reset-anchor bone at the
// bone's CURRENT world pose.  `restRaw` is the anchor bone's REST world (a
// model constant composed from the stamped pmxRest* attributes), so the
// formula is EXACT under both a whole-skeleton move M (curRaw = restRaw * M
// ⇒ M·bodyRest) and an animation pose (curRaw = restRaw * P ⇒ P·bodyRest).
// Using the CURRENT anchors as the "rest" reference instead would bake the
// pose/move into the offset (bodies stuck at rest while the skeleton is
// posed — the 51° mismatch on first play — and a rewind rebuild comparing
// against a different captured pose — the persistent rewind jump).
[[nodiscard]] std::optional<World> buildWorld(const MObject& node, const Inputs& in,
                                              const MTime& now)
{
    if (in.bodies.empty())
        return std::nullopt;

    // Note: use default-construct + emplace() rather than
    // std::optional<World>(std::in_place): clang-cl's MSVC-STL handling of
    // _SMF_control misreports World as not default-constructible when the
    // enclosing node also owns a std::optional<World> member, so the
    // in_place ctor is SFINAE'd out under clang-tidy.  MSVC accepts both.
    std::optional<World> result;
    result.emplace();
    World& world = *result;
    world.bodies = in.bodies;
    world.joints = in.joints;
    world.gravity = in.gravity;

    // The guide outputs are expressed in the RigidBodies group's space (the
    // solver's DAG parent — also the parent of every per-body guide
    // transform).  The Bullet world runs in WORLD space, so each body's
    // solved pose must be brought into the group's local frame before it can
    // drive a child of the group.
    {
        MDagPath solverPath;
        if (MDagPath::getAPathTo(node, solverPath) == MS::kSuccess &&
            solverPath.pop() == MS::kSuccess)
        {
            // inclusiveMatrix() lives on MDagPath (not the function set).
            world.groupWorld = solverPath.inclusiveMatrix();
        }
        else
        {
            world.groupWorld = MMatrix::identity;
        }
    }

    std::vector<MDagPath> jointPaths;
    resolveBodyWiring(node, world.bodies, jointPaths);
    world.k = deriveWriteBackOffsets(world.bodies, jointPaths);
    // Cache the related-joint paths for the write-back fallback (a dynamic
    // body whose parent bone has no body needs the parent joint's world to
    // express the solved pose as a joint-local pose).  Resolved once per
    // build — the DAG wiring is cached in the world, like everything else.
    world.jointPaths = jointPaths;

    RigidBodySimulation::Definition definition;
    definition.gravity = in.gravity;
    definition.bodies = world.bodies; // wired — the engine consumes the reset anchors
    definition.joints = in.joints;
    if (!world.sim.initialize(definition))
        return std::nullopt;

    // The scrub-back reset's "rest" reference: each kinematic anchor's REST
    // world (the joint's composed rest world — a model constant from the
    // stamped pmxRest* attributes; a boneless pin's anchor IS its own rest
    // world, which never moves).  Read BEFORE updateKinematicAnchors so the
    // raw placement can use it.
    const std::vector<MMatrix> curAnchors = readRawAnchorWorlds(world, node);
    std::vector<MMatrix> originalAnchors;
    originalAnchors.reserve(curAnchors.size());
    {
        std::map<int, MMatrix> restWorldCache;
        for (size_t body = 0; body < world.bodies.size(); ++body)
        {
            const RigidBodySimulation::BodyDefinition& b = world.bodies[body];
            if (!b.isKinematic() || !b.enabled)
                continue;
            if (b.relatedBoneIndex >= 0 && body < jointPaths.size() && jointPaths[body].isValid())
            {
                originalAnchors.push_back(jointRestWorldMatrix(jointPaths[body], restWorldCache));
            }
            else
            {
                // Boneless pin: the anchor is the body's own rest world.
                originalAnchors.push_back(mmd::maya::matrixFromTR(b.restPos, b.restRot));
            }
        }
    }
    world.originalAnchorWorld = originalAnchors;

    updateKinematicAnchors(world, node); // apply anchors with the fresh raw placement

    // Pin the chains to the CURRENT skeleton pose with the RAW anchor worlds
    // (the joint worlds, NOT the K^-1-conjugated kinematic-body poses).  Each
    // dynamic body is placed at curRaw * (restRaw^-1 * bodyRest) — its rest
    // pose rigidly attached to its reset-anchor bone at the bone's CURRENT
    // world pose.  This is EXACT for a whole-skeleton move M (curRaw =
    // restRaw * M ⇒ M·bodyRest) and for an animation-posed skeleton (curRaw
    // is the posed bone world).  The internally stored mAnchorCurrent is the
    // kinematic BODY pose (bodyRest * rest^-1 * cur), so the legacy reset
    // would conjugate the move by the kinematic body's rest rotation — the
    // very K^-1·M·K rotation the raw design removes.
    world.sim.resetDynamicBodies(anchorsToPoses(originalAnchors), anchorsToPoses(curAnchors));
    // The raw reset only pins bodies that HAVE a reset anchor (a kinematic
    // ancestor).  Unanchored dynamic bodies (no kinematic ancestor) have no
    // anchor to pin to; when the whole skeleton was moved as a unit since
    // rest (a character drag that then triggers a rebuild, e.g. on rewind)
    // they must ride along from their REST pose by that move instead of
    // staying at rest.  Compare against the REST reference (originalAnchors),
    // not the previous frame: at first build there is no previous frame, and
    // a whole-skeleton move must be detected relative to the model's rest.
    if (const auto move = detectWholeSkeletonMove(world, originalAnchors, curAnchors))
    {
        RigidBodySimulation::Pose movePose;
        storePose(movePose.pos, movePose.quat, *move);
        world.sim.rideUnanchoredBodiesFromRest(movePose);
    }
    // Baseline the raw anchor worlds so the whole-skeleton-move detector has
    // a "previous" frame to compare the first drag against.
    world.lastAnchorWorld = curAnchors;

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
// Whole-skeleton drags (moving/rotating the whole character) do NOT run
// physics on the move: every bone-attached anchor shares the same world-space
// rigid move, so the dynamic chains ride along by that move instead of being
// yanked by teleported anchors.  This is checked on EVERY evaluation — not
// just at a paused frame — because an interactive viewport drag does not
// necessarily re-evaluate the solver while it happens; without the check, the
// first playback step after the drag would hit anchors already teleported by
// the move and yank the chains by it (272/273 joints jumped up to 75° in the
// reproduced case).  A local bone drag moves the anchors differently (returns
// nullopt) and still steps normally.
[[nodiscard]] World advance(World world, const MObject& node, const MTime& now)
{
    const bool anchorsMoved = updateKinematicAnchors(world, node);
    const std::vector<MMatrix> curAnchors = readRawAnchorWorlds(world, node);
    const double dt = (now - MTime(world.lastTime, world.lastTimeUnit)).as(MTime::kSeconds);

    if (const auto move = detectWholeSkeletonMove(world, world.lastAnchorWorld, curAnchors))
    {
        // Character repositioned as a unit: ride the chains along by the
        // shared move (velocities zeroed + kinematic interpolation reset by
        // the engine).  When time is also advancing, keep stepping from the
        // corrected pose; at a paused frame there is nothing to step.
        RigidBodySimulation::Pose movePose;
        storePose(movePose.pos, movePose.quat, *move);
        world.sim.rideDynamicBodiesAlong(movePose);
        if (dt > 0.0)
        {
            world.sim.step(dt);
        }
    }
    else if (dt > 0.0)
    {
        world.sim.step(dt);
    }
    else if (anchorsMoved)
    {
        // Local bone drag at a paused frame — one fixed tick so chains react.
        world.sim.step(RigidBodySimulation::kFixedDt);
    }
    else
    {
        return world; // nothing moved — the anchor history is still current
    }
    world.lastAnchorWorld = curAnchors;
    world.lastTime = now.value();
    world.lastTimeUnit = now.unit();
    return world;
}

// The whole frame transition: (re)build the world when its config or time
// demands it, otherwise advance it.  The previous world is moved in and the
// result replaces it — state is never mutated piecemeal.
[[nodiscard]] std::optional<World> frame(std::optional<World> world, const Inputs& in,
                                         const MObject& node, const MTime& now)
{
    if (!world || needsRebuild(*world, in, now))
    {
        // The rebuild derives its own REST reference from the stamped
        // pmxRest* attributes (buildWorld reads the joint rest worlds — a
        // model constant, so the pinning is identical whether the world is
        // first built, rebuilt on a config change, or rebuilt on a scrub-back;
        // a moved/posed skeleton is never baked into the offset).
        return buildWorld(node, in, now);
    }
    // A PAUSED-FRAME pose change (e.g. an animation was just applied and
    // posed the bones at the current frame without time advancing) re-pins
    // the chains by rebuilding — exactly what a scrub-back rebuild produces,
    // so the frame looks the same whether you arrived by first play or by
    // rewind.  A whole-skeleton move still rides (handled by advance); the
    // rebuild also re-baselines lastAnchorWorld, which would otherwise see
    // the pose change as a "drag" on the next eval.
    const double dt = (now - MTime(world->lastTime, world->lastTimeUnit)).as(MTime::kSeconds);
    if (dt == 0.0)
    {
        const bool anchorsMoved = updateKinematicAnchors(*world, node);
        if (anchorsMoved)
        {
            const std::vector<MMatrix> curAnchors = readRawAnchorWorlds(*world, node);
            const bool wholeMove =
                detectWholeSkeletonMove(*world, world->lastAnchorWorld, curAnchors).has_value();
            if (!wholeMove)
            {
                return buildWorld(node, in, now);
            }
        }
    }
    return advance(std::move(*world), node, now);
}

// Write each dynamic body's solved local pose to outTranslate/outRotate.
// An empty world (no bodies) writes empty arrays.
MStatus writeOutputs(const std::optional<World>& world, MDataBlock& dataBlock)
{
    const unsigned int bodyCount = world ? static_cast<unsigned int>(world->bodies.size()) : 0U;
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
        //
        // KINEMATIC bodies are placed at the RAW rigid attachment
        // (anchorWorld * anchorRest^-1 * bodyRest — exact under whole-skeleton
        // moves), so their bone world IS the raw anchor world (the joint
        // world); no K multiply.  DYNAMIC bodies are solved in the sim and
        // map back to the joint world via the constant K = jointRest *
        // bodyRest^-1 (bodyPose * K — the transpose of the row-vector
        // K * bodyPose).
        std::map<int, btTransform> solvedBoneWorld;
        size_t kinIndex = 0; // kinematic-order counter, aligned with lastAnchorWorld
        for (size_t i = 0; i < world->bodies.size(); ++i)
        {
            const RigidBodySimulation::BodyDefinition& bd = world->bodies[i];
            if (!bd.enabled)
                continue; // disabled bodies have no slot in lastAnchorWorld
            if (bd.isKinematic())
            {
                // Every ENABLED kinematic body occupies a slot in
                // lastAnchorWorld (readRawAnchorWorlds counts all of them,
                // boneless pins and duplicate-bone bodies included), so the
                // counter advances in lockstep with it.
                if (bd.relatedBoneIndex >= 0 &&
                    solvedBoneWorld.find(bd.relatedBoneIndex) == solvedBoneWorld.end() &&
                    kinIndex < world->lastAnchorWorld.size())
                {
                    // First body on this bone — its bone world IS the raw
                    // anchor world (the joint world).
                    solvedBoneWorld.emplace(
                        bd.relatedBoneIndex,
                        mayaMatrixToBtTransform(world->lastAnchorWorld[kinIndex]));
                }
                ++kinIndex;
                continue;
            }
            if (bd.relatedBoneIndex < 0)
                continue; // no bone to write back to
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
                else if (it != solvedBoneWorld.end() && parentBone != -1 &&
                         i < world->jointPaths.size() && world->jointPaths[i].isValid())
                {
                    // Parent bone has NO body — its world is not in
                    // solvedBoneWorld (no solver entry).  Express the solved
                    // bone world relative to the parent joint's CURRENT world
                    // instead of writing the raw solved WORLD pose as the
                    // joint-local pose: the raw pose, composed through the
                    // parent chain, doubled the skeleton offset and launched
                    // these bones meters above the character (Endmin's
                    // shengzi / jianjia / piaodai chains at y≈14-16 landed at
                    // y≈33).
                    //
                    // The parent's world is reconstructed WITHOUT any DG
                    // pull: walk up from the parent to the nearest bone whose
                    // world IS solver-known (in solvedBoneWorld — the pass-1
                    // solved worlds, which include kinematic anchors), and
                    // compose the gap bones' LOCAL matrices.  Maya
                    // row-vector: world(child) = local(child) * world(parent),
                    // so world(P) = local(P)*...*local(A-child)*world(A).
                    // Gap bones have no body, so reading their LOCAL
                    // matrices (MFnTransform::transformationMatrix — a direct
                    // DAG read) never pulls the solver and cannot cycle, even
                    // when a dynamic ancestor exists.
                    MDagPath parentPath = world->jointPaths[i];
                    if (parentPath.pop() == MS::kSuccess)
                    {
                        btTransform base = btTransform::getIdentity();
                        bool foundBase = false;
                        std::vector<btTransform> locals; // parent-up order
                        MDagPath p = parentPath;
                        for (size_t steps = 0; steps < 256 && p.isValid(); ++steps)
                        {
                            const auto anc = solvedBoneWorld.find(mmd::maya::jointPmxBoneIndex(p));
                            if (anc != solvedBoneWorld.end())
                            {
                                base = anc->second;
                                foundBase = true;
                                break;
                            }
                            MFnTransform tf(p);
                            locals.push_back(mayaMatrixToBtTransform(tf.transformationMatrix()));
                            if (p.pop() != MS::kSuccess)
                                break;
                        }
                        if (foundBase)
                        {
                            btTransform parentWorld = base;
                            for (auto lit = locals.rbegin(); lit != locals.rend(); ++lit)
                            {
                                // The locals are stored as btTransforms, i.e.
                                // TRANSPOSED Maya matrices (column-vector):
                                // btWorld(child) = btWorld(parent) * btLocal(child),
                                // so each gap bone must be POST-multiplied
                                // (parentWorld = parentWorld * local).  The
                                // pre-multiply (local * parentWorld) computes
                                // world(parent) * local instead — identical at
                                // rest (translation-only locals commute) but
                                // wildly wrong once the chain rotates, which
                                // launched the parentless bones meters away
                                // during animation (Endmin shengzi chain).
                                parentWorld = parentWorld * (*lit);
                            }
                            boneLocal = parentWorld.inverse() * it->second;
                        }
                    }
                }
            }
            // (No other fallback: a body with no related bone keeps the raw
            // solved world pose — its joint has no bone to be local to.)

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

    // Guide outputs: every body's CURRENT world pose in the group's space.
    // The pmxRigidBody command connects these to the per-body guide
    // transforms, so the colliders follow the animation (kinematic bodies
    // track their bone; the others track the solved sim pose).  Written for
    // EVERY index (enabled or not) so the array stays dense and aligned with
    // bodyShapes[] — a disabled body falls back to its rest pose in the
    // engine, so its guide sits at rest instead of jumping to the origin.
    MArrayDataBuilder gtBuilder(&dataBlock, RigidBodyNode::aOutGuideTranslate, bodyCount);
    MArrayDataBuilder grBuilder(&dataBlock, RigidBodyNode::aOutGuideRotate, bodyCount);
    if (world)
    {
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        const MMatrix groupInv = world->groupWorld.inverse();
        for (unsigned int i = 0; i < bodyCount; ++i)
        {
            const RigidBodySimulation::Pose wp = world->sim.bodyPose(i);
            Double3 rot;
            quatToEulerXYZDegrees(Double4(wp.quat.x, wp.quat.y, wp.quat.z, wp.quat.w), rot);
            // Body world (row-vector Maya), then into the group's frame so
            // the driven guide (a child of the group) lands where the body
            // actually is.
            const MMatrix bodyWorld =
                mmd::maya::matrixFromTR(mmd::core::Double3(wp.pos.x, wp.pos.y, wp.pos.z), rot);
            const MTransformationMatrix gtm(groupInv * bodyWorld);
            const MVector t = gtm.getTranslation(MSpace::kTransform);
            const MEulerRotation e = gtm.eulerRotation(); // radians

            MDataHandle gtEl = gtBuilder.addElement(i);
            gtEl.child(RigidBodyNode::aOutGuideTranslateX).setMDistance(MDistance(t.x));
            gtEl.child(RigidBodyNode::aOutGuideTranslateY).setMDistance(MDistance(t.y));
            gtEl.child(RigidBodyNode::aOutGuideTranslateZ).setMDistance(MDistance(t.z));

            MDataHandle grEl = grBuilder.addElement(i);
            grEl.child(RigidBodyNode::aOutGuideRotateX)
                .setMAngle(MAngle(e.x * kRadToDeg, MAngle::kDegrees));
            grEl.child(RigidBodyNode::aOutGuideRotateY)
                .setMAngle(MAngle(e.y * kRadToDeg, MAngle::kDegrees));
            grEl.child(RigidBodyNode::aOutGuideRotateZ)
                .setMAngle(MAngle(e.z * kRadToDeg, MAngle::kDegrees));
        }
    }
    MArrayDataHandle gtOut = dataBlock.outputArrayValue(RigidBodyNode::aOutGuideTranslate);
    gtOut.set(gtBuilder);
    gtOut.setAllClean();
    MArrayDataHandle grOut = dataBlock.outputArrayValue(RigidBodyNode::aOutGuideRotate);
    grOut.set(grBuilder);
    grOut.setAllClean();
    dataBlock.outputValue(RigidBodyNode::aOutGuideTranslate).setClean();
    dataBlock.outputValue(RigidBodyNode::aOutGuideRotate).setClean();
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

    // --- per-body shape message array (PMX order) ---
    // Each element connects to a pmxRigidBodyShape node that holds that body's
    // PMX-verbatim data (see rigid_body_shape.{hpp,cpp}); the solver pulls
    // every body from its shape in readBodyData.  The REST pose comes from
    // the shape's bodyRestTranslate/bodyRestRotate attributes; the shape's
    // TRANSFORM is the viewport GUIDE, driven to the body's CURRENT pose each
    // frame by the outGuideTranslate/outGuideRotate outputs (a config change
    // is detected by the per-eval re-read + comparison).
    aBodyShapes = mMsgAttr.create("bodyShapes", "bsh", &stat);
    MMD_CHECK_MSTATUS(stat);
    mMsgAttr.setArray(true);
    mMsgAttr.setReadable(true);
    mMsgAttr.setWritable(true);
    mMsgAttr.setCached(false);
    mMsgAttr.setStorable(true);
    mMsgAttr.setKeyable(false);

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

    // --- guide outputs: every body's CURRENT pose (group space) ---
    // Same unit-typed compound pattern as outTranslate/outRotate so the
    // connections to the per-body guide transform's translate/rotate are
    // direct (no auto-inserted unitConversion).
    MFnUnitAttribute uGuideAttr;
    aOutGuideTranslateX =
        uGuideAttr.create("outGuideTranslateX", "ogtx", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutGuideTranslateY =
        uGuideAttr.create("outGuideTranslateY", "ogty", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutGuideTranslateZ =
        uGuideAttr.create("outGuideTranslateZ", "ogtz", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    aOutGuideTranslate = cAttr.create("outGuideTranslate", "ogtr", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutGuideTranslateX);
    cAttr.addChild(aOutGuideTranslateY);
    cAttr.addChild(aOutGuideTranslateZ);

    aOutGuideRotateX =
        uGuideAttr.create("outGuideRotateX", "ogrx", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutGuideRotateY =
        uGuideAttr.create("outGuideRotateY", "ogry", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutGuideRotateZ =
        uGuideAttr.create("outGuideRotateZ", "ogrz", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    aOutGuideRotate = cAttr.create("outGuideRotate", "ogrt", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutGuideRotateX);
    cAttr.addChild(aOutGuideRotateY);
    cAttr.addChild(aOutGuideRotateZ);

    // --- node attribute registration ---
    stat = addAttribute(aTime);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aGravity);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyShapes);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJoints);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);

    // Make `time` drive the outputs.
    stat = attributeAffects(aTime, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aTime, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aTime, aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aTime, aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);

    // Every config input drives the outputs too, so compute() re-runs on a
    // body/joint/gravity edit (config change detection) and on a kinematic
    // bone drag at a fixed time (the anchor lives on the shape's
    // bodyAnchorWorld).
    stat = attributeAffects(aGravity, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGravity, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGravity, aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aGravity, aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyShapes, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyShapes, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyShapes, aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyShapes, aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutGuideRotate);
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
    const Inputs in = readInputs(thisMObject(), dataBlock);
    mWorld = frame(std::move(mWorld), in, thisMObject(), now);

    return writeOutputs(mWorld, dataBlock);
}
