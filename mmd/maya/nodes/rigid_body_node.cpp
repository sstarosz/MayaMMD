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
 * the outTranslate/outRotate outputs plus each guide's current pose to the
 * outGuide... / outJoint... outputs.
 */

#include "rigid_body_node.hpp"

#include "joint_display.hpp"
#include "maya_utils.hpp"
#include "rigid_body_joint.hpp"
#include "rigid_body_shape.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

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
MObject RigidBodyNode::aJointDisplayScale;
MObject RigidBodyNode::aOutStep;
MObject RigidBodyNode::aOutJointDisplay;
MObject RigidBodyNode::aOutJointHubRadius;
MObject RigidBodyNode::aOutJointActive;
MObject RigidBodyNode::aOutJointBoundsMin;
MObject RigidBodyNode::aOutJointBoundsMax;

MObject RigidBodyNode::aBodyShapes;

MObject RigidBodyNode::aBodyBodies;
MObject RigidBodyNode::aInEnabled;
MObject RigidBodyNode::aInGroupId;
std::array<MObject, 16> RigidBodyNode::aInMaskGroup;
MObject RigidBodyNode::aInColliderType;
MObject RigidBodyNode::aInShapeSize;
MObject RigidBodyNode::aInRestTranslate;
MObject RigidBodyNode::aInRestRotate;
MObject RigidBodyNode::aInMass;
MObject RigidBodyNode::aInLinearDamping;
MObject RigidBodyNode::aInAngularDamping;
MObject RigidBodyNode::aInRestitution;
MObject RigidBodyNode::aInFriction;
MObject RigidBodyNode::aInPhysicsMode;

MObject RigidBodyNode::aAnchorWorlds;

MObject RigidBodyNode::aJointShapes;

MObject RigidBodyNode::aJoints;
MObject RigidBodyNode::aInJointRestTranslate;
MObject RigidBodyNode::aInJointRestRotate;
MObject RigidBodyNode::aInJointLinearMin;
MObject RigidBodyNode::aInJointLinearMax;
MObject RigidBodyNode::aInJointAngularMin;
MObject RigidBodyNode::aInJointAngularMax;
MObject RigidBodyNode::aInJointLinearStiffness;
MObject RigidBodyNode::aInJointAngularStiffness;

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

MObject RigidBodyNode::aOutJointTranslate;
MObject RigidBodyNode::aOutJointTranslateX;
MObject RigidBodyNode::aOutJointTranslateY;
MObject RigidBodyNode::aOutJointTranslateZ;
MObject RigidBodyNode::aOutJointRotate;
MObject RigidBodyNode::aOutJointRotateX;
MObject RigidBodyNode::aOutJointRotateY;
MObject RigidBodyNode::aOutJointRotateZ;

// ===========================================================================
// File-local helpers (pure attribute/plugin reading — no node state)
// ===========================================================================
namespace
{

// Per-evaluation inputs (transient — not part of the node's state).
using Inputs = RigidBodyNode::Inputs;

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
        {
            break;
        }
    }
    // Compose from the cached ancestor (or identity at the root) down to the
    // joint, caching every intermediate rest world by its bone index.
    for (auto it = chain.rbegin(); it != chain.rend(); ++it)
    {
        world = jointRestLocalMatrix(*it) * world;
        const int bone = mmd::maya::jointPmxBoneIndex(*it);
        if (bone >= 0)
        {
            cache.emplace(bone, world);
        }
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
    {
        return false;
    }
    MPlugArray sources;
    MPlug(shapeNode, RigidBodyShape::aBodyJoint).connectedTo(sources, true, false);
    if (sources.length() == 0 || MDagPath::getAPathTo(sources[0].node(), jointPath) != MS::kSuccess)
    {
        return false;
    }
    boneIndex = mmd::maya::jointPmxBoneIndex(jointPath);
    MDagPath parentPath = jointPath;
    if (parentPath.pop() == MS::kSuccess)
    {
        parentBoneIndex = mmd::maya::jointPmxBoneIndex(parentPath);
    }
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
        {
            continue;
        }
        if (b.relatedBoneIndex >= 0)
        {
            boneToAnchor.emplace(b.relatedBoneIndex, kinOrder);
        }
        ++kinOrder;
    }
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        RigidBodySimulation::BodyDefinition& b = bodies[i];
        if (b.isKinematic() || !b.enabled || b.relatedBoneIndex < 0)
        {
            continue;
        }
        MDagPath jp = jointPaths[i];
        if (!jp.isValid())
        {
            continue;
        }
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
            {
                break;
            }
        }
    }
}
// Decode one `bodies[]` element into a core BodyDefinition.  The element is
// fed by a pmxRigidBodyShape's `body` compound, so the values are the
// PMX-verbatim fields: `applyShapeSize` turns PMX shape_size into the engine's
// radius/extents/length, and the collider enum is remapped to the engine's
// PMX-aligned order (the ATTRIBUTE values are persisted in scenes, so they
// cannot change).
void decodeBody(MDataHandle element, RigidBodySimulation::BodyDefinition& out)
{
    const short collider = element.child(RigidBodyNode::aInColliderType).asShort();
    switch (collider)
    {
    case RigidBodyNode::kColliderBox:
        out.colliderType = RigidBodySimulation::ColliderType::eBox;
        break;
    case RigidBodyNode::kColliderSphere:
        out.colliderType = RigidBodySimulation::ColliderType::eSphere;
        break;
    default:
        out.colliderType = RigidBodySimulation::ColliderType::eCapsule;
        break;
    }
    out.restPos = readDouble3(element.child(RigidBodyNode::aInRestTranslate));
    out.restRot = readDouble3(element.child(RigidBodyNode::aInRestRotate));
    mmd::core::applyShapeSize(out, readDouble3(element.child(RigidBodyNode::aInShapeSize)));
    out.mass = element.child(RigidBodyNode::aInMass).asDouble();
    out.linearDamping = element.child(RigidBodyNode::aInLinearDamping).asDouble();
    out.angularDamping = element.child(RigidBodyNode::aInAngularDamping).asDouble();
    out.friction = element.child(RigidBodyNode::aInFriction).asDouble();
    out.restitution = element.child(RigidBodyNode::aInRestitution).asDouble();
    out.mask = 0;
    for (int g = 0; g < 16; ++g)
    {
        if (element.child(RigidBodyNode::aInMaskGroup.at(g)).asBool())
        {
            const auto bits =
                static_cast<unsigned long>(out.mask) | (1UL << static_cast<unsigned int>(g));
            out.mask = static_cast<long>(bits);
        }
    }
    out.groupId = element.child(RigidBodyNode::aInGroupId).asShort();
    // Keep the full PMX physics mode (0/1/2) — kinematic is a derived
    // property and PHYSICS vs PHYSICS_BONE must stay distinguishable.
    out.physicsMode = static_cast<RigidBodySimulation::PhysicsMode>(
        element.child(RigidBodyNode::aInPhysicsMode).asShort());
    out.enabled = element.child(RigidBodyNode::aInEnabled).asBool();
}

// Read the solver's body list from its `bodies[]` INPUT (PMX order — written
// DENSELY by the builder).  Each element carries one PMX-verbatim body, fed
// by the connected pmxRigidBodyShape's `body` compound (pmxRigidBody makes
// that connection), so this is a data-block read: Maya pushed the values once
// for the frame and the node's dirty flags decide when to come back.
//
// The shape NODES still come from the bodyShapes[] message array (identity:
// joints link bodies by shape, and the output slots must not shift when one
// body is deleted).  A slot with no body data — or no shape — stays a
// disabled placeholder.
[[nodiscard]] std::vector<RigidBodySimulation::BodyDefinition>
readBodyData(const MObject& node, MDataBlock& dataBlock, RigidBodyNode::BodyShapes& shapes)
{
    std::vector<RigidBodySimulation::BodyDefinition> out;
    MPlug shapesPlug(node, RigidBodyNode::aBodyShapes);
    const unsigned int shapeCount = shapesPlug.evaluateNumElements();
    MStatus stat;
    MArrayDataHandle data(dataBlock.inputArrayValue(RigidBodyNode::aBodyBodies, &stat));
    if (stat != MS::kSuccess)
    {
        return out;
    }
    unsigned int bodyCount = shapeCount;
    const unsigned int physicalCount = data.elementCount();
    for (unsigned int i = 0; i < physicalCount; ++i)
    {
        data.jumpToArrayElement(i);
        bodyCount = std::max(bodyCount, data.elementIndex() + 1);
    }
    out.reserve(bodyCount);
    for (unsigned int i = 0; i < bodyCount; ++i)
    {
        RigidBodySimulation::BodyDefinition b;
        b.enabled = false;
        MObject shape = MObject::kNullObj;
        if (i < shapeCount)
        {
            MPlugArray sources;
            shapesPlug.elementByLogicalIndex(i).connectedTo(sources, true, false);
            if (sources.length() != 0U)
            {
                shape = sources[0].node();
            }
        }
        // jumpToElement fails on a hole (a deleted body, or a slot that was
        // never fed); the disabled placeholder above survives.
        if (data.jumpToElement(i) == MS::kSuccess)
        {
            MStatus elementStat;
            const MDataHandle element = data.inputValue(&elementStat);
            if (elementStat == MS::kSuccess)
            {
                decodeBody(element, b);
            }
        }
        out.push_back(b);
        shapes.add(shape);
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
    for (unsigned int i = 0; i < bodies.size(); ++i)
    {
        MPlugArray sources;
        shapesPlug.elementByLogicalIndex(i).connectedTo(sources, true, false);
        if (sources.length() == 0)
        {
            continue;
        }
        resolveRelatedBones(sources[0].node(), jointPaths[i], bodies[i].relatedBoneIndex,
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
    {
        return false;
    }
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

// Decode one `joints[]` element (fed by a pmxRigidBodyJoint's `joint`
// compound) into the core definition.  Angular values arrive in radians,
// exactly like the joint's angle3 plugs.
void decodeJoint(MDataHandle element, RigidBodySimulation::JointDefinition& out)
{
    out.frameT = readDouble3(element.child(RigidBodyNode::aInJointRestTranslate));
    out.frameR = readDouble3(element.child(RigidBodyNode::aInJointRestRotate));
    out.linearMin = readDouble3(element.child(RigidBodyNode::aInJointLinearMin));
    out.linearMax = readDouble3(element.child(RigidBodyNode::aInJointLinearMax));
    out.angularMin = readDouble3(element.child(RigidBodyNode::aInJointAngularMin));
    out.angularMax = readDouble3(element.child(RigidBodyNode::aInJointAngularMax));
    out.linearStiffness = readDouble3(element.child(RigidBodyNode::aInJointLinearStiffness));
    out.angularStiffness = readDouble3(element.child(RigidBodyNode::aInJointAngularStiffness));
}

// Read the solver's joint list from its `joints[]` INPUT (PMX order — written
// DENSELY by the builder).  Each element carries one PMX-verbatim joint, fed
// by the connected pmxRigidBodyJoint's `joint` compound
// (pmxRigidBodyConstraint makes that connection), so this is a data-block
// read: Maya pushed the values once for the frame and the node's dirty flags
// decide when to come back.
//
// The joint NODES still come from the jointShapes[] message array (identity:
// the body links are connections, and the output slots must not shift when a
// joint is deleted).  A slot with no joint data — or no node — stays an
// INACTIVE placeholder (bodyA/B = -1), keeping the joint index aligned with
// the array index.  The bodies are connections to shape nodes, resolved to
// body indices through `shapes` (-1 for a missing body).
[[nodiscard]] std::vector<RigidBodySimulation::JointDefinition>
readJointData(const MObject& node, MDataBlock& dataBlock, const RigidBodyNode::BodyShapes& shapes)
{
    std::vector<RigidBodySimulation::JointDefinition> out;
    MPlug shapesPlug(node, RigidBodyNode::aJointShapes);
    const unsigned int jointCount = shapesPlug.evaluateNumElements();
    MStatus stat;
    MArrayDataHandle data(dataBlock.inputArrayValue(RigidBodyNode::aJoints, &stat));
    if (stat != MS::kSuccess)
    {
        return out;
    }
    out.reserve(jointCount);
    for (unsigned int i = 0; i < jointCount; ++i)
    {
        RigidBodySimulation::JointDefinition j;
        bool haveData = false;
        // jumpToElement fails on a hole (a deleted joint, or a slot that was
        // never fed); the inactive placeholder above survives.
        if (data.jumpToElement(i) == MS::kSuccess)
        {
            MStatus elementStat;
            MDataHandle element = data.inputValue(&elementStat);
            if (elementStat == MS::kSuccess)
            {
                decodeJoint(element, j);
                haveData = true;
            }
        }
        MPlugArray sources;
        shapesPlug.elementByLogicalIndex(i).connectedTo(sources, true, false);
        if (haveData && sources.length() != 0U)
        {
            const MObject joint = sources[0].node();
            j.bodyA = shapes.indexOf(RigidBodyJoint::bodyShape(joint, RigidBodyJoint::aJointBodyA));
            j.bodyB = shapes.indexOf(RigidBodyJoint::bodyShape(joint, RigidBodyJoint::aJointBodyB));
        }
        out.push_back(j);
    }
    return out;
}

[[nodiscard]] Double3 readGravity(MDataBlock& dataBlock)
{
    return readDouble3(dataBlock.inputValue(RigidBodyNode::aGravity));
}

// True when `plug` is one of the node's output compounds (or an element or
// child of one) — the only plugs compute() must service.
MObject outputAttribute(MPlug plug)
{
    while (plug.isChild())
    {
        plug = plug.parent();
    }
    return plug.attribute();
}

bool isOutputPlug(const MPlug& plug)
{
    const MObject attribute = outputAttribute(plug);
    return attribute == RigidBodyNode::aOutTranslate || attribute == RigidBodyNode::aOutRotate ||
           attribute == RigidBodyNode::aOutGuideTranslate ||
           attribute == RigidBodyNode::aOutGuideRotate ||
           attribute == RigidBodyNode::aOutJointTranslate ||
           attribute == RigidBodyNode::aOutJointRotate ||
           attribute == RigidBodyNode::aOutJointDisplay || attribute == RigidBodyNode::aOutStep;
}

// ===========================================================================
// Frame policy — pure functions over World / Inputs (no node state)
// ===========================================================================
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
        {
            continue;
        }
        const MDagPath& jointPath = jointPaths[i];
        if (!jointPath.isValid())
        {
            continue;
        }
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
    return (sources.length() != 0U) ? sources[0].node() : MObject::kNullObj;
}

// One evaluation's per-body anchors: anchorWorlds[] indexed by PMX body index
// (identity for a slot the scene leaves unset).  Read through the data block
// so Maya caches the push across the solver pulls of one frame — no per-frame
// cross-node plug evaluation, and no invalidation serial.
[[nodiscard]] std::vector<MMatrix> readAnchorWorlds(MDataBlock& dataBlock, size_t bodyCount)
{
    std::vector<MMatrix> out(bodyCount, MMatrix::identity);
    if (bodyCount == 0)
    {
        return out;
    }
    MStatus stat;
    MArrayDataHandle handles(dataBlock.inputArrayValue(RigidBodyNode::aAnchorWorlds, &stat));
    if (stat != MS::kSuccess)
    {
        return out;
    }
    const unsigned int count = handles.elementCount();
    for (unsigned int i = 0; i < count; ++i)
    {
        handles.jumpToArrayElement(i);
        const unsigned int logical = handles.elementIndex();
        if (logical >= out.size())
        {
            continue;
        }
        out[logical] = handles.inputValue().asMatrix();
    }
    return out;
}

// Returns true when any anchor moved (a bone dragged at the current frame).
//
// The anchor is placed at the RAW world pose: anchorWorlds[i] (the joint
// world) transformed by the constant bodyRest^-1 * jointRest offset — i.e.
// the body's rest pose rigidly attached to the bone's CURRENT world pose.
// This is EXACT for a whole-skeleton move M (anchor -> M·jointWorld ⇒ body ->
// M·bodyRest) and for animation (body follows the posed bone).  The write-back
// (pass 1) therefore reads the kinematic body's pose directly (no K multiply),
// and the raw-anchor reset keeps the weld constraint exactly satisfied.
//
// `rawWorlds` receives each anchor's RAW world matrix (the plug value, before
// the K^-1 rest offset) in kinematic order — the same read feeds the
// whole-skeleton-move detector.
//
// `anchors` is the anchorWorlds[] input, read through the data block by the
// caller (body-indexed; identity for a slot the scene leaves unset).
bool updateKinematicAnchors(World& world, const std::vector<MMatrix>& anchors,
                            std::vector<MMatrix>& rawWorlds)
{
    rawWorlds.clear();
    if (!world.sim.initialized())
    {
        return false;
    }
    rawWorlds.reserve(world.bodies.size());
    bool anchorsMoved = false;
    int anchorIndex = 0;
    for (size_t i = 0; i < world.bodies.size(); ++i)
    {
        const RigidBodySimulation::BodyDefinition& b = world.bodies[i];
        if (!b.isKinematic() || !b.enabled)
        {
            continue;
        }
        MMatrix w = i < anchors.size() ? anchors[i] : MMatrix::identity;
        rawWorlds.push_back(w);
        // RAW rigid attachment (row-vector): the body is a child of its joint
        // at a constant local offset, so bodyWorld = bodyRest * anchorRest^-1 *
        // anchorWorld (bodyRest on the LEFT).  At rest this is bodyRest; under
        // a whole move M (anchor = anchorRest * M) it is bodyRest * M — EXACT,
        // no K conjugation.  (For a boneless pin the anchor IS the body's own
        // rest world, so w stays put.)  The engine reset uses the same form
        // (anchorCurrent * anchorRest^-1 * bodyRest in bt/column = bodyRest *
        // anchorRest^-1 * anchorCurrent in row), so kinematic and dynamic
        // bodies stay weld-consistent.  anchorOffset is that constant
        // bodyRest * anchorRest^-1, resolved at build.
        w = world.anchorOffset.at(i) * w;
        RigidBodySimulation::Pose pose;
        const btTransform t = mayaMatrixToBtTransform(w);
        storePose(pose.pos, pose.quat, t);
        if (world.sim.setKinematicPose(anchorIndex, pose))
        {
            anchorsMoved = true;
        }
        ++anchorIndex;
    }
    return anchorsMoved;
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
    {
        return false;
    }
    for (int c = 0; c < 3; ++c)
    {
        if (a.getBasis().getColumn(c).dot(b.getBasis().getColumn(c)) < 1.0 - rotTol)
        {
            return false;
        }
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
    {
        return std::nullopt;
    }
    std::optional<btTransform> common;
    size_t anchor = 0;
    for (const auto& b : world.bodies)
    {
        if (!b.isKinematic() || !b.enabled)
        {
            continue;
        }
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
    {
        return std::nullopt;
    }
    // The shared move must be a real move (not the identity transform).
    if (transformsNear(*common, btTransform::getIdentity()))
    {
        return std::nullopt;
    }
    return common;
}

// The hub radius every joint of a model shares (core joint_display), from the
// REST distances between the bodies its joints link.  The user's
// jointDisplayScale multiplies the result.
[[nodiscard]] double
resolveHubRadius(const std::vector<RigidBodySimulation::BodyDefinition>& bodies,
                 const std::vector<RigidBodySimulation::JointDefinition>& joints)
{
    std::vector<double> links;
    links.reserve(joints.size());
    const auto valid = [&](long index)
    { return index >= 0 && index < static_cast<long>(bodies.size()); };
    for (const auto& joint : joints)
    {
        if (!valid(joint.bodyA) || !valid(joint.bodyB))
        {
            continue;
        }
        const Double3& a = bodies[joint.bodyA].restPos;
        const Double3& b = bodies[joint.bodyB].restPos;
        const double link = std::sqrt(((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y)) +
                                      ((a.z - b.z) * (a.z - b.z)));
        // A joint whose bodies coincide says nothing about the model's scale.
        if (link > 1e-4)
        {
            links.push_back(link);
        }
    }
    return mmd::core::joint_display::hubRadiusForLinks(std::move(links));
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
                                              const MTime& now, const std::vector<MMatrix>& anchors)
{
    if (in.bodies.empty())
    {
        return std::nullopt;
    }

    // Note: use default-construct + emplace() rather than
    // std::optional<World>(std::in_place): clang-cl's MSVC-STL handling of
    // _SMF_control misreports World as not default-constructible when the
    // enclosing node also owns a std::optional<World> member, so the
    // in_place ctor is SFINAE'd out under clang-tidy.  MSVC accepts both.
    std::optional<World> result;
    result.emplace();
    World& world = *result;
    world.bodies = in.bodies;
    world.bodyShapes = in.bodyShapes;
    world.joints = in.joints;
    world.gravity = in.gravity;

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
    {
        return std::nullopt;
    }

    // The scrub-back reset's "rest" reference: each kinematic anchor's REST
    // world (the joint's composed rest world — a model constant from the
    // stamped pmxRest* attributes; a boneless pin's anchor IS its own rest
    // world, which never moves).  Read before the anchors are applied, and
    // before the raw placement that uses it.
    std::vector<MMatrix> originalAnchors;
    {
        std::map<int, MMatrix> restWorldCache;
        originalAnchors.reserve(world.bodies.size());
        for (size_t body = 0; body < world.bodies.size(); ++body)
        {
            const RigidBodySimulation::BodyDefinition& b = world.bodies[body];
            if (!b.isKinematic() || !b.enabled)
            {
                continue;
            }
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

    // Per-anchor constant for the per-frame update (see World::anchorOffset):
    // the constant rigid attachment bodyRest * anchorRest^-1.
    world.anchorOffset.assign(world.bodies.size(), MMatrix());
    {
        int anchorIndex = 0;
        for (size_t i = 0; i < world.bodies.size(); ++i)
        {
            const RigidBodySimulation::BodyDefinition& b = world.bodies[i];
            if (!b.isKinematic() || !b.enabled)
            {
                continue;
            }
            if (anchorIndex < static_cast<int>(originalAnchors.size()))
            {
                world.anchorOffset[i] =
                    mmd::maya::matrixFromTR(b.restPos, b.restRot) *
                    originalAnchors.at(static_cast<size_t>(anchorIndex)).inverse();
            }
            ++anchorIndex;
        }
    }

    // Apply the anchors with the fresh raw placement; the call also returns
    // each anchor's RAW current world (the same reads feed the move detector).
    std::vector<MMatrix> curAnchors;
    updateKinematicAnchors(world, anchors, curAnchors);

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
    world.lastAnchorWorld = std::move(curAnchors);

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
// or when the time moved somewhere a step cannot carry it: backwards (rebuild
// rather than re-simulate) or forwards past what one step() can simulate.
// A forward jump beyond maxStepSpan() is under-simulated — every chain is left
// lagging behind the skeleton by the remainder, which is the transient seen
// after scrubbing the timeline (or loading a motion on the current frame).
// Rebuilding re-pins the chains to the CURRENT skeleton pose instead, so a
// jumped frame is self-consistent.
[[nodiscard]] bool needsRebuild(const World& world, const Inputs& in, const MTime& now)
{
    if (configChanged(world, in))
    {
        return true;
    }
    const double dt = (now - MTime(world.lastTime, world.lastTimeUnit)).as(MTime::kSeconds);
    return dt < 0.0 || dt > RigidBodySimulation::maxStepSpan();
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
[[nodiscard]] World advanceWithAnchors(World world, const MTime& now,
                                       std::vector<MMatrix> curAnchors, bool anchorsMoved)
{
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
    world.lastAnchorWorld = std::move(curAnchors);
    world.lastTime = now.value();
    world.lastTimeUnit = now.unit();
    return world;
}

// advance() with the anchors the caller already read: frame() needs them to
// decide between a rebuild and a ride, so they are read once per evaluation
// and passed in.
[[nodiscard]] World advance(World world, const MTime& now, const std::vector<MMatrix>& anchors)
{
    std::vector<MMatrix> curAnchors;
    const bool anchorsMoved = updateKinematicAnchors(world, anchors, curAnchors);
    return advanceWithAnchors(std::move(world), now, std::move(curAnchors), anchorsMoved);
}

// The whole frame transition: (re)build the world when its config or time
// demands it, otherwise advance it.  The previous world is moved in and the
// result replaces it — state is never mutated piecemeal.
[[nodiscard]] std::optional<World> frame(std::optional<World> world, const Inputs& in,
                                         const MObject& node, const MTime& now,
                                         const std::vector<MMatrix>& anchors)
{
    if (!world || needsRebuild(*world, in, now))
    {
        // The rebuild derives its own REST reference from the stamped
        // pmxRest* attributes (buildWorld reads the joint rest worlds — a
        // model constant, so the pinning is identical whether the world is
        // first built, rebuilt on a config change, or rebuilt on a scrub-back;
        // a moved/posed skeleton is never baked into the offset).
        return buildWorld(node, in, now, anchors);
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
        std::vector<MMatrix> curAnchors;
        const bool anchorsMoved = updateKinematicAnchors(*world, anchors, curAnchors);
        if (!anchorsMoved)
        {
            // A repeat evaluation of the same frame with the same skeleton
            // pose: nothing to simulate, and in particular no ride-along (that
            // would zero every dynamic body's velocity once per repeat).  Maya
            // pulls the solver several times per frame, so this is the common
            // case, not an edge case.
            return world;
        }
        if (!detectWholeSkeletonMove(*world, world->lastAnchorWorld, curAnchors).has_value())
        {
            return buildWorld(node, in, now, anchors);
        }
        // A whole-skeleton move at a paused frame: ride, and reuse the anchors
        // just read instead of reading them a second time.
        return advanceWithAnchors(std::move(*world), now, std::move(curAnchors), anchorsMoved);
    }
    return advance(std::move(*world), now, anchors);
}

// Display configuration is derived directly from the authored inputs. Asking
// for it never advances or rebuilds the Bullet world.
MStatus writeJointDisplay(const Inputs& in, MDataBlock& dataBlock)
{
    const double radius =
        resolveHubRadius(in.bodies, in.joints) *
        std::max(0.05, dataBlock.inputValue(RigidBodyNode::aJointDisplayScale).asDouble());
    const auto count = static_cast<unsigned int>(in.joints.size());
    MArrayDataBuilder builder(&dataBlock, RigidBodyNode::aOutJointDisplay, count);
    namespace display = mmd::core::joint_display;
    const double pad = std::max(radius, 2.25 * display::guideSizes(radius).radius);
    for (unsigned int i = 0; i < count; ++i)
    {
        const auto& joint = in.joints[i];
        const auto ranges = display::describe(joint);
        std::array<double, 3> low{-pad, -pad, -pad};
        std::array<double, 3> high{pad, pad, pad};
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            const auto& range = ranges.linear.at(axis);
            if (range.kind() != display::RangeKind::Free)
            {
                low.at(axis) += std::min(range.lower, 0.0);
                high.at(axis) += std::max(range.upper, 0.0);
            }
        }
        MDataHandle element = builder.addElement(i);
        element.child(RigidBodyNode::aOutJointHubRadius).setDouble(radius);
        element.child(RigidBodyNode::aOutJointActive)
            .setBool(RigidBodySimulation::jointStatus(joint, in.bodies) ==
                     RigidBodySimulation::JointStatus::eActive);
        element.child(RigidBodyNode::aOutJointBoundsMin).set3Double(low[0], low[1], low[2]);
        element.child(RigidBodyNode::aOutJointBoundsMax).set3Double(high[0], high[1], high[2]);
    }
    MArrayDataHandle output = dataBlock.outputArrayValue(RigidBodyNode::aOutJointDisplay);
    output.set(builder);
    output.setAllClean();
    dataBlock.outputValue(RigidBodyNode::aOutJointDisplay).setClean();
    return MS::kSuccess;
}

// Write each dynamic body's solved local pose to outTranslate/outRotate.
// An empty world (no bodies) writes empty arrays.
//
// The guide outputs are expressed in the RigidBodies group's space (the
// solver's DAG parent — the guides hang one level deeper, under the
// {model}_Bodies sub-group, which must stay at identity).  The Bullet world
// runs in WORLD space, so each body's solved pose is brought into the
// group's frame.  `groupWorld` is the group's CURRENT world matrix: one
// cached at build time would show a model-root move twice (the solved pose
// already carries it, and the guide's parent moves with the root too).
MStatus writeOutputs(const std::optional<World>& world, const MMatrix& groupWorld,
                     MDataBlock& dataBlock)
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
        // bone index -> solved bone world, filled by pass 1 and read by pass 2.
        // This used to be a std::map, which allocated a tree node per body on
        // every call — and Maya pulls the outputs several times per frame.
        // Bone indices are PMX indices, so an index vector is enough; an index
        // outside it is simply "absent", exactly like a failed map lookup.
        struct BoneWorlds
        {
            std::vector<btTransform> worlds;
            std::vector<char> present;
            void set(int bone, const btTransform& world)
            {
                if (bone < 0 || bone >= static_cast<int>(worlds.size()))
                {
                    return;
                }
                worlds.at(static_cast<size_t>(bone)) = world;
                present.at(static_cast<size_t>(bone)) = 1;
            }
            [[nodiscard]] bool has(int bone) const
            {
                return bone >= 0 && bone < static_cast<int>(worlds.size()) &&
                       present.at(static_cast<size_t>(bone)) != 0;
            }
            [[nodiscard]] const btTransform& get(int bone) const
            {
                return worlds.at(static_cast<size_t>(bone));
            }
        };
        BoneWorlds solvedBoneWorld;
        {
            int maxBone = 0;
            for (const auto& bd : world->bodies)
            {
                maxBone = std::max(maxBone, bd.relatedBoneIndex);
            }
            const auto slots = static_cast<size_t>(maxBone) + 1;
            solvedBoneWorld.worlds.resize(slots);
            solvedBoneWorld.present.assign(slots, 0);
        }
        size_t kinIndex = 0; // kinematic-order counter, aligned with lastAnchorWorld
        for (size_t i = 0; i < world->bodies.size(); ++i)
        {
            const RigidBodySimulation::BodyDefinition& bd = world->bodies[i];
            if (!bd.enabled)
            {
                continue; // disabled bodies have no slot in lastAnchorWorld
            }
            if (bd.isKinematic())
            {
                // Every ENABLED kinematic body occupies a slot in
                // lastAnchorWorld (readRawAnchorWorlds counts all of them,
                // boneless pins and duplicate-bone bodies included), so the
                // counter advances in lockstep with it.
                if (bd.relatedBoneIndex >= 0 && !solvedBoneWorld.has(bd.relatedBoneIndex) &&
                    kinIndex < world->lastAnchorWorld.size())
                {
                    // First body on this bone — its bone world IS the raw
                    // anchor world (the joint world).
                    solvedBoneWorld.set(bd.relatedBoneIndex,
                                        mayaMatrixToBtTransform(world->lastAnchorWorld[kinIndex]));
                }
                ++kinIndex;
                continue;
            }
            if (bd.relatedBoneIndex < 0)
            {
                continue; // no bone to write back to
            }
            if (solvedBoneWorld.has(bd.relatedBoneIndex))
            {
                continue; // first body on the bone wins
            }
            if (i >= world->k.size())
            {
                continue; // defensive — K is derived for every body at build
            }
            const btTransform kb = mayaMatrixToBtTransform(world->k[i]);
            const RigidBodySimulation::Pose wp = world->sim.bodyPose(i);
            solvedBoneWorld.set(bd.relatedBoneIndex, poseToTransform(wp.pos, wp.quat) * kb);
        }

        // Pass 2 — write the joint-local pose, or the raw solved world pose
        // when the parent bone has no body (rare in well-formed chains).
        for (size_t i = 0; i < world->bodies.size(); ++i)
        {
            const RigidBodySimulation::BodyDefinition& bd = world->bodies[i];
            if (bd.isKinematic() || !bd.enabled)
            {
                continue;
            }

            const RigidBodySimulation::Pose wp = world->sim.bodyPose(i);
            btTransform boneLocal = poseToTransform(wp.pos, wp.quat);

            const int bone = bd.relatedBoneIndex;
            if (bone >= 0)
            {
                const int parentBone = bd.parentBoneIndex;
                const bool hasBone = solvedBoneWorld.has(bone);
                const bool hasParent = solvedBoneWorld.has(parentBone);
                if (hasBone && hasParent)
                {
                    // boneLocal = solvedBoneWorld[parentBone]^-1 * solvedBoneWorld[bone]
                    boneLocal =
                        solvedBoneWorld.get(parentBone).inverse() * solvedBoneWorld.get(bone);
                }
                else if (parentBone == -1 && hasBone)
                {
                    // Root bone: its parent is the model root at identity, so
                    // the joint-local pose IS the solved bone world (falling
                    // through to the raw body pose would bake the K offset
                    // into the joint).
                    boneLocal = solvedBoneWorld.get(bone);
                }
                else if (hasBone && parentBone != -1 && i < world->jointPaths.size() &&
                         world->jointPaths[i].isValid())
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
                            const int ancestor = mmd::maya::jointPmxBoneIndex(p);
                            if (solvedBoneWorld.has(ancestor))
                            {
                                base = solvedBoneWorld.get(ancestor);
                                foundBase = true;
                                break;
                            }
                            MFnTransform tf(p);
                            locals.push_back(mayaMatrixToBtTransform(tf.transformationMatrix()));
                            if (p.pop() != MS::kSuccess)
                            {
                                break;
                            }
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
                            boneLocal = parentWorld.inverse() * solvedBoneWorld.get(bone);
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
                MDataHandle tEl = tBuilder.addElement(static_cast<unsigned int>(i));
                tEl.child(RigidBodyNode::aOutTranslateX).setMDistance(MDistance(o.x()));
                tEl.child(RigidBodyNode::aOutTranslateY).setMDistance(MDistance(o.y()));
                tEl.child(RigidBodyNode::aOutTranslateZ).setMDistance(MDistance(o.z()));
            }

            MDataHandle rEl = rBuilder.addElement(static_cast<unsigned int>(i));
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
        const MMatrix groupInv = groupWorld.inverse();
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

    // Joint guide outputs: every joint's live attachment on body A, in the
    // group's space.  pmxRigidBodyConstraint connects these to the joint guide
    // transforms, so the node's pivot follows the drawn hub — exactly like a
    // body guide.  Written for EVERY index (active or not) so the array stays
    // dense and aligned with jointShapes[]; a joint with no live attachment
    // falls back to its rest frame instead of jumping to the origin.
    const auto jointCount = world ? static_cast<unsigned int>(world->joints.size()) : 0U;
    MArrayDataBuilder jtBuilder(&dataBlock, RigidBodyNode::aOutJointTranslate, jointCount);
    MArrayDataBuilder jrBuilder(&dataBlock, RigidBodyNode::aOutJointRotate, jointCount);
    if (world)
    {
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        const MMatrix groupInv = groupWorld.inverse();
        for (unsigned int i = 0; i < jointCount; ++i)
        {
            const RigidBodySimulation::JointDefinition& joint = world->joints[i];
            Double3 pos = joint.frameT;
            Double3 rot = joint.frameR;
            RigidBodySimulation::Pose attachment;
            if (joint.bodyA >= 0 && joint.bodyB >= 0 &&
                world->sim.attachmentPose(joint.bodyA, joint.frameT, joint.frameR, attachment))
            {
                pos = attachment.pos;
                quatToEulerXYZDegrees(Double4(attachment.quat.x, attachment.quat.y,
                                              attachment.quat.z, attachment.quat.w),
                                      rot);
            }
            // Attachment world (row-vector Maya), then into the group's frame
            // so the driven guide (a child of the group) lands on the hub.
            const MMatrix jointWorld = mmd::maya::matrixFromTR(pos, rot);
            const MTransformationMatrix gtm(groupInv * jointWorld);
            const MVector t = gtm.getTranslation(MSpace::kTransform);
            const MEulerRotation e = gtm.eulerRotation(); // radians

            MDataHandle jtEl = jtBuilder.addElement(i);
            jtEl.child(RigidBodyNode::aOutJointTranslateX).setMDistance(MDistance(t.x));
            jtEl.child(RigidBodyNode::aOutJointTranslateY).setMDistance(MDistance(t.y));
            jtEl.child(RigidBodyNode::aOutJointTranslateZ).setMDistance(MDistance(t.z));

            MDataHandle jrEl = jrBuilder.addElement(i);
            jrEl.child(RigidBodyNode::aOutJointRotateX)
                .setMAngle(MAngle(e.x * kRadToDeg, MAngle::kDegrees));
            jrEl.child(RigidBodyNode::aOutJointRotateY)
                .setMAngle(MAngle(e.y * kRadToDeg, MAngle::kDegrees));
            jrEl.child(RigidBodyNode::aOutJointRotateZ)
                .setMAngle(MAngle(e.z * kRadToDeg, MAngle::kDegrees));
        }
    }
    MArrayDataHandle jtOut = dataBlock.outputArrayValue(RigidBodyNode::aOutJointTranslate);
    jtOut.set(jtBuilder);
    jtOut.setAllClean();
    MArrayDataHandle jrOut = dataBlock.outputArrayValue(RigidBodyNode::aOutJointRotate);
    jrOut.set(jrBuilder);
    jrOut.setAllClean();
    dataBlock.outputValue(RigidBodyNode::aOutJointTranslate).setClean();
    dataBlock.outputValue(RigidBodyNode::aOutJointRotate).setClean();
    return MS::kSuccess;
}

} // namespace

// ===========================================================================
// Node lifecycle
// ===========================================================================
RigidBodyNode::RigidBodyNode() = default;

void RigidBodyNode::BodyShapes::add(const MObject& shape)
{
    if (!shape.isNull())
    {
        const auto hash = MObjectHandle(shape).hashCode();
        const auto index = static_cast<long>(nodes.size());
        const auto at =
            std::lower_bound(byHash.begin(), byHash.end(), hash,
                             [](const auto& entry, unsigned int key) { return entry.first < key; });
        byHash.insert(at, {hash, index});
    }
    nodes.emplace_back(shape);
}

long RigidBodyNode::BodyShapes::indexOf(const MObject& shape) const
{
    if (shape.isNull())
    {
        return -1;
    }
    // Hash codes can collide: confirm the hit, and scan when it is not ours.
    const auto hash = MObjectHandle(shape).hashCode();
    const auto at =
        std::lower_bound(byHash.begin(), byHash.end(), hash,
                         [](const auto& entry, unsigned int key) { return entry.first < key; });
    if (at != byHash.end() && at->first == hash &&
        nodes.at(static_cast<size_t>(at->second)).object() == shape)
    {
        return at->second;
    }
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        if (nodes.at(i).object() == shape)
        {
            return static_cast<long>(i);
        }
    }
    return -1;
}

MMatrix RigidBodyNode::groupWorldMatrix(const MObject& solver)
{
    MDagPath solverPath;
    if (MDagPath::getAPathTo(solver, solverPath) != MS::kSuccess ||
        solverPath.pop() != MS::kSuccess)
    {
        return MMatrix::identity;
    }
    // inclusiveMatrix() lives on MDagPath (not the function set).
    return solverPath.inclusiveMatrix();
}

bool RigidBodyNode::jointViewportFrames(const RigidBodySimulation::JointDefinition& joint,
                                        MMatrix& bodyA, MMatrix& bodyB, MMatrix& attachmentA,
                                        MMatrix& attachmentB) const
{
    if (!mWorld || joint.bodyA < 0 || joint.bodyB < 0)
    {
        return false;
    }
    RigidBodySimulation::Pose a;
    RigidBodySimulation::Pose b;
    if (!mWorld->sim.attachmentPose(joint.bodyA, joint.frameT, joint.frameR, a) ||
        !mWorld->sim.attachmentPose(joint.bodyB, joint.frameT, joint.frameR, b))
    {
        return false;
    }
    const auto poseMatrix = [](const RigidBodySimulation::Pose& pose)
    {
        // Rotation from the quaternion, translation in the last ROW (Maya's
        // matrices are row-vector).  MTransformationMatrix would do the same
        // through scale/shear/setRotation/setTranslation and validate each
        // step — this runs four times per joint per frame.
        MMatrix out = MQuaternion(pose.quat.x, pose.quat.y, pose.quat.z, pose.quat.w).asMatrix();
        out(3, 0) = pose.pos.x;
        out(3, 1) = pose.pos.y;
        out(3, 2) = pose.pos.z;
        return out;
    };
    bodyA = poseMatrix(mWorld->sim.bodyPose(joint.bodyA));
    bodyB = poseMatrix(mWorld->sim.bodyPose(joint.bodyB));
    attachmentA = poseMatrix(a);
    attachmentB = poseMatrix(b);
    return true;
}

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

// Angle-triple attribute (like a transform's rotate): stored in radians, shown
// and set from scripts in the user's UI angle unit.  The joint node's
// jointAngularMin/Max are the same shape, so a compound connection pairs them.
MObject makeAngle3Numeric(MFnNumericAttribute& fn, const MString& longName,
                          const MString& shortName)
{
    MStatus stat;
    MFnUnitAttribute uAttr;
    std::array<MObject, 3> axes{};
    const std::array<const char*, 3> suffix{"X", "Y", "Z"};
    const std::array<const char*, 3> shortSuffix{"x", "y", "z"};
    for (int i = 0; i < 3; ++i)
    {
        const auto index = static_cast<std::size_t>(i);
        axes.at(index) =
            uAttr.create(longName + suffix.at(index), shortName + shortSuffix.at(index),
                         MFnUnitAttribute::kAngle, 0.0, &stat);
        MMD_CHECK_MSTATUS(stat);
        uAttr.setStorable(true);
        uAttr.setKeyable(false);
    }
    MObject attr = fn.create(longName, shortName, axes.at(0), axes.at(1), axes.at(2), &stat);
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

    // --- joint glyph size multiplier (viewport only) ---
    aJointDisplayScale =
        makeNumeric(nAttr, "jointDisplayScale", "jdsc", MFnNumericData::kDouble, 1.0);
    nAttr.setMin(0.05);
    nAttr.setDefault(1.0);

    // --- joints' viewport dependency on the simulation (see header) ---
    aOutStep = nAttr.create("outStep", "ostp", MFnNumericData::kDouble, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    nAttr.setHidden(true);

    // --- per-body shape message array (PMX order) ---
    // Each element connects to the pmxRigidBodyShape node of that body.  This
    // is IDENTITY, not data: joints reference bodies by shape, and a deleted
    // body must leave a placeholder instead of shifting every output slot.
    // The body's PMX fields arrive separately, through `bodies[]` below.  The
    // shape's TRANSFORM is the viewport GUIDE, driven to the body's CURRENT
    // pose each frame by the outGuideTranslate/outGuideRotate outputs.
    aBodyShapes = mMsgAttr.create("bodyShapes", "bsh", &stat);
    MMD_CHECK_MSTATUS(stat);
    mMsgAttr.setArray(true);
    mMsgAttr.setReadable(true);
    mMsgAttr.setWritable(true);
    mMsgAttr.setCached(false);
    mMsgAttr.setStorable(true);
    mMsgAttr.setKeyable(false);

    // --- per-joint shape message array (PMX order) ---
    // Each element connects to a pmxRigidBodyJoint node.  This is IDENTITY,
    // not data: the joint's body links are resolved from its messages, the
    // output slots stay aligned, and the joint display asks the node.  The
    // joint's PMX VALUES arrive through `joints[]` below.
    aJointShapes = mMsgAttr.create("jointShapes", "js", &stat);
    MMD_CHECK_MSTATUS(stat);
    mMsgAttr.setArray(true);
    mMsgAttr.setReadable(true);
    mMsgAttr.setWritable(true);
    mMsgAttr.setCached(false);
    mMsgAttr.setStorable(true);
    mMsgAttr.setKeyable(false);

    // --- per-joint simulation inputs, as one compound array (PMX order) ---
    // Each element is fed by the matching pmxRigidBodyJoint's `joint` compound
    // (one connectAttr per joint in pmxRigidBodyConstraint).  The children
    // repeat the joint node's long names because a compound-to-compound
    // connection pairs them by name; their TYPES must match too (the angular
    // limits are angle compounds).
    {
        MFnCompoundAttribute jointsAttr;
        aJoints = jointsAttr.create("joints", "jts", &stat);
        MMD_CHECK_MSTATUS(stat);
        jointsAttr.setArray(true);
        jointsAttr.setReadable(true);
        jointsAttr.setWritable(true);
        jointsAttr.setStorable(true);
        jointsAttr.setKeyable(false);

        aInJointRestTranslate =
            makeNumeric(nAttr, "jointRestTranslate", "jrt", MFnNumericData::k3Double, 0.0);
        aInJointRestRotate =
            makeNumeric(nAttr, "jointRestRotate", "jrr", MFnNumericData::k3Double, 0.0);
        aInJointLinearMin =
            makeNumeric(nAttr, "jointLinearMin", "jlmn", MFnNumericData::k3Double, 0.0);
        aInJointLinearMax =
            makeNumeric(nAttr, "jointLinearMax", "jlmx", MFnNumericData::k3Double, 0.0);
        aInJointAngularMin = makeAngle3Numeric(nAttr, "jointAngularMin", "jamn");
        aInJointAngularMax = makeAngle3Numeric(nAttr, "jointAngularMax", "jamx");
        aInJointLinearStiffness =
            makeNumeric(nAttr, "jointLinearStiffness", "jlsk", MFnNumericData::k3Double, 0.0);
        aInJointAngularStiffness =
            makeNumeric(nAttr, "jointAngularStiffness", "jask", MFnNumericData::k3Double, 0.0);

        jointsAttr.addChild(aInJointRestTranslate);
        jointsAttr.addChild(aInJointRestRotate);
        jointsAttr.addChild(aInJointLinearMin);
        jointsAttr.addChild(aInJointLinearMax);
        jointsAttr.addChild(aInJointAngularMin);
        jointsAttr.addChild(aInJointAngularMax);
        jointsAttr.addChild(aInJointLinearStiffness);
        jointsAttr.addChild(aInJointAngularStiffness);
    }

    // --- per-body simulation inputs, as one compound array (PMX order) ---
    // Each element is fed by the matching pmxRigidBodyShape's `body` compound
    // (one connectAttr per body in pmxRigidBody).  The children repeat the
    // shape's long names because a compound-to-compound connection pairs them
    // by name; their TYPES must match too (enum field sets included).
    {
        MFnCompoundAttribute bodiesAttr;
        aBodyBodies = bodiesAttr.create("bodies", "bds", &stat);
        MMD_CHECK_MSTATUS(stat);
        bodiesAttr.setArray(true);
        bodiesAttr.setReadable(true);
        bodiesAttr.setWritable(true);
        bodiesAttr.setStorable(true);
        bodiesAttr.setKeyable(false);

        aInEnabled = makeNumeric(nAttr, "bodyEnabled", "ben", MFnNumericData::kBoolean, 1.0);
        {
            MFnEnumAttribute groupAttr;
            aInGroupId = groupAttr.create("bodyGroupId", "bgid", 0, &stat);
            MMD_CHECK_MSTATUS(stat);
            for (int g = 0; g < 16; ++g)
            {
                const MString name(("Group " + std::to_string(g)).c_str());
                groupAttr.addField(name, static_cast<short>(g));
            }
            groupAttr.setStorable(true);
            groupAttr.setKeyable(false);
        }
        for (int g = 0; g < 16; ++g)
        {
            const MString lname(("bodyMaskGroup" + std::to_string(g)).c_str());
            const MString sname(("bmg" + std::to_string(g)).c_str());
            aInMaskGroup.at(g) = makeNumeric(nAttr, lname, sname, MFnNumericData::kBoolean, 1.0);
        }
        {
            MFnEnumAttribute colliderAttr;
            aInColliderType =
                colliderAttr.create("bodyColliderType", "bct", RigidBodyNode::kColliderBox, &stat);
            MMD_CHECK_MSTATUS(stat);
            colliderAttr.addField("Box", RigidBodyNode::kColliderBox);
            colliderAttr.addField("Sphere", RigidBodyNode::kColliderSphere);
            colliderAttr.addField("Capsule", RigidBodyNode::kColliderCapsule);
            colliderAttr.setStorable(true);
            colliderAttr.setKeyable(false);
        }
        aInShapeSize = makeNumeric(nAttr, "bodyShapeSize", "bss", MFnNumericData::k3Double, 1.0);
        aInRestTranslate =
            makeNumeric(nAttr, "bodyRestTranslate", "brt", MFnNumericData::k3Double, 0.0);
        aInRestRotate = makeNumeric(nAttr, "bodyRestRotate", "brr", MFnNumericData::k3Double, 0.0);
        aInMass = makeNumeric(nAttr, "bodyMass", "bm", MFnNumericData::kDouble, 1.0);
        aInLinearDamping =
            makeNumeric(nAttr, "bodyLinearDamping", "bld", MFnNumericData::kDouble, 0.0);
        aInAngularDamping =
            makeNumeric(nAttr, "bodyAngularDamping", "bad", MFnNumericData::kDouble, 0.0);
        aInRestitution = makeNumeric(nAttr, "bodyRestitution", "bre", MFnNumericData::kDouble, 0.0);
        aInFriction = makeNumeric(nAttr, "bodyFriction", "bfr", MFnNumericData::kDouble, 0.5);
        {
            MFnEnumAttribute modeAttr;
            aInPhysicsMode = modeAttr.create(
                "bodyPhysicsMode", "bpm",
                static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysics), &stat);
            MMD_CHECK_MSTATUS(stat);
            modeAttr.addField("FollowBone",
                              static_cast<short>(RigidBodySimulation::PhysicsMode::eFollowBone));
            modeAttr.addField("Physics",
                              static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysics));
            modeAttr.addField("PhysicsBone",
                              static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysicsBone));
            modeAttr.setStorable(true);
            modeAttr.setKeyable(false);
        }

        bodiesAttr.addChild(aInEnabled);
        bodiesAttr.addChild(aInGroupId);
        for (MObject& mask : aInMaskGroup)
        {
            bodiesAttr.addChild(mask);
        }
        bodiesAttr.addChild(aInColliderType);
        bodiesAttr.addChild(aInShapeSize);
        bodiesAttr.addChild(aInRestTranslate);
        bodiesAttr.addChild(aInRestRotate);
        bodiesAttr.addChild(aInMass);
        bodiesAttr.addChild(aInLinearDamping);
        bodiesAttr.addChild(aInAngularDamping);
        bodiesAttr.addChild(aInRestitution);
        bodiesAttr.addChild(aInFriction);
        bodiesAttr.addChild(aInPhysicsMode);
    }

    // --- per-body kinematic anchor (PMX order) ---
    // anchorWorlds[i] is fed by the related joint's worldMatrix[0] (a real
    // data connection, wired by pmxRigidBody) and read through the data
    // block, so a bone move reaches compute() as ordinary input dirt and the
    // value is cached by Maya across the pulls of one frame.  A boneless pin
    // stores its rest world directly (no connection).  See aAnchorWorlds in
    // the header, including why a write-back-driven bone cannot feed it.
    aAnchorWorlds = mAttr.create("anchorWorlds", "awld", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setArray(true);
    mAttr.setReadable(true);
    mAttr.setWritable(true);
    mAttr.setStorable(true);
    mAttr.setKeyable(false);

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

    // --- joint guide outputs: each joint's live attachment on body A ---
    // Same unit-typed compound pattern as outGuideTranslate/outGuideRotate, so
    // the connections to the joint guide transform's translate/rotate are
    // direct.  Written for every joint index.
    MFnUnitAttribute uJointAttr;
    aOutJointTranslateX =
        uJointAttr.create("outJointTranslateX", "ojtx", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutJointTranslateY =
        uJointAttr.create("outJointTranslateY", "ojty", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutJointTranslateZ =
        uJointAttr.create("outJointTranslateZ", "ojtz", MFnUnitAttribute::kDistance, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    aOutJointTranslate = cAttr.create("outJointTranslate", "ojtr", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutJointTranslateX);
    cAttr.addChild(aOutJointTranslateY);
    cAttr.addChild(aOutJointTranslateZ);

    aOutJointRotateX =
        uJointAttr.create("outJointRotateX", "ojrx", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutJointRotateY =
        uJointAttr.create("outJointRotateY", "ojry", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    aOutJointRotateZ =
        uJointAttr.create("outJointRotateZ", "ojrz", MFnUnitAttribute::kAngle, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);

    aOutJointRotate = cAttr.create("outJointRotate", "ojrt", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.addChild(aOutJointRotateX);
    cAttr.addChild(aOutJointRotateY);
    cAttr.addChild(aOutJointRotateZ);

    // Derived joint display values. The child names match jointDisplay on
    // pmxRigidBodyJoint so one compound connection carries the whole record.
    aOutJointHubRadius = nAttr.create("jointHubRadius", "jhr", MFnNumericData::kDouble,
                                      mmd::core::joint_display::kFallbackHubRadius, &stat);
    MMD_CHECK_MSTATUS(stat);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    aOutJointActive = nAttr.create("jointActive", "jac", MFnNumericData::kBoolean, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    aOutJointBoundsMin = nAttr.create("jointBoundsMin", "jbmn", MFnNumericData::k3Double,
                                      -mmd::core::joint_display::kFallbackHubRadius, &stat);
    MMD_CHECK_MSTATUS(stat);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    aOutJointBoundsMax = nAttr.create("jointBoundsMax", "jbmx", MFnNumericData::k3Double,
                                      mmd::core::joint_display::kFallbackHubRadius, &stat);
    MMD_CHECK_MSTATUS(stat);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    aOutJointDisplay = cAttr.create("outJointDisplay", "ojd", &stat);
    MMD_CHECK_MSTATUS(stat);
    cAttr.setArray(true);
    cAttr.setUsesArrayDataBuilder(true);
    cAttr.setWritable(false);
    cAttr.setStorable(false);
    cAttr.setHidden(true);
    cAttr.addChild(aOutJointHubRadius);
    cAttr.addChild(aOutJointActive);
    cAttr.addChild(aOutJointBoundsMin);
    cAttr.addChild(aOutJointBoundsMax);

    // --- node attribute registration ---
    stat = addAttribute(aTime);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aGravity);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointDisplayScale);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutStep);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutJointDisplay);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyShapes);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointShapes);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJoints);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyBodies);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aAnchorWorlds);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutJointTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aOutJointRotate);
    MMD_CHECK_MSTATUS(stat);

    // Display configuration has no dependency on time or animated anchors.
    for (const MObject& input :
         {aBodyShapes, aBodyBodies, aJointShapes, aJoints, aJointDisplayScale})
    {
        stat = attributeAffects(input, aOutJointDisplay);
        MMD_CHECK_MSTATUS(stat);
    }

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
    // bone drag at a fixed time (the anchors are a real input: anchorWorlds[]).
    // Body and joint config edits arrive through the data-block arrays
    // (bodies[] / joints[]); the message arrays report identity changes.
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
    stat = attributeAffects(aBodyBodies, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyBodies, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyBodies, aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aBodyBodies, aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJointShapes, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJointShapes, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJointShapes, aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJointShapes, aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aJoints, aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorWorlds, aOutTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorWorlds, aOutRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorWorlds, aOutGuideTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = attributeAffects(aAnchorWorlds, aOutGuideRotate);
    MMD_CHECK_MSTATUS(stat);
    // The joint guides follow the sim too (attachment A moves every step).
    for (const MObject& input : {
             aTime,
             aGravity,
             aBodyShapes,
             aJointShapes,
             aJoints,
             aBodyBodies,
             aAnchorWorlds,
         })
    {
        stat = attributeAffects(input, aOutJointTranslate);
        MMD_CHECK_MSTATUS(stat);
        stat = attributeAffects(input, aOutJointRotate);
        MMD_CHECK_MSTATUS(stat);
    }
    // Selected overlays need the current simulation snapshot. Display-only
    // settings reach outJointDisplay instead of dirtying the step output.
    for (const MObject& input : {
             aTime,
             aGravity,
             aBodyShapes,
             aJointShapes,
             aJoints,
             aBodyBodies,
             aAnchorWorlds,
         })
    {
        stat = attributeAffects(input, aOutStep);
        MMD_CHECK_MSTATUS(stat);
    }

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

MStatus RigidBodyNode::setDependentsDirty(const MPlug& plug, MPlugArray& affected)
{
    // Only element dirts on the body/joint IDENTITY arrays can be "nothing
    // changed": the kinematic anchors are a real input (anchorWorlds[]) and are
    // read every evaluation, so a bodyShapes[]/jointShapes[] element dirt is a
    // re-bind (inputStale's slot check catches it) — never per-frame data.
    // Everything else (gravity and the arrays themselves) is
    // configuration and needs the inputs read again — except the timeline, the
    // anchors, and the body/joint records (which are data-block inputs).
    const MObject attribute = plug.attribute();
    if (attribute == aBodyBodies)
    {
        // The body records arrived dirty: re-read them from the data block.
        mBodiesDirty.store(true, std::memory_order_relaxed);
        return MPxLocatorNode::setDependentsDirty(plug, affected);
    }
    if (attribute == aJoints)
    {
        // The joint records arrived dirty: re-read them from the data block.
        mJointsDirty.store(true, std::memory_order_relaxed);
        return MPxLocatorNode::setDependentsDirty(plug, affected);
    }
    if (attribute == aTime || attribute == aAnchorWorlds || attribute == aJointDisplayScale)
    {
        // Time and anchors are read per simulation evaluation. Display scale
        // is read only by writeJointDisplay; none belongs in the config cache.
        return MPxLocatorNode::setDependentsDirty(plug, affected);
    }
    if ((attribute == aBodyShapes || attribute == aJointShapes) && plug.isElement())
    {
        std::vector<bool>& slots = (attribute == aBodyShapes) ? mBodySlotDirty : mJointSlotDirty;
        const auto slot = static_cast<size_t>(plug.logicalIndex());
        if (slot >= slots.size())
        {
            slots.resize(slot + 1, false);
        }
        slots[slot] = true;
    }
    else
    {
        // Any other input (gravity, the arrays
        // themselves) is configuration: read the input set again.
        mInputsValid = false;
    }
    return MPxLocatorNode::setDependentsDirty(plug, affected);
}

// The Evaluation Manager can evaluate the node without calling
// setDependentsDirty (Parallel mode is Maya's default), so the bodies[]/joints[]
// dirt is picked up here as well — otherwise a body or joint edit during
// playback would keep simulating the cached records.
MStatus RigidBodyNode::preEvaluation(const MDGContext& context,
                                     const MEvaluationNode& evaluationNode)
{
    MStatus stat;
    if (evaluationNode.dirtyPlugExists(aBodyBodies, &stat) && (stat == MS::kSuccess))
    {
        mBodiesDirty.store(true, std::memory_order_relaxed);
    }
    if (evaluationNode.dirtyPlugExists(aJoints, &stat) && (stat == MS::kSuccess))
    {
        mJointsDirty.store(true, std::memory_order_relaxed);
    }
    return MPxLocatorNode::preEvaluation(context, evaluationNode);
}

// Whether the whole cached input set must be read again (see the header).
// An element dirt is only structural when its connection changed — the
// per-frame anchors never touch these arrays.  The body and joint records are
// data-block inputs and are handled separately (mBodiesDirty / mJointsDirty).
bool RigidBodyNode::inputsStale(const MObject& node) const
{
    if (!mInputsValid || !mWorld)
    {
        return true;
    }
    for (size_t slot = 0; slot < mBodySlotDirty.size(); ++slot)
    {
        if (!mBodySlotDirty[slot])
        {
            continue;
        }
        if (slot >= mWorld->bodyShapes.nodes.size() ||
            bodyShapeNode(node, static_cast<unsigned int>(slot)) !=
                mWorld->bodyShapes.nodes[slot].object())
        {
            return true; // re-bound / deleted body
        }
    }
    // A jointShapes[] element dirt means the joint NODE (or its body links)
    // changed: re-read the joint records.
    return std::any_of(mJointSlotDirty.cbegin(), mJointSlotDirty.cend(),
                       [](bool dirty) { return dirty; });
}

// ===========================================================================
// compute()
// ===========================================================================
MStatus RigidBodyNode::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    if (!isOutputPlug(plug))
    {
        return MS::kUnknownParameter;
    }

    // Pull the IDENTITY message arrays through the data block.  Their VALUES
    // are not read (they name the shape/joint nodes), but the pull cleans
    // their dirt paths (e.g. shape.solver -> bodyShapes[i] / joint.solver ->
    // jointShapes[i]): Maya stops propagating at a plug that is still dirty,
    // so without it only the FIRST edit at a paused frame would reach the
    // solver.
    dataBlock.inputArrayValue(aBodyShapes);
    dataBlock.inputArrayValue(aJointShapes);

    // Frame transition: refresh whichever input group went stale, then
    // (re)build or advance the world.  The new world atomically replaces the
    // old one.  Bodies and joints are both real inputs, read from the data
    // block when their arrays arrive dirty (or when the wiring changed).
    const bool staleInputs = inputsStale(thisMObject());
    const bool bodiesDirty = mBodiesDirty.exchange(false, std::memory_order_relaxed) || staleInputs;
    const bool jointsDirty = mJointsDirty.exchange(false, std::memory_order_relaxed) || staleInputs;
    if (bodiesDirty)
    {
        // BodyShapes is rebuilt from scratch: it is an append-only index.
        mInputs.bodyShapes = RigidBodyNode::BodyShapes();
        mInputs.bodies = readBodyData(thisMObject(), dataBlock, mInputs.bodyShapes);
    }
    if (jointsDirty)
    {
        mInputs.joints = readJointData(thisMObject(), dataBlock, mInputs.bodyShapes);
    }
    if (bodiesDirty || jointsDirty)
    {
        mInputs.gravity = readGravity(dataBlock);
        mInputsValid = true;
        std::fill(mBodySlotDirty.begin(), mBodySlotDirty.end(), false);
        std::fill(mJointSlotDirty.begin(), mJointSlotDirty.end(), false);
    }
    if (outputAttribute(plug) == aOutJointDisplay)
    {
        return writeJointDisplay(mInputs, dataBlock);
    }
    const MTime now = dataBlock.inputValue(aTime).asTime();
    // The anchors are a real input, so read them every evaluation: the data
    // block serves the value Maya pushed once for this frame, and frame()
    // early-outs on a repeat pull where nothing moved.
    mWorld = frame(std::move(mWorld), mInputs, thisMObject(), now,
                   readAnchorWorlds(dataBlock, mInputs.bodies.size()));

    MDataHandle step = dataBlock.outputValue(aOutStep);
    step.setDouble(now.as(MTime::kSeconds));
    step.setClean();
    return writeOutputs(mWorld, groupWorldMatrix(thisMObject()), dataBlock);
}
