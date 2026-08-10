/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_cmd.cpp
 *
 * Native C++ implementation of the ``pmxRigidBody`` command (create mode — the
 * default).
 *
 * ``-create`` is the single body-modification path (the PMX import loops it;
 * there is no Python wiring anymore).  SIMULATION IS DISABLED: create writes
 * the body DATA and binds FOLLOW_BONE bodies to their related joint through
 * the kinematic-anchor input; dynamic bodies are data-only (no write-back,
 * no stepping).
 *
 * The command's interface is minimal (see rigid_body_cmd.hpp); all the
 * implementation helpers live in the anonymous namespace below so the header
 * stays a pure interface.
 */

#include "rigid_body_cmd.hpp"

#include <maya/MArgList.h>
#include <maya/MArgParser.h>
#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MItDag.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MSyntax.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>

#include "maya_utils.hpp"
#include "nodes/physics_node.h"
#include "physics_math.hpp"

#include <cstdlib>
#include <cstring>
#include <string>

using mmd::core::Double3;
using mmd::core::Simulation;
using mmd::core::physics_math::rad2deg;

namespace
{
// ---------------------------------------------------------------------------
// Flag short names (single/compound, Maya API style).
// ---------------------------------------------------------------------------
constexpr const char* kIndexFlag = "i";
constexpr const char* kNameFlag = "n";
constexpr const char* kNameUniversalFlag = "nu";
constexpr const char* kBoneFlag = "b";
constexpr const char* kShapeFlag = "sh";
constexpr const char* kSizeFlag = "sz";
constexpr const char* kPositionFlag = "p";
constexpr const char* kRotationFlag = "rot";
constexpr const char* kMassFlag = "m";
constexpr const char* kLinearDampingFlag = "ld";
constexpr const char* kAngularDampingFlag = "ad";
constexpr const char* kFrictionFlag = "f";
constexpr const char* kRestitutionFlag = "re";
constexpr const char* kGroupFlag = "g";
constexpr const char* kMaskFlag = "msk"; // short names are limited to 3 chars
constexpr const char* kPhysicsModeFlag = "pm";

// PMX attenuation coefficients are 0..1 — clamp so the node's Bullet
// damping/friction match the import path exactly.
constexpr double clamp01(double v)
{
    if (v < 0.0)
        return 0.0;
    if (v > 1.0)
        return 1.0;
    return v;
}

// A DAG node's world (inclusive) matrix.
MMatrix worldMatrix(const MDagPath& path)
{
    return path.inclusiveMatrix();
}

// A joint's stored PMX bone index (pmxBoneIndex), or -1.
int jointPmxBoneIndex(const MDagPath& jointPath)
{
    try
    {
        MFnDependencyNode fn(jointPath.node());
        MStatus stat;
        MPlug plug = fn.findPlug("pmxBoneIndex", true, &stat);
        if (!plug.isNull())
            return plug.asInt();
    }
    // No joint / no index attribute: reported through the -1 return.
    // NOLINTNEXTLINE(bugprone-empty-catch)
    catch (...)
    {
    }
    return -1;
}

// Resolve the -bone argument to a joint dag path (or leave it empty).
MDagPath resolveBone(const MString& bone, const MDagPath& groupPath)
{
    MDagPath out;
    if (bone.length() == 0)
        return out;

    // Numeric string ⇒ PMX bone index: scan the model root's joints.
    const char* boneStr =
        bone.asChar(); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const unsigned int boneLen = static_cast<unsigned int>(std::strlen(boneStr));
    bool numeric = true;
    for (unsigned int i = 0; i < boneLen; ++i)
    {
        if (boneStr[i] < '0' || boneStr[i] > '9')
        {
            numeric = false;
            break;
        }
    }
    if (numeric)
    {
        // The string was verified to contain only digits above, so the
        // conversion cannot fail; strtol reports the parse end as a guard.
        char* end = nullptr;
        const int idx = static_cast<int>(std::strtol(boneStr, &end, 10));
        if (end == boneStr)
            return out;
        MDagPath rootPath = groupPath;
        rootPath.pop(); // |modelRoot
        MItDag it;
        it.reset(rootPath, MItDag::kDepthFirst, MFn::kJoint);
        for (; !it.isDone(); it.next())
        {
            MDagPath jp;
            it.getPath(jp);
            if (jointPmxBoneIndex(jp) == idx)
                return jp;
        }
        return out;
    }

    // Otherwise: a Maya joint name / path.
    MSelectionList sel;
    if (sel.add(bone) == MS::kSuccess && sel.length() > 0)
    {
        MDagPath p;
        if (sel.getDagPath(0, p) == MS::kSuccess)
            return p;
    }
    return out;
}

// Resolve *target* to an pmxPhysicsNode MObject (direct node or model root).
bool resolveSolver(const MString& target, MObject& outNode)
{
    try
    {
        MSelectionList sel;
        if (sel.add(target) != MS::kSuccess || sel.length() == 0)
            return false;
        MObject obj;
        if (sel.getDependNode(0, obj) != MS::kSuccess)
            return false;
        if (!obj.hasFn(MFn::kDependencyNode))
            return false;

        MFnDependencyNode fn(obj);
        if (fn.typeName() == PhysicsNode::kNodeName)
        {
            outNode = obj;
            return true;
        }
        // Model root: resolve the pmxPhysicsNode string attribute.
        MStatus stat;
        MPlug p = fn.findPlug("pmxPhysicsNode", true, &stat);
        if (!p.isNull())
        {
            const MString solverName = p.asString();
            if (solverName.length() > 0)
            {
                MSelectionList sel2;
                if (sel2.add(solverName) == MS::kSuccess && sel2.length() > 0)
                {
                    MObject obj2;
                    if (sel2.getDependNode(0, obj2) == MS::kSuccess &&
                        obj2.hasFn(MFn::kDependencyNode))
                    {
                        MFnDependencyNode fn2(obj2);
                        if (fn2.typeName() == PhysicsNode::kNodeName)
                        {
                            outNode = obj2;
                            return true;
                        }
                    }
                }
            }
        }
    }
    // Resolution failure is reported through the bool return.
    // NOLINTNEXTLINE(bugprone-empty-catch)
    catch (...)
    {
    }
    return false;
}

// ===========================================================================
// Create mode
// ===========================================================================

MStatus doCreate(const MArgParser& parser, const MObject& solverNode, int& outIndex)
{
    // ── Parse flags (safe defaults, mirroring the former Python command) ──
    int index = -1;
    if (parser.isFlagSet(kIndexFlag))
        index = parser.flagArgumentInt(kIndexFlag, 0);

    MString nameLocal;
    MString nameUniversal;
    MString bone;
    MString shape("sphere");
    MString physicsMode("physics");
    if (parser.isFlagSet(kNameFlag))
        nameLocal = parser.flagArgumentString(kNameFlag, 0);
    if (parser.isFlagSet(kNameUniversalFlag))
        nameUniversal = parser.flagArgumentString(kNameUniversalFlag, 0);
    if (parser.isFlagSet(kBoneFlag))
        bone = parser.flagArgumentString(kBoneFlag, 0);
    if (parser.isFlagSet(kShapeFlag))
        shape = parser.flagArgumentString(kShapeFlag, 0);
    if (parser.isFlagSet(kPhysicsModeFlag))
        physicsMode = parser.flagArgumentString(kPhysicsModeFlag, 0);

    Double3 size(0.5, 0.5, 0.5);
    Double3 pos;
    Double3 rot;
    if (parser.isFlagSet(kSizeFlag))
    {
        for (int i = 0; i < 3; ++i)
            size[i] = parser.flagArgumentDouble(kSizeFlag, i);
    }
    if (parser.isFlagSet(kPositionFlag))
    {
        for (int i = 0; i < 3; ++i)
            pos[i] = parser.flagArgumentDouble(kPositionFlag, i);
    }
    if (parser.isFlagSet(kRotationFlag))
    {
        for (int i = 0; i < 3; ++i)
            rot[i] = parser.flagArgumentDouble(kRotationFlag, i);
    }

    double mass = 1.0;
    double linearDamping = 0.0;
    double angularDamping = 0.0;
    double friction = 0.5;
    double restitution = 0.0;
    if (parser.isFlagSet(kMassFlag))
        mass = parser.flagArgumentDouble(kMassFlag, 0);
    if (parser.isFlagSet(kLinearDampingFlag))
        linearDamping = parser.flagArgumentDouble(kLinearDampingFlag, 0);
    if (parser.isFlagSet(kAngularDampingFlag))
        angularDamping = parser.flagArgumentDouble(kAngularDampingFlag, 0);
    if (parser.isFlagSet(kFrictionFlag))
        friction = parser.flagArgumentDouble(kFrictionFlag, 0);
    if (parser.isFlagSet(kRestitutionFlag))
        restitution = parser.flagArgumentDouble(kRestitutionFlag, 0);

    int group = 0;
    int mask = 0xFFFF; // default: collide with every group
    if (parser.isFlagSet(kGroupFlag))
        group = parser.flagArgumentInt(kGroupFlag, 0);
    if (parser.isFlagSet(kMaskFlag))
        mask = parser.flagArgumentInt(kMaskFlag, 0) & 0xFFFF;
    // PMX collision groups are 0..15 (the bodyGroupId attribute is a 16-field
    // enum) — clamp so an out-of-range flag cannot write a broken enum value.
    if (group < 0)
        group = 0;
    else if (group > 15)
        group = 15;

    // PMX attenuation coefficients are 0..1 — clamp so the node's Bullet
    // damping/friction match the import path exactly.
    linearDamping = clamp01(linearDamping);
    angularDamping = clamp01(angularDamping);
    friction = clamp01(friction);
    restitution = clamp01(restitution);

    // ── Enumerated values ──
    const MString sh = shape.toLowerCase();
    PhysicsNode::ColliderType colliderType = PhysicsNode::kColliderSphere;
    if (sh == "box")
    {
        colliderType = PhysicsNode::kColliderBox;
    }
    else if (sh == "capsule")
    {
        colliderType = PhysicsNode::kColliderCapsule;
    }
    else if (sh != "sphere")
    {
        MGlobal::displayError("Unknown shape '" + shape + "' — expected sphere, box or capsule");
        return MS::kFailure;
    }

    const MString pm = physicsMode.toLowerCase();
    // The bodyPhysicsMode attribute stores mmd::core::Simulation::PhysicsMode
    // values directly (the node casts the attribute value back to the engine
    // enum) — there is deliberately NO node-side enum.
    Simulation::PhysicsMode physicsModeEnum = Simulation::PhysicsMode::ePhysics;
    if (pm == "followbone")
    {
        physicsModeEnum = Simulation::PhysicsMode::eFollowBone;
    }
    else if (pm == "physicsbone")
    {
        physicsModeEnum = Simulation::PhysicsMode::ePhysicsBone;
    }
    else if (pm != "physics")
    {
        MGlobal::displayError("Unknown physicsMode '" + physicsMode +
                              "' — expected followBone, physics or physicsBone");
        return MS::kFailure;
    }
    const bool kinematic = (physicsModeEnum == Simulation::PhysicsMode::eFollowBone);

    // ── Solver / group / index ──
    MFnDependencyNode fn(solverNode);
    MStatus plugStat;
    MPlug bodiesPlug = fn.findPlug(PhysicsNode::aBodies, true, &plugStat);
    if (bodiesPlug.isNull())
    {
        MGlobal::displayError("pmxRigidBody: node has no 'bodies' array");
        return MS::kFailure;
    }
    const int count = static_cast<int>(bodiesPlug.numElements());
    if (index >= 0 && index != count)
    {
        MString msg = MString("Body index ") + MString(std::to_string(index).c_str()) +
                      MString(" is not the next free index (") +
                      MString(std::to_string(count).c_str()) +
                      MString(") — append at the end or use edit mode to overwrite");
        MGlobal::displayError(msg);
        return MS::kFailure;
    }
    const int n = count;

    MDagPath nodePath;
    if (MDagPath::getAPathTo(solverNode, nodePath) != MS::kSuccess)
    {
        MGlobal::displayError("pmxRigidBody: could not resolve solver dag path");
        return MS::kFailure;
    }
    MDagPath groupPath = nodePath;
    groupPath.pop(); // |group|shape ⇒ |group
    const MMatrix groupWorld = groupPath.inclusiveMatrix();

    // Related joint (anchor / write-back target).
    const MDagPath jointPath = resolveBone(bone, groupPath);
    MPlug jointWorldPlug;
    if (jointPath.isValid())
    {
        MFnDependencyNode jointFn(jointPath.node());
        MStatus jointPlugStat;
        jointWorldPlug =
            jointFn.findPlug("worldMatrix", true, &jointPlugStat).elementByLogicalIndex(0);
    }

    // ── Rest pose in group space (MMD ⇒ Maya: Z-flip + handedness) ──
    const Double3 worldT(pos.x, pos.y, -pos.z);
    const Double3 worldR(-rad2deg(rot.x), -rad2deg(rot.y), rad2deg(rot.z));
    const MMatrix local = mmd::maya::matrixFromTR(worldT, worldR) * groupWorld.inverse();
    MTransformationMatrix mt(local);
    const MVector lt = mt.getTranslation(MSpace::kTransform);
    const MEulerRotation le = mt.eulerRotation();
    const Double3 localT(lt.x, lt.y, lt.z);
    const Double3 localR(rad2deg(le.x), rad2deg(le.y), rad2deg(le.z));

    // ── Write the body data (simple create) ──
    MPlug elem = bodiesPlug.elementByLogicalIndex(n);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aBodyRestTranslate), localT);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aBodyRestRotate), localR);
    elem.child(PhysicsNode::aBodyMass).setDouble(mass);
    elem.child(PhysicsNode::aBodyLinearDamping).setDouble(linearDamping);
    elem.child(PhysicsNode::aBodyAngularDamping).setDouble(angularDamping);
    elem.child(PhysicsNode::aBodyFriction).setDouble(friction);
    elem.child(PhysicsNode::aBodyRestitution).setDouble(restitution);
    elem.child(PhysicsNode::aBodyColliderType).setShort(colliderType);
    // PMX shape_size VERBATIM (full size — the node derives the engine
    // radius/extents/length by collider type via mmd::core::applyShapeSize).
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aBodyShapeSize), size);
    elem.child(PhysicsNode::aBodyGroupId).setShort(static_cast<short>(group));
    for (int g = 0; g < 16; ++g)
        elem.child(PhysicsNode::aBodyMaskGroup.at(g)).setBool(((mask >> g) & 1) != 0);
    elem.child(PhysicsNode::aBodyPhysicsMode).setShort(static_cast<short>(physicsModeEnum));
    // Wiring fields: the parent body / reset anchor are resolved later (the
    // parent body may not exist yet); the write-back K offset is baked below.
    elem.child(PhysicsNode::aBodyResetAnchorIndex).setInt(-1);
    elem.child(PhysicsNode::aBodyParentBodyIndex).setInt(-1);
    elem.child(PhysicsNode::aBodyNameLocal).setString(nameLocal);
    elem.child(PhysicsNode::aBodyNameUniversal).setString(nameUniversal);
    elem.child(PhysicsNode::aBodyEnabled).setBool(true);

    // ── Bone binding ──
    // FOLLOW_BONE bodies are bound to their related joint through the
    // kinematic-anchor INPUT (joint.worldMatrix -> anchorWorldMatrix + baked
    // body<->bone offset) — this is what makes the collider "live on the
    // correct bone".  Dynamic bodies are wired for write-back by Python AFTER
    // the whole model exists (mmd/maya/pmx/rigid_body_builder.py — the output
    // connections must come last so the first evaluation sees every joint).
    // Bodies display from their rest pose — the draw override falls back to
    // reading the plugs when the world is never built.
    if (kinematic)
    {
        // Kinematic-order index of the new anchor.
        int k = 0;
        for (int i = 0; i < n; ++i)
        {
            if (bodiesPlug.elementByLogicalIndex(i)
                    .child(PhysicsNode::aBodyPhysicsMode)
                    .asShort() == static_cast<short>(Simulation::PhysicsMode::eFollowBone))
                ++k;
        }
        // The anchor world is the joint's world matrix (the node converts it
        // to group space and applies the body<->joint offset as K^-1 — both
        // derived internally from groupWorldMatrix and bodyWriteBackOffset,
        // so there is no anchorOffset / groupInverseWorldMatrix input to
        // populate).
        if (jointPath.isValid())
        {
            mmd::maya::connectOrReplace(
                jointWorldPlug, fn.findPlug(PhysicsNode::aAnchorWorldMatrix, true, &plugStat)
                                    .elementByLogicalIndex(k));
        }
        else
        {
            // No related joint: a static collider pinned at its rest pose.
            // Pin the body's DIRECT world rest pose (world * groupInverse
            // gives the group-space rest in the node) — not the round-tripped
            // group-space decomposition, which drops group scale.
            const MMatrix bodyWorld = mmd::maya::matrixFromTR(worldT, worldR);
            mmd::maya::setPlugMatrixValue(
                fn.findPlug(PhysicsNode::aAnchorWorldMatrix, true, &plugStat)
                    .elementByLogicalIndex(k),
                bodyWorld);
        }
    }

    // ── Write-back inputs (dense) ──
    // bodyWriteBackOffset[n] = K = jointRestWorld * bodyRestWorld^-1 for EVERY
    // body (identity when there is no related joint) — the array is dense by
    // construction (every -create sets its element).  The node's write-back
    // derives the parent joint's world from K[parentBodyIndex]
    // (M_parent = parentJointRestWorld * parentBodyRestWorld^-1 is the SAME
    // constant as the parent body's K, for kinematic AND dynamic parents), so
    // no separate parent-offset array is needed.  bodyParentInverseMatrix
    // starts as identity; Python connects the DG fallback later, ONLY for
    // bodies whose parent bone has no rigid body (that parent is never
    // node-driven, so it cannot feed back).
    {
        MMatrix k;
        k.setToIdentity();
        MMatrix identity;
        identity.setToIdentity();
        if (jointPath.isValid())
        {
            // K = jointRestWorld * bodyRestWorld^-1 with the body's DIRECT
            // world rest pose (MMD Z-flip + handedness), exactly like the old
            // Python bake.  Do NOT round-trip through the group-space
            // decomposition (matrixFromTR(localT, localR) * groupWorld): when
            // the physics group carries scale, MTransformationMatrix drops the
            // scale during euler decomposition, so the round-tripped "world"
            // is scaled wrongly and every write-back-driven bone lands off its
            // rest pose (the collider guides stay correct — they render the
            // scale back through the DAG — which is the exact breakage seen).
            const MMatrix bodyWorld = mmd::maya::matrixFromTR(worldT, worldR);
            k = worldMatrix(jointPath) * bodyWorld.inverse();
        }
        mmd::maya::setPlugMatrixValue(
            fn.findPlug(PhysicsNode::aBodyWriteBackOffset, true, &plugStat)
                .elementByLogicalIndex(n),
            k);
        mmd::maya::setPlugMatrixValue(
            fn.findPlug(PhysicsNode::aBodyParentInverseMatrix, true, &plugStat)
                .elementByLogicalIndex(n),
            identity);
    }

    outIndex = n;
    return MS::kSuccess;
}

} // namespace

// ===========================================================================
// Registration
// ===========================================================================

void* RigidBodyCmd::creator()
{
    return new RigidBodyCmd();
}

MSyntax RigidBodyCmd::syntaxCreator()
{
    MSyntax syntax;
    // First positional argument: the solver node (or a model root).
    syntax.addArg(MSyntax::kString);

    syntax.addFlag(kIndexFlag, "index", MSyntax::kLong);
    syntax.addFlag(kNameFlag, "name", MSyntax::kString);
    syntax.addFlag(kNameUniversalFlag, "nameUniversal", MSyntax::kString);
    syntax.addFlag(kBoneFlag, "bone", MSyntax::kString);
    syntax.addFlag(kShapeFlag, "shape", MSyntax::kString);
    syntax.addFlag(kSizeFlag, "size", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    syntax.addFlag(kPositionFlag, "position", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    syntax.addFlag(kRotationFlag, "rotation", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    syntax.addFlag(kMassFlag, "mass", MSyntax::kDouble);
    syntax.addFlag(kLinearDampingFlag, "linearDamping", MSyntax::kDouble);
    syntax.addFlag(kAngularDampingFlag, "angularDamping", MSyntax::kDouble);
    syntax.addFlag(kFrictionFlag, "friction", MSyntax::kDouble);
    syntax.addFlag(kRestitutionFlag, "restitution", MSyntax::kDouble);
    syntax.addFlag(kGroupFlag, "group", MSyntax::kLong);
    // Collision mask as a 16-bit "collides with" bitmask (bit i = collides
    // with group i) — the PMX non_collision_group field stored VERBATIM (MMD
    // feeds it to Bullet directly; no inversion).  Written into the
    // bodyMaskGroup0..15 boolean children.
    syntax.addFlag(kMaskFlag, "mask", MSyntax::kLong);
    syntax.addFlag(kPhysicsModeFlag, "physicsMode", MSyntax::kString);

    syntax.enableEdit(true);
    syntax.enableQuery(true);
    return syntax;
}

// ===========================================================================
// doIt
// ===========================================================================

MStatus RigidBodyCmd::doIt(const MArgList& args)
{
    MStatus stat;
    MArgParser parser(syntaxCreator(), args, &stat);
    if (!stat)
    {
        displayError("pmxRigidBody: could not parse arguments");
        return stat;
    }

    const MString target = parser.commandArgumentString(0, &stat);
    if (!stat || target.length() == 0)
    {
        displayError("pmxRigidBody: missing solver / modelRoot argument");
        return MS::kFailure;
    }

    if (parser.isQuery())
    {
        displayError("pmxRigidBody query mode is not implemented yet");
        return MS::kFailure;
    }
    if (parser.isEdit())
    {
        displayError("pmxRigidBody edit mode is not implemented yet");
        return MS::kFailure;
    }

    MObject solverNode;
    if (!resolveSolver(target, solverNode))
    {
        displayError("'" + target + "' is not an pmxPhysicsNode or a PMX model root");
        return MS::kFailure;
    }

    int newIndex = -1;
    stat = doCreate(parser, solverNode, newIndex);
    if (stat)
        setResult(newIndex);
    return stat;
}
