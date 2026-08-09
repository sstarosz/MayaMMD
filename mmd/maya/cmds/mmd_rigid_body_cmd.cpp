/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_rigid_body_cmd.cpp
 *
 * Native C++ implementation of the ``mmdRigidBody`` command (create mode —
 * the default).
 *
 * ``-create`` is the single body-modification path (the PMX import loops it;
 * there is no Python wiring anymore).  SIMULATION IS DISABLED: create writes
 * the body DATA and binds FOLLOW_BONE bodies to their related joint through
 * the kinematic-anchor input; dynamic bodies are data-only (no write-back,
 * no stepping).
 */

#include "mmd_rigid_body_cmd.h"

#include <maya/MArgList.h>
#include <maya/MArgParser.h>
#include <maya/MDGModifier.h>
#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnNumericData.h>
#include <maya/MItDag.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MSyntax.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>

#include "../nodes/mmd_physics_node.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

inline double degToRad(double d)
{
    return d * kDegToRad;
}
inline double radToDeg(double r)
{
    return r / kDegToRad;
}
inline double clamp01(double v)
{
    return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
}

// Flag short names (single/compound, Maya API style).
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
constexpr const char* kNonCollisionGroupFlag = "ncg";
constexpr const char* kPhysicsModeFlag = "pm";

// MPlug has no setValue(MMatrix) overload — wrap the matrix in an
// MFnMatrixData MObject (the standard pattern for matrix plugs).
void setMatrixValue(MPlug& plug, const MMatrix& m)
{
    MFnMatrixData data;
    MObject obj = data.create(m);
    plug.setValue(obj);
}

// MPlug has no setValue3Double — wrap 3 doubles in an MFnNumericData object.
void setDouble3(MPlug& plug, const Double3& v)
{
    MFnNumericData data;
    MObject obj = data.create(MFnNumericData::k3Double);
    data.setData3Double(v.x, v.y, v.z);
    plug.setValue(obj);
}

} // namespace

// ===========================================================================
// Registration
// ===========================================================================

void* MmdRigidBodyCmd::creator()
{
    return new MmdRigidBodyCmd();
}

MSyntax MmdRigidBodyCmd::syntaxCreator()
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
    syntax.addFlag(kNonCollisionGroupFlag, "nonCollisionGroup", MSyntax::kLong);
    syntax.addFlag(kPhysicsModeFlag, "physicsMode", MSyntax::kString);

    syntax.enableEdit(true);
    syntax.enableQuery(true);
    return syntax;
}

// ===========================================================================
// doIt
// ===========================================================================

MStatus MmdRigidBodyCmd::doIt(const MArgList& args)
{
    MStatus stat;
    MArgParser parser(syntaxCreator(), args, &stat);
    if (!stat)
    {
        displayError("mmdRigidBody: could not parse arguments");
        return stat;
    }

    MString target = parser.commandArgumentString(0, &stat);
    if (!stat || target.length() == 0)
    {
        displayError("mmdRigidBody: missing solver / modelRoot argument");
        return MS::kFailure;
    }

    if (parser.isQuery())
    {
        displayError("mmdRigidBody query mode is not implemented yet");
        return MS::kFailure;
    }
    if (parser.isEdit())
    {
        displayError("mmdRigidBody edit mode is not implemented yet");
        return MS::kFailure;
    }

    MObject solverNode;
    if (!resolveSolver(target, solverNode))
    {
        displayError("'" + target + "' is not an mmdPhysicsNode or a PMX model root");
        return MS::kFailure;
    }

    int newIndex = -1;
    stat = doCreate(parser, solverNode, newIndex);
    if (stat)
        setResult(newIndex);
    return stat;
}

// ===========================================================================
// Helpers
// ===========================================================================

bool MmdRigidBodyCmd::resolveSolver(const MString& target, MObject& outNode)
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
        if (fn.typeName() == MMDPhysicsNode::kNodeName)
        {
            outNode = obj;
            return true;
        }
        // Model root: resolve the pmxPhysicsNode string attribute.
        MPlug p = fn.findPlug("pmxPhysicsNode", true);
        if (!p.isNull())
        {
            MString solverName = p.asString();
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
                        if (fn2.typeName() == MMDPhysicsNode::kNodeName)
                        {
                            outNode = obj2;
                            return true;
                        }
                    }
                }
            }
        }
    }
    catch (...)
    {
    }
    return false;
}

MMatrix MmdRigidBodyCmd::matrixFromTR(const Double3& t, const Double3& r)
{
    MTransformationMatrix mt;
    mt.setTranslation(MVector(t.x, t.y, t.z), MSpace::kTransform);
    double rot[3] = {degToRad(r.x), degToRad(r.y), degToRad(r.z)};
    mt.setRotation(rot, MTransformationMatrix::kXYZ);
    return mt.asMatrix();
}

MMatrix MmdRigidBodyCmd::worldMatrix(const MDagPath& path)
{
    return path.inclusiveMatrix();
}

MStatus MmdRigidBodyCmd::connectOrReplace(const MPlug& src, const MPlug& dst)
{
    MDGModifier mod;
    if (dst.isConnected())
    {
        MPlugArray sources;
        dst.connectedTo(sources, true, false);
        for (unsigned int i = 0; i < sources.length(); ++i)
        {
            if (sources[i] != src)
                mod.disconnect(sources[i], dst);
        }
    }
    if (mod.connect(src, dst) != MS::kSuccess)
        return MS::kFailure;
    return mod.doIt();
}

int MmdRigidBodyCmd::jointPmxBoneIndex(const MDagPath& jointPath)
{
    try
    {
        MFnDependencyNode fn(jointPath.node());
        MPlug plug = fn.findPlug("pmxBoneIndex", true);
        if (!plug.isNull())
            return plug.asInt();
    }
    catch (...)
    {
    }
    return -1;
}

MDagPath MmdRigidBodyCmd::resolveBone(const MString& bone, const MDagPath& groupPath)
{
    MDagPath out;
    if (bone.length() == 0)
        return out;

    // Numeric string → PMX bone index: scan the model root's joints.
    const char* boneStr = bone.asChar();
    unsigned int boneLen = static_cast<unsigned int>(std::strlen(boneStr));
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
        int idx = std::atoi(boneStr);
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

// ===========================================================================
// Create mode
// ===========================================================================

MStatus MmdRigidBodyCmd::doCreate(const MArgParser& parser, const MObject& solverNode,
                                  int& outIndex)
{
    // ── Parse flags (safe defaults, mirroring the former Python command) ──
    int index = -1;
    if (parser.isFlagSet(kIndexFlag))
        index = static_cast<int>(parser.flagArgumentInt(kIndexFlag, 0));

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
        for (int i = 0; i < 3; ++i)
            size[i] = parser.flagArgumentDouble(kSizeFlag, i);
    if (parser.isFlagSet(kPositionFlag))
        for (int i = 0; i < 3; ++i)
            pos[i] = parser.flagArgumentDouble(kPositionFlag, i);
    if (parser.isFlagSet(kRotationFlag))
        for (int i = 0; i < 3; ++i)
            rot[i] = parser.flagArgumentDouble(kRotationFlag, i);

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
    int ncg = 0xFFFF;
    if (parser.isFlagSet(kGroupFlag))
        group = static_cast<int>(parser.flagArgumentInt(kGroupFlag, 0));
    if (parser.isFlagSet(kNonCollisionGroupFlag))
        ncg = static_cast<int>(parser.flagArgumentInt(kNonCollisionGroupFlag, 0));

    // PMX attenuation coefficients are 0..1 — clamp so the node's Bullet
    // damping/friction match the import path exactly.
    linearDamping = clamp01(linearDamping);
    angularDamping = clamp01(angularDamping);
    friction = clamp01(friction);
    restitution = clamp01(restitution);

    // ── Enumerated values ──
    MString sh = shape.toLowerCase();
    MMDPhysicsNode::ColliderType colliderType = MMDPhysicsNode::kColliderSphere;
    if (sh == "box")
        colliderType = MMDPhysicsNode::kColliderBox;
    else if (sh == "capsule")
        colliderType = MMDPhysicsNode::kColliderCapsule;
    else if (sh != "sphere")
    {
        displayError("Unknown shape '" + shape + "' — expected sphere, box or capsule");
        return MS::kFailure;
    }

    MString pm = physicsMode.toLowerCase();
    MMDPhysicsNode::BodyPhysicsMode physicsModeEnum = MMDPhysicsNode::kBodyPhysics;
    if (pm == "followbone")
        physicsModeEnum = MMDPhysicsNode::kBodyPhysicsFollowBone;
    else if (pm == "physicsbone")
        physicsModeEnum = MMDPhysicsNode::kBodyPhysicsBone;
    else if (pm != "physics")
    {
        displayError("Unknown physicsMode '" + physicsMode +
                     "' — expected followBone, physics or physicsBone");
        return MS::kFailure;
    }
    bool kinematic = (physicsModeEnum == MMDPhysicsNode::kBodyPhysicsFollowBone);

    // ── Solver / group / index ──
    MFnDependencyNode fn(solverNode);
    MPlug bodiesPlug = fn.findPlug(MMDPhysicsNode::aBodies, true);
    if (bodiesPlug.isNull())
    {
        displayError("mmdRigidBody: node has no 'bodies' array");
        return MS::kFailure;
    }
    int count = bodiesPlug.numElements();
    if (index >= 0 && index != count)
    {
        MString msg = MString("Body index ") + MString(std::to_string(index).c_str()) +
                      MString(" is not the next free index (") +
                      MString(std::to_string(count).c_str()) +
                      MString(") — append at the end or use edit mode to overwrite");
        displayError(msg);
        return MS::kFailure;
    }
    int n = count;

    MDagPath nodePath;
    if (MDagPath::getAPathTo(solverNode, nodePath) != MS::kSuccess)
    {
        displayError("mmdRigidBody: could not resolve solver dag path");
        return MS::kFailure;
    }
    MDagPath groupPath = nodePath;
    groupPath.pop(); // |group|shape → |group
    MMatrix groupWorld = groupPath.inclusiveMatrix();
    MFnDependencyNode groupFn(groupPath.node());
    MPlug groupWorldInversePlug = groupFn.findPlug("worldInverseMatrix").elementByLogicalIndex(0);

    // Related joint (anchor / write-back target).
    MDagPath jointPath = resolveBone(bone, groupPath);
    MPlug jointWorldPlug;
    if (jointPath.isValid())
    {
        MFnDependencyNode jointFn(jointPath.node());
        jointWorldPlug = jointFn.findPlug("worldMatrix").elementByLogicalIndex(0);
    }

    // ── Rest pose in group space (MMD → Maya: Z-flip + handedness) ──
    Double3 worldT(pos.x, pos.y, -pos.z);
    Double3 worldR(-radToDeg(rot.x), -radToDeg(rot.y), radToDeg(rot.z));
    MMatrix local = matrixFromTR(worldT, worldR) * groupWorld.inverse();
    MTransformationMatrix mt(local);
    MVector lt = mt.getTranslation(MSpace::kTransform);
    MEulerRotation le = mt.eulerRotation();
    Double3 localT(lt.x, lt.y, lt.z);
    Double3 localR(radToDeg(le.x), radToDeg(le.y), radToDeg(le.z));

    // ── Write the body data (simple create) ──
    MPlug elem = bodiesPlug.elementByLogicalIndex(n);
    setDouble3(elem.child(MMDPhysicsNode::aBodyRestTranslate), localT);
    setDouble3(elem.child(MMDPhysicsNode::aBodyRestRotate), localR);
    elem.child(MMDPhysicsNode::aBodyMass).setDouble(mass);
    elem.child(MMDPhysicsNode::aBodyLinearDamping).setDouble(linearDamping);
    elem.child(MMDPhysicsNode::aBodyAngularDamping).setDouble(angularDamping);
    elem.child(MMDPhysicsNode::aBodyFriction).setDouble(friction);
    elem.child(MMDPhysicsNode::aBodyRestitution).setDouble(restitution);
    elem.child(MMDPhysicsNode::aBodyColliderType).setShort(colliderType);
    elem.child(MMDPhysicsNode::aBodyRadius).setDouble(size.x);
    setDouble3(elem.child(MMDPhysicsNode::aBodyExtents), size);
    elem.child(MMDPhysicsNode::aBodyLength).setDouble(size.y);
    elem.child(MMDPhysicsNode::aBodyGroupId).setShort(group);
    elem.child(MMDPhysicsNode::aBodyNonCollisionGroup).setInt(ncg & 0xFFFF);
    elem.child(MMDPhysicsNode::aBodyKinematic).setBool(kinematic);
    elem.child(MMDPhysicsNode::aBodyPhysicsMode).setShort(static_cast<short>(physicsModeEnum));
    // Wiring fields stay at defaults — simulation disabled (no write-back).
    elem.child(MMDPhysicsNode::aBodyResetAnchorIndex).setInt(-1);
    elem.child(MMDPhysicsNode::aBodyParentBodyIndex).setInt(-1);
    elem.child(MMDPhysicsNode::aBodyNameLocal).setString(nameLocal);
    elem.child(MMDPhysicsNode::aBodyNameUniversal).setString(nameUniversal);
    elem.child(MMDPhysicsNode::aBodyEnabled).setBool(true);

    // ── Bone binding (simulation DISABLED) ──
    // FOLLOW_BONE bodies are bound to their related joint through the
    // kinematic-anchor INPUT (joint.worldMatrix -> anchorWorldMatrix + baked
    // body<->bone offset) — this is what makes the collider "live on the
    // correct bone".  Dynamic bodies are DATA-ONLY for now: no output wiring
    // and no solver stepping (that is what drove joints and exploded on
    // import).  Bodies display from their rest pose — the draw override
    // falls back to reading the plugs when the world is never built.
    if (kinematic)
    {
        // Kinematic-order index of the new anchor.
        int k = 0;
        for (int i = 0; i < n; ++i)
        {
            if (bodiesPlug.elementByLogicalIndex(i)
                    .child(MMDPhysicsNode::aBodyPhysicsMode)
                    .asShort() == static_cast<short>(MMDPhysicsNode::kBodyPhysicsFollowBone))
                ++k;
        }
        if (jointPath.isValid())
        {
            connectOrReplace(
                jointWorldPlug,
                fn.findPlug(MMDPhysicsNode::aAnchorWorldMatrix).elementByLogicalIndex(k));
            connectOrReplace(
                groupWorldInversePlug,
                fn.findPlug(MMDPhysicsNode::aAnchorParentInverseMatrix).elementByLogicalIndex(k));
            MMatrix bodyWorld = matrixFromTR(localT, localR) * groupWorld;
            MMatrix offset = bodyWorld * worldMatrix(jointPath).inverse();
            setMatrixValue(fn.findPlug(MMDPhysicsNode::aAnchorOffset).elementByLogicalIndex(k),
                           offset);
        }
        else
        {
            // No related joint: a static collider pinned at its rest pose.
            MMatrix bodyWorld = matrixFromTR(localT, localR) * groupWorld;
            MMatrix identity;
            identity.setToIdentity();
            setMatrixValue(fn.findPlug(MMDPhysicsNode::aAnchorWorldMatrix).elementByLogicalIndex(k),
                           bodyWorld);
            setMatrixValue(
                fn.findPlug(MMDPhysicsNode::aAnchorParentInverseMatrix).elementByLogicalIndex(k),
                groupWorld.inverse());
            setMatrixValue(fn.findPlug(MMDPhysicsNode::aAnchorOffset).elementByLogicalIndex(k),
                           identity);
        }
    }

    outIndex = n;
    return MS::kSuccess;
}
