/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_rigid_body_cmd.cpp
 *
 * Native C++ implementation of the ``mmdRigidBody`` command (create +
 * finalize modes).
 *
 * ``-create`` is the single body-modification path (the PMX import loops it;
 * there is no Python wiring anymore).  SIMULATION IS DISABLED: create writes
 * the body DATA and binds FOLLOW_BONE bodies to their related joint through
 * the kinematic-anchor input; dynamic bodies are data-only (no write-back,
 * no stepping).  ``-finalize`` (dormant) resolves every body's cross-body
 * wiring from the complete scene — kept for when the simulation is re-enabled.
 */

#include "mmd_rigid_body_cmd.h"

#include <maya/MArgList.h>
#include <maya/MArgParser.h>
#include <maya/MDGModifier.h>
#include <maya/MDagPath.h>
#include <maya/MDataHandle.h>
#include <maya/MEulerRotation.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnNumericData.h>
#include <maya/MGlobal.h>
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
#include <map>
#include <set>

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

// Scan an MPlugArray of connection endpoints and return the first JOINT, or
// follow through a Maya auto-inserted unitConversion (a raw double3 in
// radians connected to a joint's degree-based rotate gets one inserted) to
// reach the joint.
MDagPath jointFromDestinations(const MPlugArray& plugs)
{
    for (unsigned int d = 0; d < plugs.length(); ++d)
    {
        MObject node = plugs[d].node();
        if (node.hasFn(MFn::kJoint))
        {
            MDagPath p;
            if (MDagPath::getAPathTo(node, p) == MS::kSuccess)
                return p;
        }
    }
    for (unsigned int d = 0; d < plugs.length(); ++d)
    {
        MObject node = plugs[d].node();
        if (node.hasFn(MFn::kDependencyNode))
        {
            MFnDependencyNode fn(node);
            if (fn.typeName() == "unitConversion")
            {
                MPlug out = fn.findPlug("output");
                MPlugArray next;
                out.connectedTo(next, false, true);
                for (unsigned int e = 0; e < next.length(); ++e)
                {
                    MObject jnode = next[e].node();
                    if (jnode.hasFn(MFn::kJoint))
                    {
                        MDagPath p;
                        if (MDagPath::getAPathTo(jnode, p) == MS::kSuccess)
                            return p;
                    }
                }
            }
        }
    }
    return MDagPath();
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
void setFloat3(MPlug& plug, double x, double y, double z)
{
    MFnNumericData data;
    MObject obj = data.create(MFnNumericData::k3Double);
    data.setData3Double(x, y, z);
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
    // -finalize: resolve every body's cross-body wiring from the scene
    // (write-back resolution — simulation is currently disabled at import).
    syntax.addFlag("fin", "finalize", MSyntax::kNoArg);

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

    if (parser.isFlagSet("fin"))
    {
        MStatus s = doFinalize(solverNode);
        if (s)
            setResult(0);
        return s;
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

void MmdRigidBodyCmd::readFloat3(const MPlug& plug, double out[3])
{
    out[0] = out[1] = out[2] = 0.0;
    try
    {
        MDataHandle h = plug.asMDataHandle();
        MVector v = h.asVector();
        out[0] = v.x;
        out[1] = v.y;
        out[2] = v.z;
    }
    catch (...)
    {
    }
}

MMatrix MmdRigidBodyCmd::matrixFromTR(const double t[3], const double r[3])
{
    MTransformationMatrix mt;
    mt.setTranslation(MVector(t[0], t[1], t[2]), MSpace::kTransform);
    double rot[3] = {degToRad(r[0]), degToRad(r[1]), degToRad(r[2])};
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

int MmdRigidBodyCmd::jointPmxParentBoneIndex(const MDagPath& jointPath)
{
    try
    {
        MFnDependencyNode fn(jointPath.node());
        MPlug plug = fn.findPlug("pmxParentBoneIndex", true);
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

MDagPath MmdRigidBodyCmd::bodyJointPath(MFnDependencyNode& fn, MPlug& bodiesPlug, int i)
{
    MDagPath out;
    try
    {
        short mode =
            bodiesPlug.elementByLogicalIndex(i).child(MMDPhysicsNode::aBodyPhysicsMode).asShort();
        if (mode != 0)
        {
            // Dynamic: outRotate[i] → (unitConversion) → joint.rotate.
            MPlug outRotate = fn.findPlug(MMDPhysicsNode::aOutRotate)
                                  .elementByLogicalIndex(i)
                                  .child(MMDPhysicsNode::aOutRotateValue);
            MPlugArray dests;
            outRotate.connectedTo(dests, false, true);
            return jointFromDestinations(dests);
        }
        else
        {
            // Kinematic: anchorWorldMatrix[k] ← joint.worldMatrix.
            int k = 0;
            for (int j = 0; j < i; ++j)
            {
                if (bodiesPlug.elementByLogicalIndex(j)
                        .child(MMDPhysicsNode::aBodyPhysicsMode)
                        .asShort() == 0)
                    ++k;
            }
            MPlug anchor = fn.findPlug(MMDPhysicsNode::aAnchorWorldMatrix).elementByLogicalIndex(k);
            MPlugArray srcs;
            anchor.connectedTo(srcs, true, false);
            for (unsigned int d = 0; d < srcs.length(); ++d)
            {
                MObject node = srcs[d].node();
                if (node.hasFn(MFn::kJoint))
                {
                    MDagPath p;
                    if (MDagPath::getAPathTo(node, p) == MS::kSuccess)
                        return p;
                }
            }
        }
    }
    catch (...)
    {
    }
    return out;
}

void MmdRigidBodyCmd::stepSolver(const MObject& solverNode)
{
    try
    {
        MFnDependencyNode fn(solverNode);
        MString name = fn.name();
        MGlobal::executeCommand("dgdirty \"" + name + "\"");
        MGlobal::executeCommand("dgeval \"" + name + ".outTranslate\"");
    }
    catch (...)
    {
    }
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

    double size[3] = {0.5, 0.5, 0.5};
    double pos[3] = {0.0, 0.0, 0.0};
    double rot[3] = {0.0, 0.0, 0.0};
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
    int colliderType = 2; // sphere
    if (sh == "box")
        colliderType = 1;
    else if (sh == "capsule")
        colliderType = 3;
    else if (sh != "sphere")
    {
        displayError("Unknown shape '" + shape + "' — expected sphere, box or capsule");
        return MS::kFailure;
    }

    MString pm = physicsMode.toLowerCase();
    int physicsModeInt = 1; // physics
    if (pm == "followbone")
        physicsModeInt = 0;
    else if (pm == "physicsbone")
        physicsModeInt = 2;
    else if (pm != "physics")
    {
        displayError("Unknown physicsMode '" + physicsMode +
                     "' — expected followBone, physics or physicsBone");
        return MS::kFailure;
    }
    bool kinematic = (physicsModeInt == 0);

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
    double worldT[3] = {pos[0], pos[1], -pos[2]};
    double worldR[3] = {-radToDeg(rot[0]), -radToDeg(rot[1]), radToDeg(rot[2])};
    MMatrix local = matrixFromTR(worldT, worldR) * groupWorld.inverse();
    MTransformationMatrix mt(local);
    MVector lt = mt.getTranslation(MSpace::kTransform);
    MEulerRotation le = mt.eulerRotation();
    double localT[3] = {lt.x, lt.y, lt.z};
    double localR[3] = {radToDeg(le.x), radToDeg(le.y), radToDeg(le.z)};

    // ── Write the body data (simple create) ──
    MPlug elem = bodiesPlug.elementByLogicalIndex(n);
    setFloat3(elem.child(MMDPhysicsNode::aBodyRestTranslate), localT[0], localT[1], localT[2]);
    setFloat3(elem.child(MMDPhysicsNode::aBodyRestRotate), localR[0], localR[1], localR[2]);
    elem.child(MMDPhysicsNode::aBodyMass).setDouble(mass);
    elem.child(MMDPhysicsNode::aBodyLinearDamping).setDouble(linearDamping);
    elem.child(MMDPhysicsNode::aBodyAngularDamping).setDouble(angularDamping);
    elem.child(MMDPhysicsNode::aBodyFriction).setDouble(friction);
    elem.child(MMDPhysicsNode::aBodyRestitution).setDouble(restitution);
    elem.child(MMDPhysicsNode::aBodyColliderType).setShort(colliderType);
    elem.child(MMDPhysicsNode::aBodyRadius).setDouble(size[0]);
    setFloat3(elem.child(MMDPhysicsNode::aBodyExtents), size[0], size[1], size[2]);
    elem.child(MMDPhysicsNode::aBodyLength).setDouble(size[1]);
    elem.child(MMDPhysicsNode::aBodyGroupId).setShort(group);
    elem.child(MMDPhysicsNode::aBodyNonCollisionGroup).setInt(ncg & 0xFFFF);
    elem.child(MMDPhysicsNode::aBodyKinematic).setBool(kinematic);
    elem.child(MMDPhysicsNode::aBodyPhysicsMode).setShort(physicsModeInt);
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
                    .asShort() == 0)
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

// ===========================================================================
// Finalize mode + shared resolution helpers
// ===========================================================================

void MmdRigidBodyCmd::buildBodyMaps(MFnDependencyNode& fn, MPlug& bodiesPlug,
                                    std::map<int, int>& boneToBody,
                                    std::map<int, MDagPath>& boneToJoint,
                                    std::map<int, int>& boneToAnchor, int& kinematicCount)
{
    boneToBody.clear();
    boneToJoint.clear();
    boneToAnchor.clear();
    kinematicCount = 0;
    int count = bodiesPlug.numElements();
    for (int i = 0; i < count; ++i)
    {
        bool kinematic =
            bodiesPlug.elementByLogicalIndex(i).child(MMDPhysicsNode::aBodyPhysicsMode).asShort() ==
            0;
        MDagPath jp = bodyJointPath(fn, bodiesPlug, i);
        int b = -1;
        if (jp.isValid())
            b = jointPmxBoneIndex(jp);
        if (kinematic)
        {
            if (b >= 0 && !boneToAnchor.count(b))
                boneToAnchor[b] = kinematicCount;
            ++kinematicCount;
        }
        if (b >= 0)
        {
            boneToBody[b] = i; // last body on a bone wins
            boneToJoint[b] = jp;
        }
    }
}

void MmdRigidBodyCmd::resolveBody(MFnDependencyNode& fn, MPlug& bodiesPlug,
                                  const MDagPath& groupPath, const MMatrix& groupWorld, int n,
                                  bool connectFallback, const std::map<int, int>& boneToBody,
                                  const std::map<int, MDagPath>& boneToJoint,
                                  const std::map<int, int>& boneToAnchor)
{
    MPlug elem = bodiesPlug.elementByLogicalIndex(n);
    short mode = elem.child(MMDPhysicsNode::aBodyPhysicsMode).asShort();
    if (mode == 0)
    {
        elem.child(MMDPhysicsNode::aBodyParentBodyIndex).setInt(-1);
        elem.child(MMDPhysicsNode::aBodyResetAnchorIndex).setInt(-1);
        return;
    }
    MDagPath jpath = bodyJointPath(fn, bodiesPlug, n);
    if (!jpath.isValid())
        return;
    MFnDependencyNode jointFn(jpath.node());
    int ownBone = jointPmxBoneIndex(jpath);
    int parentBone = jointPmxParentBoneIndex(jpath);

    // Write-back parent (Phase 3 cycle fix): the parent inverse comes from the
    // PARENT BODY's solved Bullet transform (M_parent baked below).
    int parentRb = -1;
    MDagPath parentJointPath;
    if (parentBone >= 0)
    {
        std::map<int, int>::const_iterator it = boneToBody.find(parentBone);
        if (it != boneToBody.end())
        {
            parentRb = it->second;
            std::map<int, MDagPath>::const_iterator jt = boneToJoint.find(parentBone);
            if (jt != boneToJoint.end())
                parentJointPath = jt->second;
        }
    }
    elem.child(MMDPhysicsNode::aBodyParentBodyIndex).setInt(parentRb);
    if (parentRb >= 0 && parentJointPath.isValid())
    {
        double pt[3] = {0.0, 0.0, 0.0};
        double pr[3] = {0.0, 0.0, 0.0};
        readFloat3(
            bodiesPlug.elementByLogicalIndex(parentRb).child(MMDPhysicsNode::aBodyRestTranslate),
            pt);
        readFloat3(
            bodiesPlug.elementByLogicalIndex(parentRb).child(MMDPhysicsNode::aBodyRestRotate), pr);
        MMatrix parentWorld = matrixFromTR(pt, pr) * groupWorld;
        MMatrix mParent = worldMatrix(parentJointPath) * parentWorld.inverse();
        setMatrixValue(fn.findPlug(MMDPhysicsNode::aBodyParentJointOffset).elementByLogicalIndex(n),
                       mParent);
    }
    else if (connectFallback)
    {
        // Parent bone has no body — read its parent inverse from the DG (that
        // parent is never node-driven, so it cannot feed back).  Only safe
        // when the model is COMPLETE (finalize): a mid-import create would
        // later gain a node-driven parent and form a DG cycle.
        MPlug jpi = jointFn.findPlug("parentInverseMatrix").elementByLogicalIndex(0);
        connectOrReplace(
            jpi, fn.findPlug(MMDPhysicsNode::aBodyParentInverseMatrix).elementByLogicalIndex(n));
    }

    // Scrub-back reset anchor: nearest kinematic ancestor (walk the PMX bone
    // hierarchy via the joints' pmxBoneData — exact port of the Python map).
    int resetAnchor = -1;
    if (ownBone >= 0)
    {
        std::set<int> seen;
        int cur = ownBone;
        while (cur >= 0 && !seen.count(cur))
        {
            seen.insert(cur);
            std::map<int, int>::const_iterator it = boneToAnchor.find(cur);
            if (it != boneToAnchor.end())
            {
                resetAnchor = it->second;
                break;
            }
            MDagPath jp = resolveBone(MString(std::to_string(cur).c_str()), groupPath);
            if (!jp.isValid())
                break;
            cur = jointPmxParentBoneIndex(jp);
        }
    }
    elem.child(MMDPhysicsNode::aBodyResetAnchorIndex).setInt(resetAnchor);
}

MStatus MmdRigidBodyCmd::doFinalize(const MObject& solverNode)
{
    MFnDependencyNode fn(solverNode);
    MPlug bodiesPlug = fn.findPlug(MMDPhysicsNode::aBodies, true);
    if (bodiesPlug.isNull())
    {
        displayError("mmdRigidBody: node has no 'bodies' array");
        return MS::kFailure;
    }
    MDagPath nodePath;
    if (MDagPath::getAPathTo(solverNode, nodePath) != MS::kSuccess)
        return MS::kFailure;
    MDagPath groupPath = nodePath;
    groupPath.pop();
    MMatrix groupWorld = groupPath.inclusiveMatrix();

    std::map<int, int> boneToBody;
    std::map<int, MDagPath> boneToJoint;
    std::map<int, int> boneToAnchor;
    int kinematicCount = 0;
    buildBodyMaps(fn, bodiesPlug, boneToBody, boneToJoint, boneToAnchor, kinematicCount);

    int count = bodiesPlug.numElements();
    for (int i = 0; i < count; ++i)
    {
        try
        {
            resolveBody(fn, bodiesPlug, groupPath, groupWorld, i, true, boneToBody, boneToJoint,
                        boneToAnchor);
        }
        catch (...)
        {
        }
    }
    stepSolver(solverNode);
    return MS::kSuccess;
}
