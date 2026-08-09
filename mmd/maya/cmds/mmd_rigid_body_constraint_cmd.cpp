/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_rigid_body_constraint_cmd.cpp
 *
 * Native C++ implementation of the ``mmdRigidBodyConstraint`` command (create
 * mode — the default).  Writes ONE PMX joint into the node's ``joints`` array
 * at the next free index, replacing the former Python
 * ``_set_joint_attributes``.
 *
 * Data conversions match the old Python writer exactly:
 *   frame translate = (px, py, -pz)             (Z-flip)
 *   frame rotate    = (-rx, -ry, +rz) degrees   (MMD radians -> Maya degrees)
 * Limits / springs pass straight through (linear in PMX units, angular in PMX
 * radians — the node hands angular values to Bullet unchanged).
 */

#include "mmd_rigid_body_constraint_cmd.h"

#include "mmd_common.h"

#include <maya/MArgList.h>
#include <maya/MArgParser.h>
#include <maya/MDagPath.h>
#include <maya/MDataHandle.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnNumericData.h>
#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MSyntax.h>
#include <maya/MVector.h>

#include "../nodes/mmd_physics_node.h"

#include <cstdlib>
#include <cstring>

namespace
{
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
inline double radToDeg(double r)
{
    return r / kDegToRad;
}

// MPlug has no setValue3Double — wrap 3 doubles in an MFnNumericData object.
void setDouble3(MPlug& plug, const Double3& v)
{
    MFnNumericData data;
    MObject obj = data.create(MFnNumericData::k3Double);
    data.setData3Double(v.x, v.y, v.z);
    plug.setValue(obj);
}

// Flag short names.
constexpr const char* kIndexFlag = "i";
constexpr const char* kBodyAFlag = "ba";
constexpr const char* kBodyBFlag = "bb";
constexpr const char* kTypeFlag = "t";
constexpr const char* kPositionFlag = "p";
constexpr const char* kRotationFlag = "rot";
// NOTE: MSyntax silently rejects SHORT flag names longer than 3 chars (a
// 4-char short like "lmin" never registers — addFlag returns failure).
constexpr const char* kLinearMinFlag = "lmi";
constexpr const char* kLinearMaxFlag = "lma";
constexpr const char* kAngularMinFlag = "ami";
constexpr const char* kAngularMaxFlag = "ama";
constexpr const char* kLinearSpringFlag = "ls";
constexpr const char* kAngularSpringFlag = "as";
} // namespace

MSyntax MmdRigidBodyConstraintCmd::syntaxCreator()
{
    MSyntax syntax;
    // First positional argument: the solver node (or a model root).
    syntax.addArg(MSyntax::kString);

    syntax.addFlag(kIndexFlag, "index", MSyntax::kLong);
    syntax.addFlag(kBodyAFlag, "bodyA", MSyntax::kLong);
    syntax.addFlag(kBodyBFlag, "bodyB", MSyntax::kLong);
    syntax.addFlag(kTypeFlag, "type", MSyntax::kLong);
    syntax.addFlag(kPositionFlag, "position", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    syntax.addFlag(kRotationFlag, "rotation", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    syntax.addFlag(kLinearMinFlag, "linearMin", MSyntax::kDouble, MSyntax::kDouble,
                   MSyntax::kDouble);
    syntax.addFlag(kLinearMaxFlag, "linearMax", MSyntax::kDouble, MSyntax::kDouble,
                   MSyntax::kDouble);
    syntax.addFlag(kAngularMinFlag, "angularMin", MSyntax::kDouble, MSyntax::kDouble,
                   MSyntax::kDouble);
    syntax.addFlag(kAngularMaxFlag, "angularMax", MSyntax::kDouble, MSyntax::kDouble,
                   MSyntax::kDouble);
    syntax.addFlag(kLinearSpringFlag, "linearSpring", MSyntax::kDouble, MSyntax::kDouble,
                   MSyntax::kDouble);
    syntax.addFlag(kAngularSpringFlag, "angularSpring", MSyntax::kDouble, MSyntax::kDouble,
                   MSyntax::kDouble);

    syntax.enableEdit(true);
    syntax.enableQuery(true);
    return syntax;
}

void* MmdRigidBodyConstraintCmd::creator()
{
    return new MmdRigidBodyConstraintCmd();
}

bool MmdRigidBodyConstraintCmd::resolveSolver(const MString& target, MObject& outNode)
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

MStatus MmdRigidBodyConstraintCmd::doIt(const MArgList& args)
{
    MStatus stat;
    MArgParser parser(syntaxCreator(), args, &stat);
    if (!stat)
    {
        displayError("mmdRigidBodyConstraint: could not parse arguments");
        return stat;
    }

    MString target = parser.commandArgumentString(0, &stat);
    if (!stat || target.length() == 0)
    {
        displayError("mmdRigidBodyConstraint: missing solver / modelRoot argument");
        return MS::kFailure;
    }

    if (parser.isQuery())
    {
        displayError("mmdRigidBodyConstraint query mode is not implemented yet");
        return MS::kFailure;
    }
    if (parser.isEdit())
    {
        displayError("mmdRigidBodyConstraint edit mode is not implemented yet");
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

MStatus MmdRigidBodyConstraintCmd::doCreate(const MArgParser& parser, const MObject& solverNode,
                                            int& outIndex)
{
    // ── Parse flags (safe defaults mirror the former Python writer) ──
    int index = -1;
    if (parser.isFlagSet(kIndexFlag))
        index = static_cast<int>(parser.flagArgumentInt(kIndexFlag, 0));

    int bodyA = 0;
    int bodyB = 0;
    int type = 0;
    if (parser.isFlagSet(kBodyAFlag))
        bodyA = static_cast<int>(parser.flagArgumentInt(kBodyAFlag, 0));
    if (parser.isFlagSet(kBodyBFlag))
        bodyB = static_cast<int>(parser.flagArgumentInt(kBodyBFlag, 0));
    if (parser.isFlagSet(kTypeFlag))
        type = static_cast<int>(parser.flagArgumentInt(kTypeFlag, 0));

    Double3 pos;
    Double3 rot;
    if (parser.isFlagSet(kPositionFlag))
        for (int i = 0; i < 3; ++i)
            pos[i] = parser.flagArgumentDouble(kPositionFlag, i);
    if (parser.isFlagSet(kRotationFlag))
        for (int i = 0; i < 3; ++i)
            rot[i] = parser.flagArgumentDouble(kRotationFlag, i);

    Double3 lmin;
    Double3 lmax;
    Double3 amin;
    Double3 amax;
    Double3 ls;
    Double3 as;
    if (parser.isFlagSet(kLinearMinFlag))
        for (int i = 0; i < 3; ++i)
            lmin[i] = parser.flagArgumentDouble(kLinearMinFlag, i);
    if (parser.isFlagSet(kLinearMaxFlag))
        for (int i = 0; i < 3; ++i)
            lmax[i] = parser.flagArgumentDouble(kLinearMaxFlag, i);
    if (parser.isFlagSet(kAngularMinFlag))
        for (int i = 0; i < 3; ++i)
            amin[i] = parser.flagArgumentDouble(kAngularMinFlag, i);
    if (parser.isFlagSet(kAngularMaxFlag))
        for (int i = 0; i < 3; ++i)
            amax[i] = parser.flagArgumentDouble(kAngularMaxFlag, i);
    if (parser.isFlagSet(kLinearSpringFlag))
        for (int i = 0; i < 3; ++i)
            ls[i] = parser.flagArgumentDouble(kLinearSpringFlag, i);
    if (parser.isFlagSet(kAngularSpringFlag))
        for (int i = 0; i < 3; ++i)
            as[i] = parser.flagArgumentDouble(kAngularSpringFlag, i);

    // ── Solver / index (auto-append at the next free index) ──
    MFnDependencyNode fn(solverNode);
    MPlug jointsPlug = fn.findPlug(MMDPhysicsNode::aJoints, true);
    if (jointsPlug.isNull())
    {
        displayError("mmdRigidBodyConstraint: node has no 'joints' array");
        return MS::kFailure;
    }
    int count = jointsPlug.numElements();
    if (index >= 0 && index != count)
    {
        MString msg = MString("Joint index ") + MString(std::to_string(index).c_str()) +
                      MString(" is not the next free index (") +
                      MString(std::to_string(count).c_str()) +
                      MString(") — append at the end or use edit mode to overwrite");
        displayError(msg);
        return MS::kFailure;
    }
    int n = count;

    // ── Write the joint data (MMD -> Maya conversions, as the Python did) ──
    MPlug elem = jointsPlug.elementByLogicalIndex(n);
    elem.child(MMDPhysicsNode::aJointBodyA).setInt(bodyA);
    elem.child(MMDPhysicsNode::aJointBodyB).setInt(bodyB);
    elem.child(MMDPhysicsNode::aJointType).setInt(type);
    setDouble3(elem.child(MMDPhysicsNode::aJointFrameTranslate), Double3(pos.x, pos.y, -pos.z));
    setDouble3(elem.child(MMDPhysicsNode::aJointFrameRotate),
               Double3(-radToDeg(rot.x), -radToDeg(rot.y), radToDeg(rot.z)));
    setDouble3(elem.child(MMDPhysicsNode::aJointLinearMin), lmin);
    setDouble3(elem.child(MMDPhysicsNode::aJointLinearMax), lmax);
    setDouble3(elem.child(MMDPhysicsNode::aJointAngularMin), amin);
    setDouble3(elem.child(MMDPhysicsNode::aJointAngularMax), amax);
    setDouble3(elem.child(MMDPhysicsNode::aJointLinearSpring), ls);
    setDouble3(elem.child(MMDPhysicsNode::aJointAngularSpring), as);

    outIndex = n;
    return MS::kSuccess;
}
