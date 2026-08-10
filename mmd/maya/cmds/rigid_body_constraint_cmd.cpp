/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_constraint_cmd.cpp
 *
 * Native C++ implementation of the ``pmxRigidBodyConstraint`` command (create
 * mode — the default).  Writes ONE PMX joint into an pmxPhysicsNode's
 * ``joints`` array at the next free index, replacing the former Python
 * ``_set_joint_attributes``.
 *
 * Data conversions match the old Python writer exactly:
 *   frame translate = (px, py, -pz)             (Z-flip)
 *   frame rotate    = (-rx, -ry, +rz) degrees   (MMD radians -> Maya degrees)
 *   limits / springs pass straight through (linear in PMX units, angular in
 *   PMX radians — the node hands angular values to Bullet unchanged).
 *
 * The command's interface is minimal (see rigid_body_constraint_cmd.hpp); all
 * the implementation helpers live in the anonymous namespace below so the
 * header stays a pure interface.
 */

#include "rigid_body_constraint_cmd.hpp"

#include <maya/MArgList.h>
#include <maya/MArgParser.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MSyntax.h>

#include "maya_utils.hpp"
#include "nodes/physics_node.h"
#include "physics_math.hpp"

#include <string>

using mmd::core::Double3;
using mmd::core::physics_math::rad2deg;

namespace
{
// ---------------------------------------------------------------------------
// Flag short names (single/compound, Maya API style).
// ---------------------------------------------------------------------------
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

// Highest PMX joint type value (JointType::HINGE).
constexpr int kMaxJointType = 5;

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
    // ── Parse flags (safe defaults mirror the former Python writer) ──
    int index = -1;
    if (parser.isFlagSet(kIndexFlag))
        index = parser.flagArgumentInt(kIndexFlag, 0);

    int bodyA = 0;
    int bodyB = 0;
    int type = 0;
    if (parser.isFlagSet(kBodyAFlag))
        bodyA = parser.flagArgumentInt(kBodyAFlag, 0);
    if (parser.isFlagSet(kBodyBFlag))
        bodyB = parser.flagArgumentInt(kBodyBFlag, 0);
    if (parser.isFlagSet(kTypeFlag))
        type = parser.flagArgumentInt(kTypeFlag, 0);

    Double3 pos;
    Double3 rot;
    Double3 lmin;
    Double3 lmax;
    Double3 amin;
    Double3 amax;
    Double3 ls;
    Double3 as;
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
    if (parser.isFlagSet(kLinearMinFlag))
    {
        for (int i = 0; i < 3; ++i)
            lmin[i] = parser.flagArgumentDouble(kLinearMinFlag, i);
    }
    if (parser.isFlagSet(kLinearMaxFlag))
    {
        for (int i = 0; i < 3; ++i)
            lmax[i] = parser.flagArgumentDouble(kLinearMaxFlag, i);
    }
    if (parser.isFlagSet(kAngularMinFlag))
    {
        for (int i = 0; i < 3; ++i)
            amin[i] = parser.flagArgumentDouble(kAngularMinFlag, i);
    }
    if (parser.isFlagSet(kAngularMaxFlag))
    {
        for (int i = 0; i < 3; ++i)
            amax[i] = parser.flagArgumentDouble(kAngularMaxFlag, i);
    }
    if (parser.isFlagSet(kLinearSpringFlag))
    {
        for (int i = 0; i < 3; ++i)
            ls[i] = parser.flagArgumentDouble(kLinearSpringFlag, i);
    }
    if (parser.isFlagSet(kAngularSpringFlag))
    {
        for (int i = 0; i < 3; ++i)
            as[i] = parser.flagArgumentDouble(kAngularSpringFlag, i);
    }

    // ── Joint type validation (PMX JointType 0..5) ──
    if (type < 0 || type > kMaxJointType)
    {
        MGlobal::displayError("Unknown joint type '" + MString(std::to_string(type).c_str()) +
                              "' — expected 0..5 (SPRING_6DOF/6DOF/P2P/CONETWIST/SLIDER/HINGE)");
        return MS::kFailure;
    }

    // ── Solver / index ──
    MFnDependencyNode fn(solverNode);
    MStatus plugStat;
    MPlug jointsPlug = fn.findPlug(PhysicsNode::aJoints, true, &plugStat);
    if (jointsPlug.isNull())
    {
        MGlobal::displayError("pmxRigidBodyConstraint: node has no 'joints' array");
        return MS::kFailure;
    }
    const int count = static_cast<int>(jointsPlug.numElements());
    if (index >= 0 && index != count)
    {
        MString msg = MString("Joint index ") + MString(std::to_string(index).c_str()) +
                      MString(" is not the next free index (") +
                      MString(std::to_string(count).c_str()) +
                      MString(") — append at the end or use edit mode to overwrite");
        MGlobal::displayError(msg);
        return MS::kFailure;
    }
    const int n = count;

    // ── Validate the referenced rigid bodies ──
    // PMX joints are constraints BETWEEN bodies (rigid_body_index_a/b), so
    // the referenced bodies must already exist.  Bodies are appended first by
    // the importer (pmxRigidBody loops before this command), and the node's
    // engine silently SKIPS joints whose bodies are missing — catching the
    // mistake here is far friendlier than a silently-dead constraint.
    MPlug bodiesPlug = fn.findPlug(PhysicsNode::aBodies, true, &plugStat);
    if (bodiesPlug.isNull())
    {
        MGlobal::displayError("pmxRigidBodyConstraint: node has no 'bodies' array");
        return MS::kFailure;
    }
    const int bodyCount = static_cast<int>(bodiesPlug.numElements());
    if (bodyA < 0 || bodyA >= bodyCount || bodyB < 0 || bodyB >= bodyCount)
    {
        MString msg = MString("bodyA/bodyB (") + MString(std::to_string(bodyA).c_str()) +
                      MString(", ") + MString(std::to_string(bodyB).c_str()) +
                      MString(") out of range — node has ") +
                      MString(std::to_string(bodyCount).c_str()) + MString(" bodies");
        MGlobal::displayError(msg);
        return MS::kFailure;
    }

    // ── Write the joint data (simple create) ──
    // Frame conversions match the old Python writer: Z-flip on position,
    // MMD radians -> Maya degrees with the handedness flip on rotation.
    // Limits/springs pass straight through (linear in PMX units, angular in
    // PMX radians — the node hands angular values to Bullet unchanged).
    MPlug elem = jointsPlug.elementByLogicalIndex(n);
    elem.child(PhysicsNode::aJointBodyA).setInt(bodyA);
    elem.child(PhysicsNode::aJointBodyB).setInt(bodyB);
    elem.child(PhysicsNode::aJointType).setInt(type);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointFrameTranslate),
                              Double3(pos.x, pos.y, -pos.z));
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointFrameRotate),
                              Double3(-rad2deg(rot.x), -rad2deg(rot.y), rad2deg(rot.z)));
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointLinearMin), lmin);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointLinearMax), lmax);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointAngularMin), amin);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointAngularMax), amax);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointLinearSpring), ls);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointAngularSpring), as);

    outIndex = n;
    return MS::kSuccess;
}

} // namespace

// ===========================================================================
// Registration
// ===========================================================================

void* RigidBodyConstraintCmd::creator()
{
    return new RigidBodyConstraintCmd();
}

MSyntax RigidBodyConstraintCmd::syntaxCreator()
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

// ===========================================================================
// doIt
// ===========================================================================

MStatus RigidBodyConstraintCmd::doIt(const MArgList& args)
{
    MStatus stat;
    MArgParser parser(syntaxCreator(), args, &stat);
    if (!stat)
    {
        displayError("pmxRigidBodyConstraint: could not parse arguments");
        return stat;
    }

    const MString target = parser.commandArgumentString(0, &stat);
    if (!stat || target.length() == 0)
    {
        displayError("pmxRigidBodyConstraint: missing solver / modelRoot argument");
        return MS::kFailure;
    }

    if (parser.isQuery())
    {
        displayError("pmxRigidBodyConstraint query mode is not implemented yet");
        return MS::kFailure;
    }
    if (parser.isEdit())
    {
        displayError("pmxRigidBodyConstraint edit mode is not implemented yet");
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
