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
 * Data conversions mirror the old Python writer's frame handling (Z-flip +
 * MMD radians -> Maya degrees handedness flip) and add the reflection to the
 * LIMITS that the old writer was missing:
 *   frame translate = (px, py, -pz)            (Z-flip)
 *   frame rotate    = (-rx, -ry, +rz) degrees  (MMD radians -> Maya degrees)
 *   frame stored in GROUP space (world * groupWorld^-1 — same as pmxRigidBody)
 *   linearMin/Max   = Z component negated + min/max swapped
 *   angularMin/Max  = X/Y negated + min/max swapped, Z unchanged (radians)
 *   springs         = verbatim (magnitudes, invariant under the reflection)
 *
 * WHY the limits are reflected: the MMD->Maya conversion is the reflection
 * F = diag(1, 1, -1) (Z-flip on position, rotation negated on X/Y).  The
 * joint limits live in the FRAME's local space, so they transform under the
 * same reflection: a rotation about local X/Y negates (F·Rx·F⁻¹ = Rx(-θ)), so
 * its [min, max] interval negates AND swaps; rotation about Z and linear X/Y
 * are unchanged; linear Z negates (the local Z axis reverses) so its interval
 * negates AND swaps.  Without this, every joint with ASYMMETRIC limits is
 * stored MIRRORED (429/496 joints in the test model have asymmetric angular
 * limits) and the constraint allows rotation in the wrong sense.
 *
 * The command's interface is minimal (see rigid_body_constraint_cmd.hpp); all
 * the implementation helpers live in the anonymous namespace below so the
 * header stays a pure interface.
 */

#include "rigid_body_constraint_cmd.hpp"

#include <maya/MArgList.h>
#include <maya/MArgParser.h>
#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MGlobal.h>
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

#include <string>

using mmd::core::Double3;
using mmd::core::physics_math::deg2rad;
using mmd::core::physics_math::rad2deg;

namespace
{
// ---------------------------------------------------------------------------
// Flag short names (single/compound, Maya API style).
// ---------------------------------------------------------------------------
constexpr const char* kIndexFlag = "i";
constexpr const char* kNameFlag = "n";
constexpr const char* kNameUniversalFlag = "nu";
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

// 4x4 row-vector matrix from translate + XYZ euler degrees (same helper as
// pmxRigidBody — shared by the frame's group-space conversion).
MMatrix matrixFromTR(const Double3& t, const Double3& r)
{
    MTransformationMatrix mt;
    mt.setTranslation(MVector(t.x, t.y, t.z), MSpace::kTransform);
    double rot[3] = {deg2rad(r.x), deg2rad(r.y), deg2rad(r.z)};
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    mt.setRotation(rot, MTransformationMatrix::kXYZ);
    return mt.asMatrix();
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

    MString nameLocal;
    MString nameUniversal;
    if (parser.isFlagSet(kNameFlag))
        nameLocal = parser.flagArgumentString(kNameFlag, 0);
    if (parser.isFlagSet(kNameUniversalFlag))
        nameUniversal = parser.flagArgumentString(kNameUniversalFlag, 0);

    Double3 pos;
    Double3 rot;
    Double3 lmin0;
    Double3 lmax0;
    Double3 amin0;
    Double3 amax0;
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
            lmin0[i] = parser.flagArgumentDouble(kLinearMinFlag, i);
    }
    if (parser.isFlagSet(kLinearMaxFlag))
    {
        for (int i = 0; i < 3; ++i)
            lmax0[i] = parser.flagArgumentDouble(kLinearMaxFlag, i);
    }
    if (parser.isFlagSet(kAngularMinFlag))
    {
        for (int i = 0; i < 3; ++i)
            amin0[i] = parser.flagArgumentDouble(kAngularMinFlag, i);
    }
    if (parser.isFlagSet(kAngularMaxFlag))
    {
        for (int i = 0; i < 3; ++i)
            amax0[i] = parser.flagArgumentDouble(kAngularMaxFlag, i);
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
    // A joint linking a body to ITSELF is degenerate (frameInA == frameInB on
    // the same body) and never meaningful — PMX never produces it (the test
    // model has 0/496), so reject it instead of writing a broken constraint.
    if (bodyA == bodyB)
    {
        MString msg = MString("bodyA == bodyB (") + MString(std::to_string(bodyA).c_str()) +
                      MString(") — a joint must link two DIFFERENT bodies");
        MGlobal::displayError(msg);
        return MS::kFailure;
    }

    // ── Group-space frame ──
    // The Bullet world runs in the physics group's local space (bodies store
    // group-space rest poses), so the joint frame must too — mirror the
    // pmxRigidBody group-space conversion (world * groupWorld^-1).  At import
    // the group is identity so this is a no-op; it keeps frames attached to
    // the bodies if the user ever transforms the physics group.
    MDagPath nodePath;
    if (MDagPath::getAPathTo(solverNode, nodePath) != MS::kSuccess)
    {
        MGlobal::displayError("pmxRigidBodyConstraint: could not resolve solver dag path");
        return MS::kFailure;
    }
    MDagPath groupPath = nodePath;
    groupPath.pop(); // |group|shape ⇒ |group
    const MMatrix groupWorld = groupPath.inclusiveMatrix();

    // Rest pose in group space (MMD ⇒ Maya: Z-flip + handedness).
    const Double3 worldT(pos.x, pos.y, -pos.z);
    const Double3 worldR(-rad2deg(rot.x), -rad2deg(rot.y), rad2deg(rot.z));
    const MMatrix local = matrixFromTR(worldT, worldR) * groupWorld.inverse();
    MTransformationMatrix mt(local);
    const MVector lt = mt.getTranslation(MSpace::kTransform);
    const MEulerRotation le = mt.eulerRotation();
    const Double3 localT(lt.x, lt.y, lt.z);
    const Double3 localR(rad2deg(le.x), rad2deg(le.y), rad2deg(le.z));

    // ── Limits through the MMD→Maya reflection F = diag(1, 1, -1) ──
    // Linear: X/Y unchanged; local Z reverses so its interval negates AND
    // swaps.  Angular: rotations about local X/Y negate (F·Rx·F⁻¹ = Rx(-θ)) so
    // their intervals negate AND swap; rotation about Z is invariant.  Springs
    // are magnitudes — invariant, passed through.
    const Double3 lmin(lmin0.x, lmin0.y, -lmax0.z);
    const Double3 lmax(lmax0.x, lmax0.y, -lmin0.z);
    const Double3 amin(-amax0.x, -amax0.y, amin0.z);
    const Double3 amax(-amin0.x, -amin0.y, amax0.z);

    // ── Write the joint data (simple create) ──
    MPlug elem = jointsPlug.elementByLogicalIndex(n);
    elem.child(PhysicsNode::aJointNameLocal).setString(nameLocal);
    elem.child(PhysicsNode::aJointNameUniversal).setString(nameUniversal);
    elem.child(PhysicsNode::aJointBodyA).setInt(bodyA);
    elem.child(PhysicsNode::aJointBodyB).setInt(bodyB);
    elem.child(PhysicsNode::aJointType).setInt(type);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointFrameTranslate), localT);
    mmd::maya::setPlugDouble3(elem.child(PhysicsNode::aJointFrameRotate), localR);
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
    syntax.addFlag(kNameFlag, "name", MSyntax::kString);
    syntax.addFlag(kNameUniversalFlag, "nameUniversal", MSyntax::kString);
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
