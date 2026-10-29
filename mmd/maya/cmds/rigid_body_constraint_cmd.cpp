/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_constraint_cmd.cpp
 *
 * Native C++ implementation of the ``pmxRigidBodyConstraint`` command (create
 * mode — the default).  Creates ONE PMX joint as a pmxRigidBodyJoint node,
 * registered at the solver's next free ``jointShapes`` index and linked to
 * its two bodies' pmxRigidBodyShape nodes by message connections.
 *
 * Data conversions (Z-flip + MMD radians -> Maya degrees handedness flip)
 * and the reflection applied to the LIMITS:
 *   frame translate = (px, py, -pz)            (Z-flip)
 *   frame rotate    = (-rx, -ry, +rz) degrees  (MMD radians -> Maya degrees)
 *   frame stored in the rigid bodies group's space (the bodies' rest space)
 *   linearMin/Max   = Z component negated + min/max swapped
 *   angularMin/Max  = X/Y negated + min/max swapped, Z unchanged (radians)
 *   stiffness       = verbatim (invariant magnitudes)
 * Only PMX 2.0's Spring6Dof (type 0) is accepted; the PMX 2.1 types are not
 * supported yet.
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
#include "common.hpp"
#include "pmx_rigid_body_cmd_utils.hpp"

#include <maya/MArgList.h>
#include <maya/MArgParser.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnTransform.h>
#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MStatus.h>
#include <maya/MSyntax.h>
#include <maya/MVector.h>

#include "maya_utils.hpp"
#include "nodes/rigid_body_joint.hpp"
#include "nodes/rigid_body_node.hpp"

#include <string>
#include <utility>

using mmd::core::Double3;

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
constexpr const char* kPositionFlag = "p";
constexpr const char* kRotationFlag = "rot";
// NOTE: MSyntax silently rejects SHORT flag names longer than 3 chars (a
// 4-char short like "lmin" never registers — addFlag returns failure).
constexpr const char* kLinearMinFlag = "lmi";
constexpr const char* kLinearMaxFlag = "lma";
constexpr const char* kAngularMinFlag = "ami";
constexpr const char* kAngularMaxFlag = "ama";
constexpr const char* kLinearStiffnessFlag = "lsk";
constexpr const char* kAngularStiffnessFlag = "ask";
constexpr const char* kNodeNameFlag = "nn";

// PMXNamingManager's RIGID_BODY_JOINT_SUFFIX: a joint is named after the
// body it moves often enough that the bare name would clash with that body,
// and bones already own "_Jnt".
constexpr const char* kJointNodeSuffix = "_Constraint";

// Reflect Spring6Dof PMX limits into Maya's right-handed joint frame
// (F = diag(1, 1, -1)): linear Z and angular X/Y intervals negate and swap.
// Linear X/Y and angular Z are invariant.
void reflectJointLimits(Double3& linearMin, Double3& linearMax, Double3& angularMin,
                        Double3& angularMax)
{
    const auto flip = [](double& lo, double& hi)
    {
        std::swap(lo, hi);
        lo = -lo;
        hi = -hi;
    };
    flip(linearMin.z, linearMax.z);
    flip(angularMin.x, angularMax.x);
    flip(angularMin.y, angularMax.y);
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
    if (parser.isFlagSet(kBodyAFlag))
        bodyA = parser.flagArgumentInt(kBodyAFlag, 0);
    if (parser.isFlagSet(kBodyBFlag))
        bodyB = parser.flagArgumentInt(kBodyBFlag, 0);

    MString nameLocal;
    MString nameUniversal;
    if (parser.isFlagSet(kNameFlag))
        nameLocal = parser.flagArgumentString(kNameFlag, 0);
    if (parser.isFlagSet(kNameUniversalFlag))
        nameUniversal = parser.flagArgumentString(kNameUniversalFlag, 0);
    MString nodeName;
    if (parser.isFlagSet(kNodeNameFlag))
        nodeName = parser.flagArgumentString(kNodeNameFlag, 0);

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
    if (parser.isFlagSet(kLinearStiffnessFlag))
    {
        for (int i = 0; i < 3; ++i)
            ls[i] = parser.flagArgumentDouble(kLinearStiffnessFlag, i);
    }
    if (parser.isFlagSet(kAngularStiffnessFlag))
    {
        for (int i = 0; i < 3; ++i)
            as[i] = parser.flagArgumentDouble(kAngularStiffnessFlag, i);
    }

    // ── Solver / index ──
    MFnDependencyNode fn(solverNode);
    MStatus plugStat;
    MPlug jointShapesPlug = fn.findPlug(RigidBodyNode::aJointShapes, true, &plugStat);
    if (jointShapesPlug.isNull())
    {
        MGlobal::displayError("pmxRigidBodyConstraint: node has no 'jointShapes' array");
        return MS::kFailure;
    }
    // The next free joint index is the array's CURRENT logical element count
    // (evaluateNumElements, NOT numElements — see the body command).
    const int count = static_cast<int>(jointShapesPlug.evaluateNumElements());
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
    MPlug shapesPlug = fn.findPlug(RigidBodyNode::aBodyShapes, true, &plugStat);
    if (shapesPlug.isNull())
    {
        MGlobal::displayError("pmxRigidBodyConstraint: node has no 'bodyShapes' array");
        return MS::kFailure;
    }
    const int bodyCount = static_cast<int>(shapesPlug.evaluateNumElements());
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
    // The joint links the bodies' SHAPE nodes (message connections), not
    // their indices: the shape in each bodyShapes[] slot.
    const auto bodyShapeAt = [&shapesPlug](int index)
    {
        MPlugArray sources;
        shapesPlug.elementByLogicalIndex(index).connectedTo(sources, true, false);
        return sources.length() > 0 ? sources[0].node() : MObject::kNullObj;
    };
    const MObject shapeA = bodyShapeAt(bodyA);
    const MObject shapeB = bodyShapeAt(bodyB);
    for (const auto& [slot, shape] : {std::pair(bodyA, shapeA), std::pair(bodyB, shapeB)})
    {
        if (shape.isNull())
        {
            MGlobal::displayError(MString("pmxRigidBodyConstraint: bodyShapes[") +
                                  MString(std::to_string(slot).c_str()) +
                                  "] has no connected pmxRigidBodyShape");
            return MS::kFailure;
        }
    }

    // ── Frame, in the space of the bodies' rest poses ──
    // Bodies store their PMX rest poses in the model space, so the joint
    // frame is stored in it too (MMD ⇒ Maya: Z-flip + handedness).
    const Double3 frameT = mmd::maya::mmdToMayaTranslate(pos);
    const Double3 frameR = mmd::maya::mmdToMayaRotateDeg(rot);

    // ── Limits through the MMD→Maya reflection F = diag(1, 1, -1) ──
    // Linear: X/Y unchanged; local Z reverses so its interval negates AND
    // swaps.  Angular: rotations about local X/Y negate (F·Rx·F⁻¹ = Rx(-θ)) so
    // their intervals negate AND swap; rotation about Z is invariant.  Springs
    // are magnitudes — invariant, passed through.
    reflectJointLimits(lmin, lmax, amin, amax);

    // ── Create the per-joint node (pmxRigidBodyJoint) ──
    // One selectable joint per PMX constraint: a plain TRANSFORM holding the
    // locator shape under it (like the per-body guides), parented under the
    // rigid bodies group.  The TRANSFORM is the joint FRAME (joints are
    // static config — move/rotate it to reposition the constraint); all
    // PMX-verbatim joint data lives on the shape.  The solver discovers it
    // via solver.jointShapes[n].
    MFnDagNode solverDagFn(solverNode, &plugStat);
    MObject groupObj = solverDagFn.parent(0, &plugStat);
    const MString jointName =
        nodeName.length() > 0 ? nodeName
                              : mmd::maya::pmxNodeName(solverNode, nameLocal, nameUniversal,
                                                       "Joint_" + std::to_string(n),
                                                       kJointNodeSuffix);
    MObject guideParent = groupObj;
    MFnTransform jointTf;
    MObject jointTransform = jointTf.create(guideParent, &plugStat);
    if (jointTransform.isNull())
    {
        MGlobal::displayError("pmxRigidBodyConstraint: could not create the joint guide transform");
        return MS::kFailure;
    }
    jointTf.setName(jointName);
    // The locator shape (data holder + draw target) lives under the transform;
    // Maya names it `{guideName}Shape` automatically.
    MFnDagNode jointDagFn;
    MObject shapeParent = jointTransform;
    MObject jointObj =
        jointDagFn.create(RigidBodyJoint::kNodeName, MString(), shapeParent, &plugStat);
    if (jointObj.isNull())
    {
        MGlobal::displayError(
            "pmxRigidBodyConstraint: could not create the pmxRigidBodyJoint node");
        return MS::kFailure;
    }
    // The joint transform's LOCAL value under the group IS the frame: the
    // solver measures frames in the group's space, the space of the bodies'
    // rest poses (RigidBodyJoint::readJointFrame), so wherever the group or
    // the model root sits, the constraint stays where the PMX puts it.
    const double kDegToRad = 3.14159265358979323846 / 180.0;
    jointTf.setTranslation(MVector(frameT.x, frameT.y, frameT.z), MSpace::kTransform);
    jointTf.setRotation(MEulerRotation(frameR.x * kDegToRad, frameR.y * kDegToRad,
                                       frameR.z * kDegToRad, MEulerRotation::kXYZ));

    MFnDependencyNode jointFn(jointObj);
    const auto jointPlug = [&jointFn, &plugStat](const MObject& a) -> MPlug
    { return jointFn.findPlug(a, true, &plugStat); };
    jointPlug(RigidBodyJoint::aJointNameLocal).setString(nameLocal);
    jointPlug(RigidBodyJoint::aJointNameUniversal).setString(nameUniversal);
    // The linked bodies: shape.message -> joint.jointBodyA / jointBodyB.
    mmd::maya::connectOrReplace(MFnDependencyNode(shapeA).findPlug("message", true, &plugStat),
                                jointPlug(RigidBodyJoint::aJointBodyA));
    mmd::maya::connectOrReplace(MFnDependencyNode(shapeB).findPlug("message", true, &plugStat),
                                jointPlug(RigidBodyJoint::aJointBodyB));
    mmd::maya::setPlugDouble3(jointPlug(RigidBodyJoint::aJointLinearMin), lmin);
    mmd::maya::setPlugDouble3(jointPlug(RigidBodyJoint::aJointLinearMax), lmax);
    mmd::maya::setPlugDouble3(jointPlug(RigidBodyJoint::aJointAngularMin), amin);
    mmd::maya::setPlugDouble3(jointPlug(RigidBodyJoint::aJointAngularMax), amax);
    mmd::maya::setPlugDouble3(jointPlug(RigidBodyJoint::aJointLinearStiffness), ls);
    mmd::maya::setPlugDouble3(jointPlug(RigidBodyJoint::aJointAngularStiffness), as);

    // Register the joint on the solver: joint.solver -> solver.jointShapes[n]
    // (PMX order — the solver pulls each joint from its node in readJointData).
    mmd::maya::connectOrReplace(jointPlug(RigidBodyJoint::aSolver),
                                jointShapesPlug.elementByLogicalIndex(n));
    // Redraw the joint after every solver step: solver.outStep -> joint.solverStep.
    mmd::maya::connectOrReplace(MPlug(solverNode, RigidBodyNode::aOutStep),
                                jointPlug(RigidBodyJoint::aSolverStep));
    // Re-run the solver when the frame moves: transform.worldMatrix[0] ->
    // joint.frameMatrix.
    const MPlug frameWorld =
        MFnDependencyNode(jointTransform).findPlug("worldMatrix", true, &plugStat);
    mmd::maya::connectOrReplace(frameWorld.elementByLogicalIndex(0),
                                jointPlug(RigidBodyJoint::aFrameMatrix));

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
    syntax.addFlag(kNodeNameFlag, "nodeName", MSyntax::kString);
    syntax.addFlag(kBodyAFlag, "bodyA", MSyntax::kLong);
    syntax.addFlag(kBodyBFlag, "bodyB", MSyntax::kLong);
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
    syntax.addFlag(kLinearStiffnessFlag, "linearStiffness", MSyntax::kDouble, MSyntax::kDouble,
                   MSyntax::kDouble);
    syntax.addFlag(kAngularStiffnessFlag, "angularStiffness", MSyntax::kDouble, MSyntax::kDouble,
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
    if (!mmd::maya::resolveSolver(target, solverNode))
    {
        displayError("'" + target + "' is not a pmxRigidBodyNode or a PMX model root");
        return MS::kFailure;
    }

    int newIndex = -1;
    stat = doCreate(parser, solverNode, newIndex);
    if (stat)
        setResult(newIndex);
    return stat;
}
