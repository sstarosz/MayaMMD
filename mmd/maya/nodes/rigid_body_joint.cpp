/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint.cpp
 *
 * RigidBodyJoint — one selectable, movable Maya node per PMX rigid-body joint
 * (see rigid_body_joint.hpp).
 *
 * The node is a plain data holder: every PMX joint field except the FRAME is
 * a storable attribute (no compute() — the solver reads them through its
 * `jointShapes[]` message array).  The node's DAG TRANSFORM is the joint's
 * FRAME — joints are static config, so moving/rotating the node repositions
 * the constraint, and the solver reads the frame straight from the transform
 * in the {model}_RigidBodies group's space (translate + XYZ rotate).  The
 * joint is drawn by RigidBodyJointGeometryOverride (not here); the node only
 * tells VP2 when that drawing is out of date.
 */

#include "rigid_body_joint.hpp"

#include <maya/MDGContext.h>
#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTransform.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>
#include <maya/MViewport2Renderer.h>

#include "joint_display.hpp"
#include "maya_utils.hpp"
#include "rigid_body_node.hpp"
#include "rigid_body_shape.hpp"
#include "rigid_body_simulation.hpp"

#include <maya/MPlugArray.h>

#include <algorithm>
#include <cmath>
#include <string>

using mmd::core::Double3;
using mmd::core::RigidBodySimulation;

namespace
{
void readDouble3(const MPlug& plug, Double3& out)
{
    out = mmd::maya::readPlugDouble3(plug);
}

// Numeric attribute helper: storable, non-keyable, readable+writable.
[[nodiscard]] MObject makeNumeric(MFnNumericAttribute& fn, const MString& longName,
                                  const MString& shortName, MFnNumericData::Type type,
                                  double defaultValue)
{
    MStatus stat;
    MObject a = fn.create(longName, shortName, type, defaultValue, &stat);
    MMD_CHECK_MSTATUS(stat);
    fn.setStorable(true);
    fn.setKeyable(false);
    fn.setReadable(true);
    fn.setWritable(true);
    return a;
}

// Angle-triple attribute (like a transform's rotate): stored in radians, shown
// and set from scripts in the user's UI angle unit (degrees by default).
[[nodiscard]] MObject makeAngle3(MFnNumericAttribute& fn, const MString& longName,
                                 const MString& shortName)
{
    MStatus stat;
    MFnUnitAttribute uAttr;
    MObject axes[3];
    const char* suffix[3] = {"X", "Y", "Z"};
    const char* shortSuffix[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i)
    {
        axes[i] = uAttr.create(longName + suffix[i], shortName + shortSuffix[i],
                               MFnUnitAttribute::kAngle, 0.0, &stat);
        MMD_CHECK_MSTATUS(stat);
        uAttr.setStorable(true);
        uAttr.setKeyable(false);
    }
    MObject a = fn.create(longName, shortName, axes[0], axes[1], axes[2], &stat);
    MMD_CHECK_MSTATUS(stat);
    fn.setStorable(true);
    fn.setKeyable(false);
    fn.setReadable(true);
    fn.setWritable(true);
    return a;
}

// String attribute helper: storable, non-keyable, readable+writable.
[[nodiscard]] MObject makeString(MFnTypedAttribute& fn, const MString& longName,
                                 const MString& shortName)
{
    MStatus stat;
    MObject a = fn.create(longName, shortName, MFnData::kString, &stat);
    MMD_CHECK_MSTATUS(stat);
    fn.setStorable(true);
    fn.setKeyable(false);
    fn.setReadable(true);
    fn.setWritable(true);
    return a;
}
} // namespace

// ===========================================================================
// Static attribute definitions
// ===========================================================================
const MTypeId RigidBodyJoint::kTypeId(0x0011C107); // unique Maya node type id

MObject RigidBodyJoint::aSolver;
MObject RigidBodyJoint::aSolverStep;
MObject RigidBodyJoint::aFrameMatrix;
MObject RigidBodyJoint::aJointNameLocal;
MObject RigidBodyJoint::aJointNameUniversal;
MObject RigidBodyJoint::aJointBodyA;
MObject RigidBodyJoint::aJointBodyB;
MObject RigidBodyJoint::aJointType;
MObject RigidBodyJoint::aJointLinearMin;
MObject RigidBodyJoint::aJointLinearMax;
MObject RigidBodyJoint::aJointAngularMin;
MObject RigidBodyJoint::aJointAngularMax;
MObject RigidBodyJoint::aJointLinearStiffness;
MObject RigidBodyJoint::aJointAngularStiffness;
MObject RigidBodyJoint::aDrawMode;

// ===========================================================================
// Lifecycle
// ===========================================================================
RigidBodyJoint::RigidBodyJoint() = default;
RigidBodyJoint::~RigidBodyJoint() = default;

void* RigidBodyJoint::creator()
{
    return new RigidBodyJoint();
}

MStatus RigidBodyJoint::initialize()
{
    MStatus stat;
    MFnMessageAttribute mAttr;
    MFnNumericAttribute nAttr;
    MFnTypedAttribute tAttr;
    MFnEnumAttribute eAttr;

    // --- solver back-pointer (message) ---
    aSolver = mAttr.create("solver", "slv", &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(false); // connections are not stored as data

    aSolverStep = nAttr.create("solverStep", "sstp", MFnNumericData::kDouble, 0.0, &stat);
    MMD_CHECK_MSTATUS(stat);
    nAttr.setStorable(false);
    nAttr.setKeyable(false);
    nAttr.setHidden(true);

    {
        MFnMatrixAttribute matAttr;
        aFrameMatrix = matAttr.create("frameMatrix", "fmtx", MFnMatrixAttribute::kDouble, &stat);
        MMD_CHECK_MSTATUS(stat);
        matAttr.setStorable(false);
        matAttr.setKeyable(false);
        matAttr.setHidden(true);
    }

    // --- joint data ---
    aJointNameLocal = makeString(tAttr, "jointNameLocal", "jnml");
    aJointNameUniversal = makeString(tAttr, "jointNameUniversal", "jnmu");
    // --- the linked bodies (message <- each pmxRigidBodyShape.message) ---
    aJointBodyA = mAttr.create("jointBodyA", "jba", &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(false); // connections are not stored as data
    aJointBodyB = mAttr.create("jointBodyB", "jbb", &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(false);

    // PMX joint type — enum: PMX 2.0 only has Spring6Dof.  The joint FRAME
    // (position + orientation) is NOT an attribute — it is the node's DAG
    // transform.
    {
        aJointType = eAttr.create("jointType", "jt", kSpring6Dof, &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("Spring6Dof", kSpring6Dof);
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

    aJointLinearMin = makeNumeric(nAttr, "jointLinearMin", "jlmn", MFnNumericData::k3Double, 0.0);
    aJointLinearMax = makeNumeric(nAttr, "jointLinearMax", "jlmx", MFnNumericData::k3Double, 0.0);
    aJointAngularMin = makeAngle3(nAttr, "jointAngularMin", "jamn");
    aJointAngularMax = makeAngle3(nAttr, "jointAngularMax", "jamx");
    aJointLinearStiffness =
        makeNumeric(nAttr, "jointLinearStiffness", "jlsk", MFnNumericData::k3Double, 0.0);
    aJointAngularStiffness =
        makeNumeric(nAttr, "jointAngularStiffness", "jask", MFnNumericData::k3Double, 0.0);

    // Viewport draw mode — enum: off / hub / ranges (see the header).
    {
        aDrawMode = eAttr.create("drawMode", "dm", kDrawRanges, &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("Off", kDrawOff);
        eAttr.addField("Hub", kDrawHub);
        eAttr.addField("Ranges", kDrawRanges);
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

    // --- attribute registration ---
    stat = addAttribute(aSolver);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aSolverStep);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aFrameMatrix);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointNameLocal);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointNameUniversal);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointBodyA);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointBodyB);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointType);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointLinearMin);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointLinearMax);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointAngularMin);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointAngularMax);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointLinearStiffness);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointAngularStiffness);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aDrawMode);
    MMD_CHECK_MSTATUS(stat);

    // An edit must re-run the solver, which pulls the joint through the
    // message array (not a data connection): every simulated field, and the
    // frame (through frameMatrix — the inherited worldMatrix is dirtied by
    // the DAG, not through attributeAffects), dirties the solver message ->
    // pmxRigidBodyNode.jointShapes[i] -> its outputs.
    for (const MObject& input :
         {aJointBodyA, aJointBodyB, aJointType, aJointLinearMin, aJointLinearMax, aJointAngularMin,
          aJointAngularMax, aJointLinearStiffness, aJointAngularStiffness, aFrameMatrix})
    {
        stat = attributeAffects(input, aSolver);
        MMD_CHECK_MSTATUS(stat);
    }

    return MS::kSuccess;
}

// ===========================================================================
// Viewport invalidation
// ===========================================================================
// Every input can change the drawing (the solver's bodies through solverStep,
// the frame, the limits, the draw mode), so any dirty plug invalidates it.
MStatus RigidBodyJoint::setDependentsDirty(const MPlug& plug, MPlugArray& affected)
{
    MHWRender::MRenderer::setGeometryDrawDirty(thisMObject());
    return MPxLocatorNode::setDependentsDirty(plug, affected);
}

// The Evaluation Manager evaluates the node only when an input changed (each
// solver step reaches solverStep), so every evaluation invalidates it.
MStatus RigidBodyJoint::postEvaluation(const MDGContext& context,
                                       const MEvaluationNode& evaluationNode,
                                       PostEvaluationType evalType)
{
    if (context.isNormal())
        MHWRender::MRenderer::setGeometryDrawDirty(thisMObject());
    return MPxLocatorNode::postEvaluation(context, evaluationNode, evalType);
}

// ===========================================================================
// Bounds
// ===========================================================================
// Everything the joint can draw, in the shape's object space: the hub and its
// guides around the LIVE attachment on body A, plus attachment B, the body
// centres a selected joint links, and the rest-frame ghost at the transform.
// Without a solver world the hub sits at the frame.
MBoundingBox RigidBodyJoint::boundingBox() const
{
    using namespace mmd::core::joint_display;
    const MObject node = thisMObject();
    MDagPath path;
    if (MDagPath::getAPathTo(node, path) != MS::kSuccess)
        return {};
    const MMatrix worldToLocal = path.inclusiveMatrixInverse();

    MPlugArray destinations;
    MPlug(node, aSolver).connectedTo(destinations, false, true);
    const MObject solver = destinations.length() > 0 ? destinations[0].node() : MObject::kNullObj;
    const auto* solverNode =
        solver.isNull() ? nullptr
                        : dynamic_cast<RigidBodyNode*>(MFnDependencyNode(solver).userNode());

    RigidBodySimulation::JointDefinition def;
    readJointDefinition(
        node, solver.isNull() ? MMatrix::identity : RigidBodyNode::groupWorldMatrix(solver), def);
    double hub = kFallbackHubRadius;
    MMatrix attachmentA = path.inclusiveMatrix();
    MMatrix attachmentB, bodyA, bodyB;
    bool live = false;
    if (solverNode != nullptr)
    {
        def.bodyA = solverNode->bodyIndex(bodyShape(node, aJointBodyA));
        def.bodyB = solverNode->bodyIndex(bodyShape(node, aJointBodyB));
        live = solverNode->jointViewportFrames(def, bodyA, bodyB, attachmentA, attachmentB);
        if (solverNode->jointHubRadius() > 0.0)
            hub = solverNode->jointHubRadius();
        hub *= std::max(0.05, MPlug(solver, RigidBodyNode::aJointDisplayScale).asDouble());
    }

    // The symbolic guides reach about 2.25 dial radii from the hub (spring
    // zigzag, free-axis arrows, the Z dial with its notch and needles); a
    // translation region adds its physical extent on top.
    const double pad = std::max(hub, 2.25 * guideSizes(hub).radius);
    const Ranges ranges = describe(def);
    MBoundingBox box;
    for (unsigned int corner = 0; corner < 8; ++corner)
    {
        MPoint p;
        for (unsigned int axis = 0; axis < 3; ++axis)
        {
            const Range& range = ranges.linear[axis];
            const bool free = range.kind() == RangeKind::Free;
            p[axis] = ((corner >> axis) & 1U) != 0
                          ? (free ? 0.0 : std::max(range.upper, 0.0)) + pad
                          : (free ? 0.0 : std::min(range.lower, 0.0)) - pad;
        }
        box.expand(p * attachmentA * worldToLocal);
    }
    if (live)
        for (const MMatrix* m : {&attachmentB, &bodyA, &bodyB})
            box.expand(MPoint::origin * *m * worldToLocal);
    // The rest-frame ghost at the transform, for when the hub has left it.
    box.expand(MPoint(hub, hub, hub));
    box.expand(MPoint(-hub, -hub, -hub));
    return box;
}

// ===========================================================================
// Joint data reading (used by the solver)
// ===========================================================================
bool RigidBodyJoint::readJointDefinition(const MObject& node, const MMatrix& groupWorld,
                                         RigidBodySimulation::JointDefinition& out)
{
    if (node.isNull() || !node.hasFn(MFn::kDependencyNode))
        return false;
    MFnDependencyNode depFn(node);
    if (depFn.typeName() != kNodeName)
        return false;

    MStatus stat;
    const auto plug = [&depFn, &stat](const MObject& a) -> MPlug
    {
        MPlug p = depFn.findPlug(a, true, &stat);
        return p;
    };

    out.bodyA = -1; // connections, resolved by the solver (bodyShape)
    out.bodyB = -1;
    out.type = plug(aJointType).asInt();
    readDouble3(plug(aJointLinearMin), out.linearMin);
    readDouble3(plug(aJointLinearMax), out.linearMax);
    readDouble3(plug(aJointAngularMin), out.angularMin);
    readDouble3(plug(aJointAngularMax), out.angularMax);
    readDouble3(plug(aJointLinearStiffness), out.linearStiffness);
    readDouble3(plug(aJointAngularStiffness), out.angularStiffness);

    readJointFrame(node, groupWorld, out.frameT, out.frameR);
    return true;
}

MObject RigidBodyJoint::bodyShape(const MObject& joint, const MObject& end)
{
    MPlugArray sources;
    MPlug(joint, end).connectedTo(sources, true, false);
    if (sources.length() == 0)
        return MObject::kNullObj;
    const MObject shape = sources[0].node();
    return MFnDependencyNode(shape).typeId() == RigidBodyShape::kTypeId ? shape : MObject::kNullObj;
}

// The FRAME is the shape's DAG PARENT transform, measured in the
// {model}_RigidBodies group's space: the bodies' rest poses are attribute
// values in that space (the PMX model space), so the frame must be too.
// Reading WORLD instead would move every frame with the model root while the
// rest poses stay put — each pivot then ends up offset by the root move.
// Moving/rotating the guide in the viewport repositions the constraint.
bool RigidBodyJoint::readJointFrame(const MObject& node, const MMatrix& groupWorld,
                                    Double3& translate, Double3& rotate)
{
    if (!node.hasFn(MFn::kDagNode))
        return false;
    MStatus stat;
    MFnDagNode dag(node, &stat);
    const MObject transform = dag.parent(0, &stat);
    MDagPath path;
    if (!stat || transform.isNull() || !transform.hasFn(MFn::kTransform) ||
        MDagPath::getAPathTo(transform, path) != MS::kSuccess)
        return false;
    const MTransformationMatrix tm(path.inclusiveMatrix() * groupWorld.inverse());
    const MVector t = tm.getTranslation(MSpace::kTransform);
    const MEulerRotation e = tm.eulerRotation(); // radians, XYZ decomposition
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    translate = Double3(t.x, t.y, t.z);
    rotate = Double3(e.x * kRadToDeg, e.y * kRadToDeg, e.z * kRadToDeg);
    return true;
}
