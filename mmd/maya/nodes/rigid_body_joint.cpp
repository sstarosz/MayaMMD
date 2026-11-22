/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_joint.cpp
 *
 * RigidBodyJoint — one selectable, movable Maya node per PMX rigid-body joint
 * (see rigid_body_joint.hpp).
 *
 * The node is a plain data holder: every PMX joint field — the FRAME included,
 * as `jointRestTranslate` / `jointRestRotate` — is a storable attribute in the
 * `joint` compound that the solver reads from its `joints[]` input (no
 * compute()).  The DAG transform above the node is a solver-driven GUIDE to
 * the joint's live attachment on body A, so the node's pivot sits on the drawn
 * hub and the glyphs are local geometry under that transform; edit the
 * rest-frame attributes to move the constraint.  The joint is drawn by
 * RigidBodyJointGeometryOverride (not here); the node only tells VP2 when that
 * drawing is out of date. Bounds arrive as solver-driven inputs; all
 * rendering decisions and caches belong to the override.
 */

#include "rigid_body_joint.hpp"

#include <maya/MDGContext.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>

#include "joint_display.hpp"
#include "maya_utils.hpp"
#include "rigid_body_joint_geometry_override.hpp"
#include "rigid_body_shape.hpp"
#include "rigid_body_simulation.hpp"

#include <maya/MPlugArray.h>

#include <array>

using mmd::core::Double3;
using mmd::core::RigidBodySimulation;

namespace
{
void readDouble3(const MPlug& plug, Double3& out)
{
    // A missing attribute (never for a pmxRigidBodyJoint, whose type is checked
    // first) keeps the JointDefinition default: readPlugDouble3 would call
    // asMDataHandle() on a null plug.
    if (plug.isNull())
    {
        return;
    }
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
    MObject a = fn.create(longName, shortName, axes.at(0), axes.at(1), axes.at(2), &stat);
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
    MObject a = fn.create(longName, shortName, MFnData::kString, MObject::kNullObj, &stat);
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
MObject RigidBodyJoint::aJointDisplay;
MObject RigidBodyJoint::aJointHubRadius;
MObject RigidBodyJoint::aJointActive;
MObject RigidBodyJoint::aJointBoundsMin;
MObject RigidBodyJoint::aJointBoundsMax;
MObject RigidBodyJoint::aJoint;
MObject RigidBodyJoint::aJointRestTranslate;
MObject RigidBodyJoint::aJointRestRotate;
MObject RigidBodyJoint::aJointNameLocal;
MObject RigidBodyJoint::aJointNameUniversal;
MObject RigidBodyJoint::aJointBodyA;
MObject RigidBodyJoint::aJointBodyB;
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

    // Solver-derived display data is not authored PMX data. Configuration
    // outputs drive this compound; time only drives the guide and solverStep.
    aJointHubRadius = makeNumeric(nAttr, "jointHubRadius", "jhr", MFnNumericData::kDouble,
                                  mmd::core::joint_display::kFallbackHubRadius);
    nAttr.setStorable(false);
    aJointActive = makeNumeric(nAttr, "jointActive", "jac", MFnNumericData::kBoolean, 0.0);
    nAttr.setStorable(false);
    aJointBoundsMin = makeNumeric(nAttr, "jointBoundsMin", "jbmn", MFnNumericData::k3Double,
                                  -mmd::core::joint_display::kFallbackHubRadius);
    nAttr.setStorable(false);
    aJointBoundsMax = makeNumeric(nAttr, "jointBoundsMax", "jbmx", MFnNumericData::k3Double,
                                  mmd::core::joint_display::kFallbackHubRadius);
    nAttr.setStorable(false);
    {
        MFnCompoundAttribute cAttr;
        aJointDisplay = cAttr.create("jointDisplay", "jdsp", &stat);
        MMD_CHECK_MSTATUS(stat);
        cAttr.setStorable(false);
        cAttr.setHidden(true);
        cAttr.addChild(aJointHubRadius);
        cAttr.addChild(aJointActive);
        cAttr.addChild(aJointBoundsMin);
        cAttr.addChild(aJointBoundsMax);
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

    // The FRAME is authored data here, in the bodies' rest space (world at
    // import), exactly like bodyRestTranslate/bodyRestRotate: the transform
    // above the node is driven by the solver, so it cannot hold the frame.
    aJointRestTranslate =
        makeNumeric(nAttr, "jointRestTranslate", "jrt", MFnNumericData::k3Double, 0.0);
    aJointRestRotate = makeNumeric(nAttr, "jointRestRotate", "jrr", MFnNumericData::k3Double, 0.0);
    aJointLinearMin = makeNumeric(nAttr, "jointLinearMin", "jlmn", MFnNumericData::k3Double, 0.0);
    aJointLinearMax = makeNumeric(nAttr, "jointLinearMax", "jlmx", MFnNumericData::k3Double, 0.0);
    aJointAngularMin = makeAngle3(nAttr, "jointAngularMin", "jamn");
    aJointAngularMax = makeAngle3(nAttr, "jointAngularMax", "jamx");
    aJointLinearStiffness =
        makeNumeric(nAttr, "jointLinearStiffness", "jlsk", MFnNumericData::k3Double, 0.0);
    aJointAngularStiffness =
        makeNumeric(nAttr, "jointAngularStiffness", "jask", MFnNumericData::k3Double, 0.0);

    // --- the simulation inputs, as ONE compound the solver consumes ---
    // pmxRigidBodyConstraint connects this straight into
    // pmxRigidBodyNode.joints[i], so every field edit (the frame included) is
    // ordinary DG dirt along a real edge: this node needs no
    // attributeAffects/message relay, and the solver reads the values from its
    // data block.  The children keep their flat long names, so
    // `joint.jointLinearMin` (AE template, physics panel) is unchanged.
    {
        MFnCompoundAttribute cAttr;
        aJoint = cAttr.create("joint", "jnt", &stat);
        MMD_CHECK_MSTATUS(stat);
        cAttr.setStorable(true);
        cAttr.setKeyable(false);
        cAttr.addChild(aJointRestTranslate);
        cAttr.addChild(aJointRestRotate);
        cAttr.addChild(aJointLinearMin);
        cAttr.addChild(aJointLinearMax);
        cAttr.addChild(aJointAngularMin);
        cAttr.addChild(aJointAngularMax);
        cAttr.addChild(aJointLinearStiffness);
        cAttr.addChild(aJointAngularStiffness);
    }

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
    stat = addAttribute(aJointDisplay);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointNameLocal);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointNameUniversal);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointBodyA);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointBodyB);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJoint);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aDrawMode);
    MMD_CHECK_MSTATUS(stat);

    // The simulation values reach the solver through the `joint` compound
    // connection (native dirt), so only the BODY IDENTITY needs a relay: a
    // re-connected body must dirty the solver's jointShapes[i] so it re-reads
    // the body indices.
    for (const MObject& input : {aJointBodyA, aJointBodyB})
    {
        stat = attributeAffects(input, aSolver);
        MMD_CHECK_MSTATUS(stat);
    }

    return MS::kSuccess;
}

// ===========================================================================
// Viewport invalidation
// ===========================================================================
MStatus RigidBodyJoint::setDependentsDirty(const MPlug& plug, MPlugArray& affected)
{
    RigidBodyJointGeometryOverride::inputChanged(thisMObject(), plug);
    return MPxLocatorNode::setDependentsDirty(plug, affected);
}

MStatus RigidBodyJoint::preEvaluation(const MDGContext& context,
                                      const MEvaluationNode& evaluationNode)
{
    if (context.isNormal())
    {
        RigidBodyJointGeometryOverride::inputsEvaluated(thisMObject(), evaluationNode);
    }
    return MPxLocatorNode::preEvaluation(context, evaluationNode);
}

MStatus RigidBodyJoint::postEvaluation(const MDGContext& context,
                                       const MEvaluationNode& evaluationNode,
                                       PostEvaluationType evalType)
{
    if (context.isNormal())
    {
        RigidBodyJointGeometryOverride::poseChanged(thisMObject());
    }
    return MPxLocatorNode::postEvaluation(context, evaluationNode, evalType);
}

MBoundingBox RigidBodyJoint::boundingBox() const
{
    // These are this node's configuration-only inputs. No solver lookup,
    // pose query, selection state or mutable display cache is needed.
    const Double3 low = mmd::maya::readPlugDouble3(MPlug(thisMObject(), aJointBoundsMin));
    const Double3 high = mmd::maya::readPlugDouble3(MPlug(thisMObject(), aJointBoundsMax));
    return {MPoint(low.x, low.y, low.z), MPoint(high.x, high.y, high.z)};
}

// ===========================================================================
// Joint data reading (used by the display path)
// ===========================================================================
bool RigidBodyJoint::readJointDefinition(const MObject& node,
                                         RigidBodySimulation::JointDefinition& out)
{
    if (node.isNull() || !node.hasFn(MFn::kDependencyNode))
    {
        return false;
    }
    MFnDependencyNode depFn(node);
    if (depFn.typeId() != kTypeId)
    {
        return false;
    }

    MStatus stat;
    const auto plug = [&depFn, &stat](const MObject& a) -> MPlug
    {
        MPlug p = depFn.findPlug(a, true, &stat);
        return p;
    };

    out.bodyA = -1; // connections, resolved by the solver (bodyShape)
    out.bodyB = -1;
    readDouble3(plug(aJointRestTranslate), out.frameT);
    readDouble3(plug(aJointRestRotate), out.frameR);
    readDouble3(plug(aJointLinearMin), out.linearMin);
    readDouble3(plug(aJointLinearMax), out.linearMax);
    readDouble3(plug(aJointAngularMin), out.angularMin);
    readDouble3(plug(aJointAngularMax), out.angularMax);
    readDouble3(plug(aJointLinearStiffness), out.linearStiffness);
    readDouble3(plug(aJointAngularStiffness), out.angularStiffness);
    return true;
}

MObject RigidBodyJoint::bodyShape(const MObject& joint, const MObject& end)
{
    MPlugArray sources;
    MPlug(joint, end).connectedTo(sources, true, false);
    if (sources.length() == 0)
    {
        return MObject::kNullObj;
    }
    const MObject shape = sources[0].node();
    return MFnDependencyNode(shape).typeId() == RigidBodyShape::kTypeId ? shape : MObject::kNullObj;
}
