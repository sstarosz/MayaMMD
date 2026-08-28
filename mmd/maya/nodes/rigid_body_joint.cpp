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
 * (translate + XYZ rotate in the RigidBodies group's space).  The joint is
 * drawn by a geometry override registered for `drawdb/geometry/
 * pmxRigidBodyJoint` (not here).
 */

#include "rigid_body_joint.hpp"

#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTransform.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>

#include "maya_utils.hpp"
#include "rigid_body_simulation.hpp"

#include <cmath>
#include <string>

using mmd::core::Double3;
using mmd::core::RigidBodySimulation;

namespace
{
void readDouble3(const MPlug& plug, Double3& out)
{
    MDataHandle h = plug.asMDataHandle();
    out.x = h.asDouble3()[0];
    out.y = h.asDouble3()[1];
    out.z = h.asDouble3()[2];
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
MObject RigidBodyJoint::aJointNameLocal;
MObject RigidBodyJoint::aJointNameUniversal;
MObject RigidBodyJoint::aJointBodyA;
MObject RigidBodyJoint::aJointBodyB;
MObject RigidBodyJoint::aJointType;
MObject RigidBodyJoint::aJointLinearMin;
MObject RigidBodyJoint::aJointLinearMax;
MObject RigidBodyJoint::aJointAngularMin;
MObject RigidBodyJoint::aJointAngularMax;
MObject RigidBodyJoint::aJointLinearSpring;
MObject RigidBodyJoint::aJointAngularSpring;
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

    // --- joint data (mirrors the old pmxRigidBodyNode joints[] children) ---
    aJointNameLocal = makeString(tAttr, "jointNameLocal", "jnml");
    aJointNameUniversal = makeString(tAttr, "jointNameUniversal", "jnmu");
    aJointBodyA = makeNumeric(nAttr, "jointBodyA", "jba", MFnNumericData::kLong, 0);
    aJointBodyB = makeNumeric(nAttr, "jointBodyB", "jbb", MFnNumericData::kLong, 0);

    // PMX joint type — enum: one field per PMX JointType (0..5).  The joint
    // FRAME (position + orientation) is NOT an attribute — it is the node's
    // DAG transform.
    {
        aJointType = eAttr.create("jointType", "jt", kSpring6Dof, &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("Spring6Dof", kSpring6Dof);
        eAttr.addField("SixDof", kSixDof);
        eAttr.addField("P2P", kP2P);
        eAttr.addField("ConeTwist", kConeTwist);
        eAttr.addField("Slider", kSlider);
        eAttr.addField("Hinge", kHinge);
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

    aJointLinearMin = makeNumeric(nAttr, "jointLinearMin", "jlmn", MFnNumericData::k3Double, 0.0);
    aJointLinearMax = makeNumeric(nAttr, "jointLinearMax", "jlmx", MFnNumericData::k3Double, 0.0);
    aJointAngularMin = makeNumeric(nAttr, "jointAngularMin", "jamn", MFnNumericData::k3Double, 0.0);
    aJointAngularMax = makeNumeric(nAttr, "jointAngularMax", "jamx", MFnNumericData::k3Double, 0.0);
    aJointLinearSpring =
        makeNumeric(nAttr, "jointLinearSpring", "jls", MFnNumericData::k3Double, 0.0);
    aJointAngularSpring =
        makeNumeric(nAttr, "jointAngularSpring", "jas", MFnNumericData::k3Double, 0.0);

    // Viewport draw mode — enum: off / wire / solid / wire+solid.
    {
        aDrawMode = eAttr.create("drawMode", "dm", kDrawWire, &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("Off", kDrawOff);
        eAttr.addField("Wire", kDrawWire);
        eAttr.addField("Solid", kDrawSolid);
        eAttr.addField("WireSolid", kDrawWireSolid);
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

    // --- attribute registration ---
    stat = addAttribute(aSolver);
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
    stat = addAttribute(aJointLinearSpring);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aJointAngularSpring);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aDrawMode);
    MMD_CHECK_MSTATUS(stat);

    return MS::kSuccess;
}

// Object-space bounding box: the linear-limit box around the frame (the node's
// transform IS the frame, so the limits are local offsets).  Degenerate axes
// get a small minimum so the joint stays framable and pickable.
MBoundingBox RigidBodyJoint::boundingBox() const
{
    constexpr double kMin = 0.05;
    MFnDependencyNode fn(thisMObject());
    const auto read = [&fn](const MObject& a) -> MPoint
    {
        MPlug p = fn.findPlug(a, true);
        return MPoint(p.asMDataHandle().asDouble3()[0], p.asMDataHandle().asDouble3()[1],
                      p.asMDataHandle().asDouble3()[2]);
    };
    MPoint lo = read(aJointLinearMin);
    MPoint hi = read(aJointLinearMax);
    for (int i = 0; i < 3; ++i)
    {
        if (std::fabs(hi[i] - lo[i]) < 1e-4)
        {
            lo[i] = -kMin;
            hi[i] = kMin;
        }
    }
    return MBoundingBox(lo, hi);
}

// ===========================================================================
// Joint data reading (used by the solver)
// ===========================================================================
bool RigidBodyJoint::readJointDefinition(const MObject& node,
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

    out.bodyA = plug(aJointBodyA).asInt();
    out.bodyB = plug(aJointBodyB).asInt();
    out.type = plug(aJointType).asInt();
    readDouble3(plug(aJointLinearMin), out.linearMin);
    readDouble3(plug(aJointLinearMax), out.linearMax);
    readDouble3(plug(aJointAngularMin), out.angularMin);
    readDouble3(plug(aJointAngularMax), out.angularMax);
    readDouble3(plug(aJointLinearSpring), out.linearSpring);
    readDouble3(plug(aJointAngularSpring), out.angularSpring);

    // The FRAME is the shape's DAG PARENT transform in WORLD space — the
    // command parents the shape under a guide transform whose world transform
    // IS the PMX world frame (the same space as the bodies' world-fixed rest
    // poses).  Reading world keeps the sim independent of the group's
    // location, and moving/rotating the guide in the viewport repositions the
    // constraint.
    if (node.hasFn(MFn::kDagNode))
    {
        MFnDagNode dag(node, &stat);
        MObject transform = dag.parent(0, &stat);
        if (stat && !transform.isNull() && transform.hasFn(MFn::kTransform))
        {
            MDagPath path;
            if (MDagPath::getAPathTo(transform, path) == MS::kSuccess)
            {
                const MTransformationMatrix tm(path.inclusiveMatrix());
                const MVector t = tm.getTranslation(MSpace::kTransform);
                const MEulerRotation e = tm.eulerRotation(); // radians, XYZ decomposition
                constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
                out.frameT = Double3(t.x, t.y, t.z);
                out.frameR = Double3(e.x * kRadToDeg, e.y * kRadToDeg, e.z * kRadToDeg);
            }
        }
    }
    return true;
}
