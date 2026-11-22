/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape.cpp
 *
 * RigidBodyShape — one selectable, movable Maya node per PMX rigid body (see
 * rigid_body_shape.hpp).
 *
 * The node is a plain data holder: every PMX body field is a storable
 * attribute (no compute()).  The simulation inputs are gathered in ONE `body`
 * compound that pmxRigidBody connects into the solver's `bodies[i]` input, so
 * an edit travels as ordinary DG dirt and this node needs no notification
 * logic.  The body's REST pose lives in the `bodyRestTranslate` /
 * `bodyRestRotate` attributes; the node's DAG parent transform is the viewport
 * GUIDE — placed at the rest pose by the builder, then DRIVEN by the solver
 * to the body's CURRENT pose each frame.  The collider is drawn by a
 * geometry override registered for `drawdb/geometry/pmxRigidBodyShape` (not
 * here).
 *
 * The body's per-frame INPUT is not stored here either: the kinematic anchor
 * lives on the solver (`pmxRigidBodyNode.anchorWorlds[]`, fed straight from
 * the related joint's worldMatrix), so this node holds authored PMX data only.
 */

#include "rigid_body_shape.hpp"

#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnData.h>
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
#include <maya/MTypes.h>
#include <maya/MVector.h>
#include <maya/MViewport2Renderer.h>

#include "maya_utils.hpp"
#include "rigid_body_simulation.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <vector>

using mmd::core::Double3;
using mmd::core::RigidBodySimulation;

namespace
{

// Create a numeric attribute with the standard storable + non-keyable flags,
// so each schema line stays a single readable declaration.
[[nodiscard]] MObject makeNumeric(MFnNumericAttribute& fn, const MString& longName,
                                  const MString& shortName, MFnNumericData::Type type, double def)
{
    MStatus stat;
    MObject attr = fn.create(longName, shortName, type, def, &stat);
    MMD_CHECK_MSTATUS(stat);
    fn.setStorable(true);
    fn.setKeyable(false);
    return attr;
}

// Same, for a PMX-name string attribute.
[[nodiscard]] MObject makeString(MFnTypedAttribute& fn, const MString& longName,
                                 const MString& shortName)
{
    MStatus stat;
    MObject attr = fn.create(longName, shortName, MFnData::kString, MObject::kNullObj, &stat);
    MMD_CHECK_MSTATUS(stat);
    fn.setStorable(true);
    fn.setKeyable(false);
    return attr;
}
} // namespace

// ===========================================================================
// Constants
// ===========================================================================
const MTypeId RigidBodyShape::kTypeId(0x0011C106); // unique Maya node type id

// ===========================================================================
// Attribute declarations
// ===========================================================================
MObject RigidBodyShape::aSolver;
MObject RigidBodyShape::aBodyJoint;
MObject RigidBodyShape::aBody;
MObject RigidBodyShape::aBodyEnabled;
MObject RigidBodyShape::aBodyNameLocal;
MObject RigidBodyShape::aBodyNameUniversal;
MObject RigidBodyShape::aBodyGroupId;
std::array<MObject, 16> RigidBodyShape::aBodyMaskGroup;
MObject RigidBodyShape::aBodyColliderType;
MObject RigidBodyShape::aBodyShapeSize;
MObject RigidBodyShape::aBodyRestTranslate;
MObject RigidBodyShape::aBodyRestRotate;
MObject RigidBodyShape::aBodyMass;
MObject RigidBodyShape::aBodyLinearDamping;
MObject RigidBodyShape::aBodyAngularDamping;
MObject RigidBodyShape::aBodyRestitution;
MObject RigidBodyShape::aBodyFriction;
MObject RigidBodyShape::aBodyPhysicsMode;
MObject RigidBodyShape::aDrawMode;

// ===========================================================================
// Node lifecycle
// ===========================================================================
RigidBodyShape::RigidBodyShape() = default;
RigidBodyShape::~RigidBodyShape() = default;

void* RigidBodyShape::creator()
{
    return new RigidBodyShape();
}

// ===========================================================================
// Attribute registration
// ===========================================================================
MStatus RigidBodyShape::initialize()
{
    MFnNumericAttribute nAttr;
    MFnTypedAttribute tAttr;
    MFnEnumAttribute eAttr;
    MFnMessageAttribute mAttr;
    MStatus stat;

    // --- solver back-pointer (message) ---
    aSolver = mAttr.create("solver", "slv", &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(false); // connections are not stored as data

    // --- related joint (message <- the PMX bone's joint) ---
    aBodyJoint = mAttr.create("bodyJoint", "bjnt", &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(false);

    // --- body data (mirrors the old pmxRigidBodyNode bodies[] children) ---
    aBodyEnabled = makeNumeric(nAttr, "bodyEnabled", "ben", MFnNumericData::kBoolean, 1.0);
    aBodyNameLocal = makeString(tAttr, "bodyNameLocal", "bnml");
    aBodyNameUniversal = makeString(tAttr, "bodyNameUniversal", "bnmu");

    // PMX collision group — enum: one field per group (0..15, "Group 0".."Group 15").
    {
        MFnEnumAttribute groupAttr;
        aBodyGroupId = groupAttr.create("bodyGroupId", "bgid", 0, &stat);
        MMD_CHECK_MSTATUS(stat);
        for (int g = 0; g < 16; ++g)
        {
            const MString name(("Group " + std::to_string(g)).c_str());
            groupAttr.addField(name, static_cast<short>(g));
        }
        groupAttr.setStorable(true);
        groupAttr.setKeyable(false);
    }

    // THE collision mask: one boolean toggle per collision group (0..15),
    // True = collides with that group (PMX non_collision_group verbatim).
    for (int g = 0; g < 16; ++g)
    {
        const MString lname(("bodyMaskGroup" + std::to_string(g)).c_str());
        const MString sname(("bmg" + std::to_string(g)).c_str());
        aBodyMaskGroup.at(g) = makeNumeric(nAttr, lname, sname, MFnNumericData::kBoolean, 1.0);
    }

    // PMX collider type — enum: box / sphere / capsule.
    {
        aBodyColliderType = eAttr.create("bodyColliderType", "bct", kColliderBox, &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("Box", kColliderBox);
        eAttr.addField("Sphere", kColliderSphere);
        eAttr.addField("Capsule", kColliderCapsule);
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }
    // PMX shape_size VERBATIM (3 doubles; box shape_size is the half-extent).
    aBodyShapeSize = makeNumeric(nAttr, "bodyShapeSize", "bss", MFnNumericData::k3Double, 1.0);
    // The REST pose (world space after the MMD→Maya flip).  Lives here, NOT
    // on the guide transform — the transform is driven to the CURRENT pose by
    // the solver each frame so the guide follows the animation.
    aBodyRestTranslate =
        makeNumeric(nAttr, "bodyRestTranslate", "brt", MFnNumericData::k3Double, 0.0);
    aBodyRestRotate = makeNumeric(nAttr, "bodyRestRotate", "brr", MFnNumericData::k3Double, 0.0);

    aBodyMass = makeNumeric(nAttr, "bodyMass", "bm", MFnNumericData::kDouble, 1.0);
    aBodyLinearDamping =
        makeNumeric(nAttr, "bodyLinearDamping", "bld", MFnNumericData::kDouble, 0.0);
    aBodyAngularDamping =
        makeNumeric(nAttr, "bodyAngularDamping", "bad", MFnNumericData::kDouble, 0.0);
    aBodyRestitution = makeNumeric(nAttr, "bodyRestitution", "bre", MFnNumericData::kDouble, 0.0);
    nAttr.setMin(0.0);
    nAttr.setSoftMax(1.0);
    aBodyFriction = makeNumeric(nAttr, "bodyFriction", "bfr", MFnNumericData::kDouble, 0.5);

    // PMX physics mode — enum: followBone / physics / physicsBone (field
    // values match RigidBodySimulation::PhysicsMode).
    {
        aBodyPhysicsMode =
            eAttr.create("bodyPhysicsMode", "bpm",
                         static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysics), &stat);
        MMD_CHECK_MSTATUS(stat);
        eAttr.addField("FollowBone",
                       static_cast<short>(RigidBodySimulation::PhysicsMode::eFollowBone));
        eAttr.addField("Physics", static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysics));
        eAttr.addField("PhysicsBone",
                       static_cast<short>(RigidBodySimulation::PhysicsMode::ePhysicsBone));
        eAttr.setStorable(true);
        eAttr.setKeyable(false);
    }

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

    // --- the simulation inputs, as ONE compound the solver consumes ---
    // pmxRigidBody connects this straight into pmxRigidBodyNode.bodies[i], so
    // every field edit is ordinary DG dirt along a real edge: the shape needs
    // no attributeAffects/message relay, and the solver reads the values from
    // its data block.  The children keep their flat long names, so
    // `shape.bodyMass` (AE template, physics panel) is unchanged.
    {
        MFnCompoundAttribute cAttr;
        aBody = cAttr.create("body", "body", &stat);
        MMD_CHECK_MSTATUS(stat);
        cAttr.setStorable(true);
        cAttr.setKeyable(false);
        cAttr.addChild(aBodyEnabled);
        cAttr.addChild(aBodyGroupId);
        for (MObject& mask : aBodyMaskGroup)
        {
            cAttr.addChild(mask);
        }
        cAttr.addChild(aBodyColliderType);
        cAttr.addChild(aBodyShapeSize);
        cAttr.addChild(aBodyRestTranslate);
        cAttr.addChild(aBodyRestRotate);
        cAttr.addChild(aBodyMass);
        cAttr.addChild(aBodyLinearDamping);
        cAttr.addChild(aBodyAngularDamping);
        cAttr.addChild(aBodyRestitution);
        cAttr.addChild(aBodyFriction);
        cAttr.addChild(aBodyPhysicsMode);
    }

    // --- attribute registration ---
    stat = addAttribute(aSolver);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyJoint);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyNameLocal);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyNameUniversal);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBody);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aDrawMode);
    MMD_CHECK_MSTATUS(stat);

    return MS::kSuccess;
}

// ===========================================================================
// Viewport invalidation — the simulation needs none
// ===========================================================================
// The simulation fields live in the `body` compound, which pmxRigidBody
// connects into the solver's `bodies[]` input: an edit is ordinary DG dirt
// along that edge, so this node has no notification logic to write.  What is
// left is the COLLIDER drawing, whose geometry depends on the shape's fields
// only (its pose comes from the guide transform, which VP2 carries without
// rebuilding anything).
MStatus RigidBodyShape::setDependentsDirty(const MPlug& plug, MPlugArray& affected)
{
    // A shape edit can change the collider's vertex data, so mark it as a
    // topology change: VP2 must re-run populateGeometry, not just updateDG().
    MHWRender::MRenderer::setGeometryDrawDirty(thisMObject());
    return MPxLocatorNode::setDependentsDirty(plug, affected);
}

// ===========================================================================
// Bounding box (rest collider, object space)
// ===========================================================================
MBoundingBox RigidBodyShape::boundingBox() const
{
    // Object-space box around the REST collider (the node's transform is the
    // rest pose, so the collider sits at the local origin).  Used for
    // viewport framing/culling; the geometry override draws the collider.
    MFnDependencyNode fn(thisMObject());
    MStatus stat;
    const MPlug sizePlug = fn.findPlug(aBodyShapeSize, true, &stat);
    const MPlug typePlug = fn.findPlug(aBodyColliderType, true, &stat);
    std::array<double, 3> half{0.5, 0.5, 0.5};
    if (!sizePlug.isNull())
    {
        const Double3 s = mmd::maya::readPlugDouble3(sizePlug);
        half[0] = s.x;
        half[1] = s.y;
        half[2] = s.z;
    }
    // For sphere/capsule the size is a radius-ish extent; for the bounding
    // box a cube is a safe superset for framing.
    double r = std::max({half[0], half[1], half[2]});
    r = std::max(r, 0.5);
    if (!typePlug.isNull())
    {
        if (typePlug.asShort() == kColliderCapsule)
        {
            r = half[0] + (half[1] * 0.5); // radius + half length
        }
        else if (typePlug.asShort() == kColliderBox)
        {
            r = std::max({half[0], half[1], half[2]});
        }
        // sphere: radius = half[0]
    }
    return {MPoint(-r, -r, -r), MPoint(r, r, r)};
}
