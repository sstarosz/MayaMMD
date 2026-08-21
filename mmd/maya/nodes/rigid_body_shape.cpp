/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_shape.cpp
 *
 * RigidBodyShape — one selectable, movable Maya node per PMX rigid body (see
 * rigid_body_shape.hpp).
 *
 * The node is a plain data holder: every PMX body field is a storable
 * attribute (no compute() — the solver reads them through its `bodyShapes[]`
 * message array).  The node's TRANSFORM is the body's rest pose — the builder
 * positions the locator at the PMX shape position/rotation — so the user can
 * select and move the guide with the standard Move tool, and the solver reads
 * the rest pose straight from the transform.  The collider is drawn by a draw
 * override registered for `drawdb/geometry/pmxRigidBodyShape` (not here).
 */

#include "rigid_body_shape.hpp"

#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnData.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
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

#include "maya_utils.hpp"
#include "rigid_body_simulation.hpp"

#include <algorithm>
#include <string>

using mmd::core::applyShapeSize;
using mmd::core::Double3;
using mmd::core::RigidBodySimulation;

namespace
{
// Map the node's persisted attribute enum (kColliderBox=1..kColliderCapsule=3)
// to the engine's PMX-aligned enum (eSphere=0..eCapsule=2).  The attribute
// values are stored in scenes, so they cannot change.
[[nodiscard]] RigidBodySimulation::ColliderType colliderToEngine(short v)
{
    switch (v)
    {
    case RigidBodyShape::kColliderBox:
        return RigidBodySimulation::ColliderType::eBox;
    case RigidBodyShape::kColliderSphere:
        return RigidBodySimulation::ColliderType::eSphere;
    default:
        return RigidBodySimulation::ColliderType::eCapsule; // kColliderCapsule
    }
}

// Read a k3Double child plug into a core Double3.  asDouble3() decays inside
// the Maya SDK header — hence the NOLINT on that single line.
void readDouble3(const MPlug& plug, Double3& out)
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const double* v = plug.asMDataHandle().asDouble3();
    out.x = v[0];
    out.y = v[1];
    out.z = v[2];
}

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
MObject RigidBodyShape::aBodyAnchorWorld;
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
    MFnMatrixAttribute matAttr;
    MStatus stat;

    // --- solver back-pointer (message) ---
    aSolver = mAttr.create("solver", "slv", &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(false); // connections are not stored as data

    // --- related joint (message <- the PMX bone's joint) ---
    aBodyJoint = mAttr.create("bodyJoint", "bjnt", &stat);
    MMD_CHECK_MSTATUS(stat);
    mAttr.setStorable(false);

    // --- kinematic-anchor input: the bone world the body follows ---
    aBodyAnchorWorld = matAttr.create("bodyAnchorWorld", "baw", MFnMatrixAttribute::kDouble, &stat);
    MMD_CHECK_MSTATUS(stat);
    matAttr.setStorable(true);
    matAttr.setKeyable(false);

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

    // --- attribute registration ---
    stat = addAttribute(aSolver);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyJoint);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyAnchorWorld);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyEnabled);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyNameLocal);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyNameUniversal);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyGroupId);
    MMD_CHECK_MSTATUS(stat);
    for (int g = 0; g < 16; ++g)
    {
        stat = addAttribute(aBodyMaskGroup.at(g));
        MMD_CHECK_MSTATUS(stat);
    }
    stat = addAttribute(aBodyColliderType);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyShapeSize);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyRestTranslate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyRestRotate);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyMass);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyLinearDamping);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyAngularDamping);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyRestitution);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyFriction);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aBodyPhysicsMode);
    MMD_CHECK_MSTATUS(stat);
    stat = addAttribute(aDrawMode);
    MMD_CHECK_MSTATUS(stat);

    return MS::kSuccess;
}

// ===========================================================================
// Body data reading (used by the solver)
// ===========================================================================
bool RigidBodyShape::readBodyDefinition(const MObject& node,
                                        RigidBodySimulation::BodyDefinition& out)
{
    if (node.isNull() || !node.hasFn(MFn::kDependencyNode))
        return false;
    MFnDependencyNode depFn(node);
    if (depFn.typeName() != kNodeName)
        return false;

    // The body's REST pose lives in the shape's bodyRestTranslate/Rotate
    // attributes (world space, PMX-verbatim after the MMD→Maya flip).  The
    // shape's TRANSFORM is the CURRENT pose, driven by the solver each frame
    // so the guide follows the animation — the rest pose is deliberately NOT
    // read from the transform (that would feed the animated pose back as
    // rest and break the write-back offset K).
    MStatus stat;
    const auto plug = [&depFn, &stat](const MObject& a) -> MPlug
    {
        MPlug p = depFn.findPlug(a, true, &stat);
        return p;
    };
    Double3 restT;
    Double3 restR;
    readDouble3(plug(aBodyRestTranslate), restT);
    readDouble3(plug(aBodyRestRotate), restR);
    out.restPos = restT;
    out.restRot = restR;

    out.mass = plug(aBodyMass).asDouble();
    out.linearDamping = plug(aBodyLinearDamping).asDouble();
    out.angularDamping = plug(aBodyAngularDamping).asDouble();
    out.friction = plug(aBodyFriction).asDouble();
    out.restitution = plug(aBodyRestitution).asDouble();
    out.colliderType = colliderToEngine(plug(aBodyColliderType).asShort());
    Double3 shapeSize;
    readDouble3(plug(aBodyShapeSize), shapeSize);
    applyShapeSize(out, shapeSize); // PMX shape_size -> engine radius/extents/length
    out.mask = 0;
    for (int g = 0; g < 16; ++g)
        if (plug(aBodyMaskGroup.at(g)).asBool())
            out.mask |= 1L << g;
    out.groupId = plug(aBodyGroupId).asShort();
    // Keep the full PMX physics mode (0/1/2) — kinematic is a derived
    // property and PHYSICS vs PHYSICS_BONE must stay distinguishable.
    out.physicsMode =
        static_cast<RigidBodySimulation::PhysicsMode>(plug(aBodyPhysicsMode).asShort());
    out.enabled = plug(aBodyEnabled).asBool();
    return true;
}

// ===========================================================================
// Bounding box (rest collider, object space)
// ===========================================================================
MBoundingBox RigidBodyShape::boundingBox() const
{
    // Object-space box around the REST collider (the node's transform is the
    // rest pose, so the collider sits at the local origin).  Used for
    // viewport framing/culling; the draw override draws the collider.
    MFnDependencyNode fn(thisMObject());
    MStatus stat;
    const MPlug sizePlug = fn.findPlug(aBodyShapeSize, true, &stat);
    const MPlug typePlug = fn.findPlug(aBodyColliderType, true, &stat);
    double half[3] = {0.5, 0.5, 0.5};
    if (!sizePlug.isNull())
    {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
        const double* s = sizePlug.asMDataHandle().asDouble3();
        half[0] = s[0];
        half[1] = s[1];
        half[2] = s[2];
    }
    // For sphere/capsule the size is a radius-ish extent; for the bounding
    // box a cube is a safe superset for framing.
    double r = std::max({half[0], half[1], half[2]});
    r = std::max(r, 0.5);
    if (!typePlug.isNull())
    {
        if (typePlug.asShort() == kColliderCapsule)
            r = half[0] + half[1] * 0.5; // radius + half length
        else if (typePlug.asShort() == kColliderBox)
            r = std::max({half[0], half[1], half[2]});
        // sphere: radius = half[0]
    }
    return MBoundingBox(MPoint(-r, -r, -r), MPoint(r, r, r));
}
