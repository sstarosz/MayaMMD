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
#include <array>
#include <mutex>

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
MObject RigidBodyJoint::aFrameMatrix;
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
    for (const MObject& input : {
             aJointBodyA,
             aJointBodyB,
             aJointLinearMin,
             aJointLinearMax,
             aJointAngularMin,
             aJointAngularMax,
             aJointLinearStiffness,
             aJointAngularStiffness,
             aFrameMatrix,
         })
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
// The two plugs that are NOT configuration are reported to the solver's input
// cache differently: solverStep follows every solver step, and aFrameMatrix
// follows the frame (which is compared by value in resolveView — a moving
// model root dirties it every frame without any edit).
MStatus RigidBodyJoint::setDependentsDirty(const MPlug& plug, MPlugArray& affected)
{
    // Two dirts are not configuration: solverStep follows every solver step,
    // and frameMatrix follows the joint frame (which a dragged or animated
    // model root moves on every frame).  The frame gets its own serial so the
    // display re-reads just the frame and keeps its cached configuration.
    if (plug.attribute() == aFrameMatrix)
    {
        mmd::maya::noteRigidBodyFrameEdit();
    }
    else if (plug.attribute() != aSolverStep)
    {
        mmd::maya::noteRigidBodyConfigEdit();
    }
    MHWRender::MRenderer::setGeometryDrawDirty(thisMObject());
    return MPxLocatorNode::setDependentsDirty(plug, affected);
}

// The Evaluation Manager evaluates the node only when an input changed (each
// solver step reaches solverStep), so every evaluation invalidates it.  This is
// what keeps the hub on body A: its pose is baked into the vertex data, and VP2
// re-populates only for an object it considers dirty.
//
// NOTE: the second argument of setGeometryDrawDirty() must stay at its default
// (true).  Passing false — "only the vertex data changed, keep the topology" —
// reads like exactly what a moving hub needs, but VP2 then stops calling
// populateGeometry altogether and the hubs freeze at their last pose.
MStatus RigidBodyJoint::postEvaluation(const MDGContext& context,
                                       const MEvaluationNode& evaluationNode,
                                       PostEvaluationType evalType)
{
    if (context.isNormal())
    {
        MHWRender::MRenderer::setGeometryDrawDirty(thisMObject());
    }
    return MPxLocatorNode::postEvaluation(context, evaluationNode, evalType);
}

// ===========================================================================
// Bounds
// ===========================================================================
// Everything the joint display needs from the owning solver, in one place:
// both RigidBodyJoint::boundingBox() and the viewport override resolve their
// solver, frames and hub radius through this.
//
// The CONFIGURATION half (the joint's own fields, its frame, the linked
// bodies, the constraint status, the hub radius) is cached per node: it
// changes only on a reported edit or when the joint frame moves, so the
// display path — which runs for every joint on every playback frame — stops
// re-reading all of it through the DG.  The LIVE half (attachment and body
// frames) is resolved every call; that is the part that moves per solver step.
RigidBodyJoint::SolverView RigidBodyJoint::resolveSolverView(const MObject& node,
                                                             bool evaluateSolver)
{
    // Reading the solverStep input evaluates the owning solver (outStep ->
    // solverStep), so the world queried below is the current one.  The
    // bounding box opts out — see the header.
    if (evaluateSolver)
    {
        const MPlug solverStep(node, aSolverStep);
        if (!solverStep.isNull())
        {
            solverStep.asDouble();
        }
    }
    if (const auto* self = dynamic_cast<RigidBodyJoint*>(MFnDependencyNode(node).userNode()))
    {
        return self->resolveView(); // solverStep already pulled above
    }
    return resolveViewUncached(node);
}

RigidBodyJoint::SolverView RigidBodyJoint::resolveView() const
{
    const MObject node = thisMObject();
    SolverView view;
    const std::scoped_lock guard(fViewCacheMutex);
    ViewCache& cache = fViewCache;

    // Both dirts are serials, not DG reads: a joint frame move (its own or the
    // model root's) only refreshes the frame, everything else refreshes the
    // whole cached configuration.
    const std::uint64_t configSerial = mmd::maya::rigidBodyEditSerial();
    const std::uint64_t frameSerial = mmd::maya::rigidBodyFrameSerial();
    const bool configStale = !cache.valid || cache.serial != configSerial;
    // A joint whose frameMatrix is not driven carries no move signal, so its
    // frame is always read (exactly like the uncached resolver).
    const bool frameStale = configStale || !cache.frameTracked || cache.frameSerial != frameSerial;

    if (configStale)
    {
        cache.solverObject = MObject::kNullObj;
        MPlugArray destinations;
        MPlug(node, aSolver).connectedTo(destinations, false, true);
        if (destinations.length() != 0)
        {
            cache.solverObject = destinations[0].node();
        }
        cache.serial = configSerial;
        cache.definition = mmd::core::RigidBodySimulation::JointDefinition{};
        cache.active = false;
        cache.hubRadius = mmd::core::joint_display::kFallbackHubRadius;
        cache.solverUser = nullptr;
        const MPlug framePlug(node, aFrameMatrix);
        cache.frameTracked = !framePlug.isNull() && framePlug.isDestination();
    }
    // Resolve the solver once per configuration: this is the per-joint hot path.
    if (cache.solverUser != nullptr && !cache.solverHandle.isAlive())
    {
        cache.solverUser = nullptr; // the solver node was deleted
        cache.solverObject = MObject::kNullObj;
    }
    if (cache.solverUser == nullptr && !cache.solverObject.isNull())
    {
        cache.solverUser =
            dynamic_cast<RigidBodyNode*>(MFnDependencyNode(cache.solverObject).userNode());
        cache.solverHandle = MObjectHandle(cache.solverObject);
    }
    view.solverObject = cache.solverObject;
    view.solver = cache.solverUser;
    if (view.solver == nullptr)
    {
        cache.valid = false;
        return view;
    }
    if (view.solver->hubRadius() <= 0.0)
    {
        // The solver has no world yet (nothing to cache: body indices and the
        // hub radius only exist once it has one).  Resolve as before.
        cache.valid = false;
        return resolveViewUncached(node);
    }

    if (configStale || frameStale)
    {
        // Both halves of the refresh need the group space; it is a solver
        // constant unless the model root moved, which moves the frame too.
        const MMatrix groupWorldInverse =
            RigidBodyNode::groupWorldMatrix(view.solverObject).inverse();
        if (configStale)
        {
            if (!readJointDefinition(node, groupWorldInverse, cache.definition))
            {
                // No readable frame (no parent transform): the limits were
                // still read — keep them, exactly like the uncached resolver.
                cache.valid = false;
                view.definition = cache.definition;
                return view;
            }
            cache.definition.bodyA = view.solver->bodyIndex(bodyShape(node, aJointBodyA));
            cache.definition.bodyB = view.solver->bodyIndex(bodyShape(node, aJointBodyB));
            cache.active = view.solver->jointStatus(cache.definition) ==
                           RigidBodySimulation::JointStatus::eActive;
            const MPlug scale(view.solverObject, RigidBodyNode::aJointDisplayScale);
            const double displayScale = scale.isNull() ? 1.0 : std::max(0.05, scale.asDouble());
            const double model = view.solver->hubRadius();
            cache.hubRadius =
                (model > 0.0 ? model : mmd::core::joint_display::kFallbackHubRadius) * displayScale;
            cache.boundsValid = false; // the corners follow the radius and the limits
        }
        else if (!readJointFrame(node, groupWorldInverse, cache.definition.frameT,
                                 cache.definition.frameR))
        {
            cache.valid = false;
            return view;
        }
        cache.frameSerial = frameSerial;
        cache.valid = true;
    }

    view.definition = cache.definition;
    view.active = cache.active;
    view.hubRadius = cache.hubRadius;
    view.serial = cache.serial;
    view.live = view.solver->jointViewportFrames(cache.definition, view.bodyAWorld, view.bodyBWorld,
                                                 view.attachmentA, view.attachmentB);
    return view;
}

// Uncached resolution (the node has no RigidBodyJoint instance behind it, which
// only happens for non-plugin nodes) — same data, no cache.
RigidBodyJoint::SolverView RigidBodyJoint::resolveViewUncached(const MObject& node)
{
    SolverView view;
    MPlugArray destinations;
    MPlug(node, aSolver).connectedTo(destinations, false, true);
    if (destinations.length() == 0)
    {
        return view;
    }
    view.solverObject = destinations[0].node();
    view.solver = dynamic_cast<RigidBodyNode*>(MFnDependencyNode(view.solverObject).userNode());
    if (view.solver == nullptr)
    {
        return view;
    }

    if (!readJointDefinition(node, RigidBodyNode::groupWorldMatrix(view.solverObject).inverse(),
                             view.definition))
    {
        return view; // no readable frame: keep the display's "inactive" defaults
    }
    view.definition.bodyA = view.solver->bodyIndex(bodyShape(node, aJointBodyA));
    view.definition.bodyB = view.solver->bodyIndex(bodyShape(node, aJointBodyB));
    view.active = view.solver->jointStatus(view.definition) ==
                  mmd::core::RigidBodySimulation::JointStatus::eActive;
    view.live = view.solver->jointViewportFrames(view.definition, view.bodyAWorld, view.bodyBWorld,
                                                 view.attachmentA, view.attachmentB);

    const MPlug scale(view.solverObject, RigidBodyNode::aJointDisplayScale);
    const double displayScale = scale.isNull() ? 1.0 : std::max(0.05, scale.asDouble());
    const double model = view.solver->hubRadius();
    view.hubRadius =
        (model > 0.0 ? model : mmd::core::joint_display::kFallbackHubRadius) * displayScale;
    return view;
}

// Everything the joint can draw, in the shape's object space: the hub and its
// guides around the LIVE attachment on body A, plus attachment B and the body
// centres a selected joint links.  Without a solver world the hub sits at the
// frame.
//
// The solver is NOT evaluated here: a bounding box is a hint Maya asks for
// far more often than the solver must run (see resolveSolverView), and one
// stale-by-an-evaluation hull still contains the drawn hub.  updateDG() is
// what evaluates the solver, and it is what dirties this drawing.
// The bounding-box corners of a joint, in the joint's OWN frame: the linear
// guide limits expanded by the marker padding.  They depend only on the limits
// and the hub radius, so they are computed once per configuration and reused.
namespace
{
std::array<MPoint, 8>
guideCornerTemplate(double hubRadius, const mmd::core::RigidBodySimulation::JointDefinition& def)
{
    using namespace mmd::core::joint_display;
    // The symbolic guides reach about 2.25 dial radii from the hub (spring
    // zigzag, free-axis arrows, the Z dial with its notch and needles); a
    // translation region adds its physical extent on top.
    const double pad = std::max(hubRadius, 2.25 * guideSizes(hubRadius).radius);
    const Ranges ranges = describe(def);
    std::array<MPoint, 8> corners;
    for (unsigned int corner = 0; corner < 8; ++corner)
    {
        MPoint p;
        for (unsigned int axis = 0; axis < 3; ++axis)
        {
            const Range& range = ranges.linear.at(axis);
            const bool upper = ((corner >> axis) & 1U) != 0U;
            const bool free = range.kind() == RangeKind::Free;
            double limit = 0.0;
            if (!free)
            {
                limit = upper ? std::max(range.upper, 0.0) : std::min(range.lower, 0.0);
            }
            p[axis] = upper ? limit + pad : limit - pad;
        }
        corners.at(corner) = p;
    }
    return corners;
}
} // namespace

MBoundingBox RigidBodyJoint::boundingBox() const
{
    const MObject node = thisMObject();
    MDagPath path;
    if (MDagPath::getAPathTo(node, path) != MS::kSuccess)
    {
        return {};
    }
    const MMatrix worldToLocal = path.inclusiveMatrixInverse();

    const SolverView view = resolveSolverView(node, /*evaluateSolver=*/false);
    // Without a live world the hub sits at the joint's own transform.
    const MMatrix& attachmentA = view.live ? view.attachmentA : path.inclusiveMatrix();

    // This runs for every DRAWN joint on every frame — Maya asks for bounds to
    // cull — so the invariant half (the guide ranges) comes from the cached
    // template and only the transform below is per frame.
    std::array<MPoint, 8> corners;
    if (!boundsTemplate(corners))
    {
        corners = guideCornerTemplate(view.hubRadius, view.definition);
    }

    // Both matrices are affine, so one product carries all eight corners into
    // the node's local space.
    const MMatrix guideToLocal = attachmentA * worldToLocal;
    MBoundingBox box;
    for (const MPoint& corner : corners)
    {
        box.expand(corner * guideToLocal);
    }
    if (view.live)
    {
        for (const MMatrix* m : {&view.attachmentB, &view.bodyAWorld, &view.bodyBWorld})
        {
            box.expand(MPoint::origin * *m * worldToLocal);
        }
    }
    return box;
}

// The joint's own-frame bounding corners, cached per configuration and filled
// on first use, so the eight points are derived once per edit instead of once
// per frame.
bool RigidBodyJoint::boundsTemplate(std::array<MPoint, 8>& corners) const
{
    const std::scoped_lock guard(fViewCacheMutex);
    ViewCache& cache = fViewCache;
    if (!cache.boundsValid)
    {
        if (!cache.valid)
        {
            return false; // no resolved configuration yet
        }
        cache.boundsCorners = guideCornerTemplate(cache.hubRadius, cache.definition);
        cache.boundsValid = true;
    }
    corners = cache.boundsCorners;
    return true;
}

// ===========================================================================
// Joint data reading (used by the solver)
// ===========================================================================
bool RigidBodyJoint::readJointDefinition(const MObject& node, const MMatrix& groupWorldInverse,
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
    readDouble3(plug(aJointLinearMin), out.linearMin);
    readDouble3(plug(aJointLinearMax), out.linearMax);
    readDouble3(plug(aJointAngularMin), out.angularMin);
    readDouble3(plug(aJointAngularMax), out.angularMax);
    readDouble3(plug(aJointLinearStiffness), out.linearStiffness);
    readDouble3(plug(aJointAngularStiffness), out.angularStiffness);

    // A joint whose frame cannot be read has no usable constraint placement;
    // fail instead of handing the solver an origin-centred joint.
    return readJointFrame(node, groupWorldInverse, out.frameT, out.frameR);
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

// The FRAME is the shape's DAG PARENT transform, measured in the
// {model}_RigidBodies group's space: the bodies' rest poses are attribute
// values in that space (the PMX model space), so the frame must be too.
// Reading WORLD instead would move every frame with the model root while the
// rest poses stay put — each pivot then ends up offset by the root move.
// Moving/rotating the guide in the viewport repositions the constraint.
// `groupWorldInverse` is passed in (not inverted here) so the solver, which
// reads every joint of a model each evaluation, inverts the group matrix once.
bool RigidBodyJoint::readJointFrame(const MObject& node, const MMatrix& groupWorldInverse,
                                    Double3& translate, Double3& rotate)
{
    if (!node.hasFn(MFn::kDagNode))
    {
        return false;
    }
    MStatus stat;
    MFnDagNode dag(node, &stat);
    const MObject transform = dag.parent(0, &stat);
    MDagPath path;
    if (!stat || transform.isNull() || !transform.hasFn(MFn::kTransform) ||
        MDagPath::getAPathTo(transform, path) != MS::kSuccess)
    {
        return false;
    }
    const MTransformationMatrix tm(path.inclusiveMatrix() * groupWorldInverse);
    const MVector t = tm.getTranslation(MSpace::kTransform);
    const MEulerRotation e = tm.eulerRotation(); // radians, XYZ decomposition
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    translate = Double3(t.x, t.y, t.z);
    rotate = Double3(e.x * kRadToDeg, e.y * kRadToDeg, e.z * kRadToDeg);
    return true;
}
