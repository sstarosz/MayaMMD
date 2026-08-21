/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_cmd.cpp
 *
 * Native C++ implementation of the ``pmxRigidBody`` command (create mode — the
 * default).
 *
 * ``-create`` is the single body-modification path (the PMX import loops it).
 * Create writes the body DATA, binds FOLLOW_BONE bodies to their related
 * joint through the kinematic-anchor input (bodies[i].bodyAnchorWorld), and
 * ALWAYS connects outTranslate/outRotate STRAIGHT into the related joint for
 * a dynamic body on a bone (the node computes the joint-local pose internally
 * via the bone hierarchy and derives the write-back K offset itself).
 *
 * The command's interface is minimal (see rigid_body_cmd.hpp); all the
 * implementation helpers live in the anonymous namespace below so the header
 * stays a pure interface.
 */

#include "rigid_body_cmd.hpp"
#include "pmx_rigid_body_cmd_utils.hpp"

#include <maya/MArgList.h>
#include <maya/MArgParser.h>
#include <maya/MDagPath.h>
#include <maya/MEulerRotation.h>
#include <maya/MFn.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnTransform.h>
#include <maya/MGlobal.h>
#include <maya/MItDag.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MSyntax.h>
#include <maya/MTypes.h>
#include <maya/MVector.h>

#include "maya_utils.hpp"
#include "nodes/rigid_body_node.hpp"
#include "nodes/rigid_body_shape.hpp"
#include "rigid_body_simulation.hpp"

#include <cmath>
#include <cstring>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

using mmd::core::Double3;
using mmd::core::RigidBodySimulation;

namespace
{
// ---------------------------------------------------------------------------
// Flag short names (single/compound, Maya API style).
// ---------------------------------------------------------------------------
constexpr const char* kIndexFlag = "i";
constexpr const char* kNameFlag = "n";
constexpr const char* kNameUniversalFlag = "nu";
constexpr const char* kBoneFlag = "b";
constexpr const char* kShapeFlag = "sh";
constexpr const char* kSizeFlag = "sz";
constexpr const char* kPositionFlag = "p";
constexpr const char* kRotationFlag = "rot";
constexpr const char* kMassFlag = "m";
constexpr const char* kLinearDampingFlag = "ld";
constexpr const char* kAngularDampingFlag = "ad";
constexpr const char* kFrictionFlag = "f";
constexpr const char* kRestitutionFlag = "re";
constexpr const char* kGroupFlag = "g";
constexpr const char* kMaskFlag = "msk"; // short names are limited to 3 chars
constexpr const char* kPhysicsModeFlag = "pm";

// Read a 3-double flag (e.g. -size x y z) into a Double3 with a single
// MArgList::asVector call instead of three flagArgumentDouble() lookups: the
// parser resolves the flag once, then asVector copies all three values in one
// step.  Requires the flag to be registered with three MSyntax::kDouble
// arguments (see syntaxCreator).  Returns false if the flag has no arguments.
bool flagArgumentDouble3(const MArgParser& parser, const char* flag, Double3& out)
{
    MArgList args;
    if (parser.getFlagArgumentList(flag, 0, args) != MS::kSuccess)
        return false;
    unsigned int index = 0;
    const MVector v = args.asVector(index, 3);
    out = Double3(v.x, v.y, v.z);
    return true;
}

// ASCII-safe node-name segment for the per-body guide DAG node: keeps only
// [A-Za-z0-9_].  Maya cannot store Japanese (or other non-ASCII) object
// names, and the PMX body names arrive through mayapy -> MArgParser possibly
// mangled — so the guide's Maya name is built from this segment, falling back
// to "RB_<n>" when nothing survives.
[[nodiscard]] std::string sanitizeNodeNameSegment(const MString& name)
{
    std::string out;
    const char* bytes = name.asUTF8();
    const size_t len = bytes ? std::strlen(bytes) : 0;
    for (size_t i = 0; i < len; ++i)
    {
        const unsigned char c = static_cast<unsigned char>(bytes[i]);
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')
        {
            out.push_back(static_cast<char>(c));
        }
    }
    return out;
}

// The per-body guide DAG node name: `{model}_{sanitizedBodyName}` (the PMX
// rigid-body naming convention), with the model prefix taken from the solver's
// name ("{model}_RigidBodySolver") and the body segment from the ASCII-safe
// PMX name_local (or "RB_<n>").  Maya auto-uniquifies the final name.
[[nodiscard]] MString makeShapeNodeName(const MObject& solverNode, int n, const MString& nameLocal)
{
    MFnDependencyNode fn(solverNode);
    MString prefix = fn.name();
    const int suffix = prefix.rindexW("_RigidBodySolver");
    if (suffix >= 0)
        prefix = prefix.substringW(0, suffix - 1);

    std::string segment = sanitizeNodeNameSegment(nameLocal);
    if (segment.empty())
        segment = "RB_" + std::to_string(n);
    return prefix + "_" + segment.c_str();
}

// Resolve the -bone argument to a joint dag path (or leave it empty).
MDagPath resolveBone(const MString& bone, const MDagPath& groupPath)
{
    MDagPath out;
    if (bone.length() == 0)
        return out;

    // Numeric string ⇒ PMX bone index: scan the model root's joints.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    const char* boneStr = bone.asChar();
    const std::string_view boneView(boneStr);
    const bool numeric =
        std::all_of(boneView.begin(), boneView.end(), [](char c) { return c >= '0' && c <= '9'; });
    if (numeric)
    {
        // The string was verified to contain only digits above, so the
        // conversion cannot fail; strtol reports the parse end as a guard.
        char* end = nullptr;
        const int idx = static_cast<int>(std::strtol(boneStr, &end, 10));
        if (end == boneStr)
            return out;
        MDagPath rootPath = groupPath;
        rootPath.pop(); // |modelRoot
        MItDag it;
        it.reset(rootPath, MItDag::kDepthFirst, MFn::kJoint);
        for (; !it.isDone(); it.next())
        {
            MDagPath jp;
            it.getPath(jp);
            if (mmd::maya::jointPmxBoneIndex(jp) == idx)
                return jp;
        }
        return out;
    }

    // Otherwise: a Maya joint name / path.
    MSelectionList sel;
    if (sel.add(bone) == MS::kSuccess && sel.length() > 0)
    {
        MDagPath p;
        if (sel.getDagPath(0, p) == MS::kSuccess)
            return p;
    }
    return out;
}

// ===========================================================================
// Create mode
// ===========================================================================

MStatus doCreate(const MArgParser& parser, const MObject& solverNode, int& outIndex)
{
    // ── Parse flags (safe defaults) ──
    int index = -1;
    if (parser.isFlagSet(kIndexFlag))
        index = parser.flagArgumentInt(kIndexFlag, 0);

    MString nameLocal;
    MString nameUniversal;
    MString bone;
    MString shape("sphere");
    MString physicsMode("physics");
    if (parser.isFlagSet(kNameFlag))
        nameLocal = parser.flagArgumentString(kNameFlag, 0);
    if (parser.isFlagSet(kNameUniversalFlag))
        nameUniversal = parser.flagArgumentString(kNameUniversalFlag, 0);
    if (parser.isFlagSet(kBoneFlag))
        bone = parser.flagArgumentString(kBoneFlag, 0);
    if (parser.isFlagSet(kShapeFlag))
        shape = parser.flagArgumentString(kShapeFlag, 0);
    if (parser.isFlagSet(kPhysicsModeFlag))
        physicsMode = parser.flagArgumentString(kPhysicsModeFlag, 0);

    Double3 size(0.5, 0.5, 0.5);
    Double3 pos;
    Double3 rot;
    // The user explicitly provided a 3-double flag — if the argument list
    // cannot be read, that is a malformed invocation, not a missing default.
    // Fail loudly instead of silently falling back to the defaults (which
    // would hide the typo and make debugging harder).
    if (parser.isFlagSet(kSizeFlag) && !flagArgumentDouble3(parser, kSizeFlag, size))
    {
        MGlobal::displayError("pmxRigidBody: could not read -size flag (expected three doubles)");
        return MS::kFailure;
    }
    if (parser.isFlagSet(kPositionFlag) && !flagArgumentDouble3(parser, kPositionFlag, pos))
    {
        MGlobal::displayError(
            "pmxRigidBody: could not read -position flag (expected three doubles)");
        return MS::kFailure;
    }
    if (parser.isFlagSet(kRotationFlag) && !flagArgumentDouble3(parser, kRotationFlag, rot))
    {
        MGlobal::displayError(
            "pmxRigidBody: could not read -rotation flag (expected three doubles)");
        return MS::kFailure;
    }

    double mass = 1.0;
    double linearDamping = 0.0;
    double angularDamping = 0.0;
    double friction = 0.5;
    double restitution = 0.0;
    if (parser.isFlagSet(kMassFlag))
        mass = parser.flagArgumentDouble(kMassFlag, 0);
    if (parser.isFlagSet(kLinearDampingFlag))
        linearDamping = parser.flagArgumentDouble(kLinearDampingFlag, 0);
    if (parser.isFlagSet(kAngularDampingFlag))
        angularDamping = parser.flagArgumentDouble(kAngularDampingFlag, 0);
    if (parser.isFlagSet(kFrictionFlag))
        friction = parser.flagArgumentDouble(kFrictionFlag, 0);
    if (parser.isFlagSet(kRestitutionFlag))
        restitution = parser.flagArgumentDouble(kRestitutionFlag, 0);

    int group = 0;
    int mask = 0xFFFF; // default: collide with every group
    if (parser.isFlagSet(kGroupFlag))
        group = parser.flagArgumentInt(kGroupFlag, 0);
    if (parser.isFlagSet(kMaskFlag))
        mask = parser.flagArgumentInt(kMaskFlag, 0) & 0xFFFF;
    // PMX collision groups are 0..15 (the bodyGroupId attribute is a 16-field
    // enum) — clamp so an out-of-range flag cannot write a broken enum value.
    if (group < 0)
        group = 0;
    else if (group > 15)
        group = 15;

    // PMX attenuation coefficients are 0..1 — clamp so the node's Bullet
    // damping/friction match the import path exactly.
    linearDamping = std::clamp(linearDamping, 0.0, 1.0);
    angularDamping = std::clamp(angularDamping, 0.0, 1.0);
    friction = std::clamp(friction, 0.0, 1.0);
    restitution = std::clamp(restitution, 0.0, 1.0);

    // ── Enumerated values ──
    const MString sh = shape.toLowerCase();
    RigidBodyNode::ColliderType colliderType = RigidBodyNode::kColliderSphere;
    if (sh == "box")
    {
        colliderType = RigidBodyNode::kColliderBox;
    }
    else if (sh == "capsule")
    {
        colliderType = RigidBodyNode::kColliderCapsule;
    }
    else if (sh != "sphere")
    {
        MGlobal::displayError("Unknown shape '" + shape + "' — expected sphere, box or capsule");
        return MS::kFailure;
    }

    const MString pm = physicsMode.toLowerCase();
    // The bodyPhysicsMode attribute stores mmd::core::RigidBodySimulation::PhysicsMode
    // values directly (the node casts the attribute value back to the engine
    // enum) — there is deliberately NO node-side enum.
    RigidBodySimulation::PhysicsMode physicsModeEnum = RigidBodySimulation::PhysicsMode::ePhysics;
    if (pm == "followbone")
    {
        physicsModeEnum = RigidBodySimulation::PhysicsMode::eFollowBone;
    }
    else if (pm == "physicsbone")
    {
        physicsModeEnum = RigidBodySimulation::PhysicsMode::ePhysicsBone;
    }
    else if (pm != "physics")
    {
        MGlobal::displayError("Unknown physicsMode '" + physicsMode +
                              "' — expected followBone, physics or physicsBone");
        return MS::kFailure;
    }
    const bool kinematic = (physicsModeEnum == RigidBodySimulation::PhysicsMode::eFollowBone);

    // ── Solver / group / index ──
    MFnDependencyNode fn(solverNode);
    MStatus plugStat;
    MPlug shapesPlug = fn.findPlug(RigidBodyNode::aBodyShapes, true, &plugStat);
    if (shapesPlug.isNull())
    {
        MGlobal::displayError("pmxRigidBody: node has no 'bodyShapes' array");
        return MS::kFailure;
    }
    // The next free body index is the array's CURRENT logical element count.
    // evaluateNumElements() (NOT numElements(), which returns 0 for a
    // non-cached message array) materializes the array's backing store — this
    // is what makes elementByLogicalIndex(n) below create a DISTINCT element
    // for each body instead of collapsing every append onto element 0.
    const unsigned int bodyCount = shapesPlug.evaluateNumElements();
    const int count = static_cast<int>(bodyCount);
    if (index >= 0 && index != count)
    {
        MString msg = MString("Body index ") + MString(std::to_string(index).c_str()) +
                      MString(" is not the next free index (") +
                      MString(std::to_string(count).c_str()) +
                      MString(") — append at the end or use edit mode to overwrite");
        MGlobal::displayError(msg);
        return MS::kFailure;
    }
    const int n = count;

    MDagPath nodePath;
    if (MDagPath::getAPathTo(solverNode, nodePath) != MS::kSuccess)
    {
        MGlobal::displayError("pmxRigidBody: could not resolve solver dag path");
        return MS::kFailure;
    }
    MDagPath groupPath = nodePath;
    groupPath.pop(); // |group|shape ⇒ |group (used to resolve -bone under the model root)

    // Related joint (anchor / write-back target).
    const MDagPath jointPath = resolveBone(bone, groupPath);
    MPlug jointWorldPlug;
    if (jointPath.isValid())
    {
        MFnDependencyNode jointFn(jointPath.node());
        MStatus jointPlugStat;
        jointWorldPlug =
            jointFn.findPlug("worldMatrix", true, &jointPlugStat).elementByLogicalIndex(0);
    }

    // ── Rest pose in WORLD space (MMD ⇒ Maya: Z-flip + handedness) ──
    // The Bullet world runs in world space (the node no longer depends on the
    // rigid bodies group's location), so the PMX rest pose is stored as-is.
    const Double3 worldT = mmd::maya::mmdToMayaTranslate(pos);
    const Double3 worldR = mmd::maya::mmdToMayaRotateDeg(rot);

    // ── Create the per-body guide (pmxRigidBodyShape) ──
    // One selectable guide per body: a plain TRANSFORM holding the locator
    // shape under it, parented under the rigid bodies group.  The transform
    // is positioned at the REST pose here (so the guide shows the body before
    // the first evaluation) but is then DRIVEN by the solver's
    // outGuideTranslate/outGuideRotate outputs to the body's CURRENT pose
    // each frame (see the Guide driving block below) — the collider follows
    // the animation.  All PMX-verbatim data — including the rest pose
    // (bodyRestTranslate/bodyRestRotate) — lives on the shape node; the
    // solver discovers it via solver.bodyShapes[n].
    const MString shapeName = makeShapeNodeName(solverNode, n, nameLocal);
    MObject guideParent = groupPath.node(); // non-const ref param on MFnTransform::create
    MFnTransform guideTf;
    MObject guideObj = guideTf.create(guideParent, &plugStat);
    if (guideObj.isNull())
    {
        MGlobal::displayError("pmxRigidBody: could not create the body guide transform");
        return MS::kFailure;
    }
    guideTf.setName(shapeName);
    const double kDegToRad = 3.14159265358979323846 / 180.0;
    guideTf.setTranslation(MVector(worldT.x, worldT.y, worldT.z), MSpace::kTransform);
    // No MEulerRotation overload in this SDK — go through the quaternion.
    const MEulerRotation euler(worldR.x * kDegToRad, worldR.y * kDegToRad, worldR.z * kDegToRad);
    guideTf.setRotation(euler.asQuaternion(), MSpace::kTransform);

    // The locator shape (data holder + draw target) lives under the guide
    // transform; Maya names it `{guideName}Shape` automatically.
    MFnDagNode shapeDagFn;
    MObject shapeParent = guideObj;
    MObject shapeObj =
        shapeDagFn.create(RigidBodyShape::kNodeName, MString(), shapeParent, &plugStat);
    if (shapeObj.isNull())
    {
        MGlobal::displayError("pmxRigidBody: could not create the pmxRigidBodyShape node");
        return MS::kFailure;
    }
    MFnDependencyNode shapeFn(shapeObj);

    // ── Write the body data (PMX-verbatim fields) ──
    const auto shapePlug = [&shapeFn, &plugStat](const MObject& a) -> MPlug
    { return shapeFn.findPlug(a, true, &plugStat); };
    shapePlug(RigidBodyShape::aBodyMass).setDouble(mass);
    shapePlug(RigidBodyShape::aBodyLinearDamping).setDouble(linearDamping);
    shapePlug(RigidBodyShape::aBodyAngularDamping).setDouble(angularDamping);
    shapePlug(RigidBodyShape::aBodyFriction).setDouble(friction);
    shapePlug(RigidBodyShape::aBodyRestitution).setDouble(restitution);
    shapePlug(RigidBodyShape::aBodyColliderType).setShort(colliderType);
    // PMX shape_size VERBATIM (box shape_size IS the Bullet half-extent — the
    // node derives the engine radius/extents/length by collider type via
    // mmd::core::applyShapeSize).
    mmd::maya::setPlugDouble3(shapePlug(RigidBodyShape::aBodyShapeSize), size);
    // The REST pose (world space, MMD→Maya) lives on the shape.  The guide
    // transform is driven to the body's CURRENT pose each frame (see below),
    // so the rest cannot live on the transform — the solver reads it from
    // these attributes.
    mmd::maya::setPlugDouble3(shapePlug(RigidBodyShape::aBodyRestTranslate), worldT);
    mmd::maya::setPlugDouble3(shapePlug(RigidBodyShape::aBodyRestRotate), worldR);
    shapePlug(RigidBodyShape::aBodyGroupId).setShort(static_cast<short>(group));
    for (int g = 0; g < 16; ++g)
        shapePlug(RigidBodyShape::aBodyMaskGroup.at(g)).setBool(((mask >> g) & 1) != 0);
    shapePlug(RigidBodyShape::aBodyPhysicsMode).setShort(static_cast<short>(physicsModeEnum));
    shapePlug(RigidBodyShape::aBodyNameLocal).setString(nameLocal);
    shapePlug(RigidBodyShape::aBodyNameUniversal).setString(nameUniversal);
    shapePlug(RigidBodyShape::aBodyEnabled).setBool(true);

    // Wiring: the body's related joint as a MESSAGE (shape.bodyJoint ->
    // joint.message).  The solver resolves the bone index, the write-back
    // parent and the scrub-back reset anchor from it + the joint DAG — there
    // is no per-body wiring input.  Unconnected = no related joint (a static
    // collider with no write-back).
    if (jointPath.isValid())
    {
        MStatus jointMsgStat;
        MFnDependencyNode jointFn(jointPath.node());
        const MPlug jointMsg = jointFn.findPlug("message", true, &jointMsgStat);
        if (!jointMsg.isNull())
        {
            mmd::maya::connectOrReplace(jointMsg, shapePlug(RigidBodyShape::aBodyJoint));
        }
    }
    // Register the shape on the solver: shape.solver -> solver.bodyShapes[n]
    // (PMX order — the solver pulls each body from its shape in readBodyData).
    mmd::maya::connectOrReplace(shapePlug(RigidBodyShape::aSolver),
                                shapesPlug.elementByLogicalIndex(n));

    // ── Bone binding ──
    // FOLLOW_BONE bodies are bound to their related joint through the
    // kinematic-anchor INPUT (joint.worldMatrix -> shape.bodyAnchorWorld; the
    // node applies the body<->bone rest offset K^-1, derived internally at
    // world build) — this is what makes the collider "live on the correct
    // bone".  Dynamic bodies are wired for write-back here too (see the
    // Output wiring block below — the command ALWAYS connects them).  Bodies
    // display from their rest pose — the draw override draws the collider at
    // the shape's transform.
    if (kinematic)
    {
        // The anchor world lives on the shape node (bodyAnchorWorld — a
        // matrix attribute, not an array element), so there is no separate
        // kinematic-order index to track.  The node applies the
        // body<->joint rest offset (K^-1, derived internally when the world
        // is built) on top.  The Bullet world runs in world space, so the
        // anchor is stored as-is.
        if (jointPath.isValid())
        {
            mmd::maya::connectOrReplace(jointWorldPlug,
                                        shapePlug(RigidBodyShape::aBodyAnchorWorld));
        }
        else
        {
            // No related joint: a static collider pinned at its rest pose.
            // Pin the body's DIRECT world rest pose — not a round-tripped
            // decomposition, which would drop group scale.
            const MMatrix bodyWorld = mmd::maya::matrixFromTR(worldT, worldR);
            mmd::maya::setPlugMatrixValue(shapePlug(RigidBodyShape::aBodyAnchorWorld), bodyWorld);
        }
    }

    // ── Output wiring (write-back connections) ──
    // The command ALWAYS attempts the write-back connection for a DYNAMIC
    // body with a related joint: outTranslate/outRotate are connected STRAIGHT
    // into the joint (unit-typed children — no unitConversion).  PHYSICS_BONE
    // is rotation-only.  Kinematic (followBone) bodies are anchors, never
    // driven.  A body whose parent bone has no rigid body is still connected —
    // the node cannot write a joint-local pose for it and falls back to the
    // raw solved world pose (rare in well-formed chains, where every bone in a
    // physics chain has a body).
    if (!kinematic && jointPath.isValid())
    {
        MPlug outT =
            fn.findPlug(RigidBodyNode::aOutTranslate, true, &plugStat).elementByLogicalIndex(n);
        MPlug outR =
            fn.findPlug(RigidBodyNode::aOutRotate, true, &plugStat).elementByLogicalIndex(n);
        MFnDependencyNode jointFn(jointPath.node());
        // Compound-to-compound: connecting the output ELEMENT compound to the
        // joint's translate/rotate materializes the element and wires the
        // unit-typed children DIRECTLY (kDistance/kAngle — no auto-inserted
        // unitConversion), the same connection cmds.connectAttr used to make.
        // A later body on the same bone replaces the source (last wins; all
        // bodies on a bone produce the same bone-world pose, so the joint's
        // value is unaffected).
        if (physicsModeEnum != RigidBodySimulation::PhysicsMode::ePhysicsBone)
        {
            if (mmd::maya::connectOrReplace(outT, jointFn.findPlug("translate", true, &plugStat)) !=
                MS::kSuccess)
            {
                MGlobal::displayWarning("pmxRigidBody: could not connect translate output");
            }
        }
        if (mmd::maya::connectOrReplace(outR, jointFn.findPlug("rotate", true, &plugStat)) !=
            MS::kSuccess)
        {
            MGlobal::displayWarning("pmxRigidBody: could not connect rotate output");
        }
    }

    // ── Guide driving (the guide follows the body) ──
    // EVERY body's guide transform is DRIVEN to the body's CURRENT world pose
    // each frame (solver.outGuideTranslate/outGuideRotate -> guide.translate
    // /guide.rotate).  Kinematic bodies track their bone; dynamic bodies
    // track the solved sim pose; disabled bodies sit at rest.  The rest pose
    // lives in the shape's bodyRestTranslate/Rotate attributes, so the solver
    // is unaffected by the animated transform.
    {
        MPlug outGT = fn.findPlug(RigidBodyNode::aOutGuideTranslate, true, &plugStat)
                          .elementByLogicalIndex(n);
        MPlug outGR =
            fn.findPlug(RigidBodyNode::aOutGuideRotate, true, &plugStat).elementByLogicalIndex(n);
        MFnDependencyNode guideFn(guideObj);
        if (mmd::maya::connectOrReplace(outGT, guideFn.findPlug("translate", true, &plugStat)) !=
            MS::kSuccess)
        {
            MGlobal::displayWarning("pmxRigidBody: could not connect guide translate output");
        }
        if (mmd::maya::connectOrReplace(outGR, guideFn.findPlug("rotate", true, &plugStat)) !=
            MS::kSuccess)
        {
            MGlobal::displayWarning("pmxRigidBody: could not connect guide rotate output");
        }
    }

    outIndex = n;
    return MS::kSuccess;
}

} // namespace

// ===========================================================================
// Registration
// ===========================================================================

void* RigidBodyCmd::creator()
{
    return new RigidBodyCmd();
}

MSyntax RigidBodyCmd::syntaxCreator()
{
    MSyntax syntax;
    // First positional argument: the solver node (or a model root).
    syntax.addArg(MSyntax::kString);

    syntax.addFlag(kIndexFlag, "index", MSyntax::kLong);
    syntax.addFlag(kNameFlag, "name", MSyntax::kString);
    syntax.addFlag(kNameUniversalFlag, "nameUniversal", MSyntax::kString);
    syntax.addFlag(kBoneFlag, "bone", MSyntax::kString);
    syntax.addFlag(kShapeFlag, "shape", MSyntax::kString);
    syntax.addFlag(kSizeFlag, "size", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    syntax.addFlag(kPositionFlag, "position", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    syntax.addFlag(kRotationFlag, "rotation", MSyntax::kDouble, MSyntax::kDouble, MSyntax::kDouble);
    syntax.addFlag(kMassFlag, "mass", MSyntax::kDouble);
    syntax.addFlag(kLinearDampingFlag, "linearDamping", MSyntax::kDouble);
    syntax.addFlag(kAngularDampingFlag, "angularDamping", MSyntax::kDouble);
    syntax.addFlag(kFrictionFlag, "friction", MSyntax::kDouble);
    syntax.addFlag(kRestitutionFlag, "restitution", MSyntax::kDouble);
    syntax.addFlag(kGroupFlag, "group", MSyntax::kLong);
    // Collision mask as a 16-bit "collides with" bitmask (bit i = collides
    // with group i) — the PMX non_collision_group field stored VERBATIM (MMD
    // feeds it to Bullet directly; no inversion).  Written into the
    // bodyMaskGroup0..15 boolean children.
    syntax.addFlag(kMaskFlag, "mask", MSyntax::kLong);
    syntax.addFlag(kPhysicsModeFlag, "physicsMode", MSyntax::kString);

    syntax.enableEdit(true);
    syntax.enableQuery(true);
    return syntax;
}

// ===========================================================================
// doIt
// ===========================================================================

MStatus RigidBodyCmd::doIt(const MArgList& args)
{
    MStatus stat;
    MArgParser parser(syntaxCreator(), args, &stat);
    if (!stat)
    {
        displayError("pmxRigidBody: could not parse arguments");
        return stat;
    }

    const MString target = parser.commandArgumentString(0, &stat);
    if (!stat || target.length() == 0)
    {
        displayError("pmxRigidBody: missing solver / modelRoot argument");
        return MS::kFailure;
    }

    if (parser.isQuery())
    {
        displayError("pmxRigidBody query mode is not implemented yet");
        return MS::kFailure;
    }
    if (parser.isEdit())
    {
        displayError("pmxRigidBody edit mode is not implemented yet");
        return MS::kFailure;
    }

    MObject solverNode;
    if (!mmd::maya::resolveSolver(target, solverNode))
    {
        displayError("'" + target + "' is not an pmxRigidBodyNode or a PMX model root");
        return MS::kFailure;
    }

    int newIndex = -1;
    stat = doCreate(parser, solverNode, newIndex);
    if (stat)
        setResult(newIndex);
    return stat;
}
