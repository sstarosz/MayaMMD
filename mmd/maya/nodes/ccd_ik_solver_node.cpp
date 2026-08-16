/*
 * ccd_ik_solver_node.cpp
 *
 * C++ CCD IK Solver implementation — ported from Python ccd_ik_solver_node.py.
 */

#include "ccd_ik_solver_node.h"

#include "maya_utils.hpp"

#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MDagPath.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MEulerRotation.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnIkEffector.h>
#include <maya/MFnIkHandle.h>
#include <maya/MFnIkJoint.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MIkHandleGroup.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPoint.h>
#include <maya/MQuaternion.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

// ===========================================================================
// Static member definitions
// ===========================================================================
const MTypeId CCDIKSolverNode::kTypeId(0x00080052);

MObject CCDIKSolverNode::aLimitRadian;
MObject CCDIKSolverNode::aIkLinkLimits;
MObject CCDIKSolverNode::aIkLinkBoneIndex;
MObject CCDIKSolverNode::aHasIkLinkLimits;
MObject CCDIKSolverNode::aIkLinkLimitMin;
MObject CCDIKSolverNode::aIkLinkLimitMax;

// ===========================================================================
// Fold-boost tuning
//
// Pure angular CCD makes only glacial progress when the target is nearly in
// line with the chain (the classic "straight leg" singularity): the alignment
// angle per iteration is tiny even when the positional error is large.  In
// that regime we drive each constrained joint's clamped state toward the fold
// angle that places the effector at the target's radius from the chain root,
// so the chain can start folding and CCD takes over once the geometry opens up.
// ===========================================================================
namespace
{
// Alignment angles (radians) below this are treated as near-degenerate.
constexpr double kFoldBoostThreshold = 0.035; // ~2.0 deg
// Only boost while the positional error exceeds this (Maya units).
constexpr double kFoldMinDistErr = 0.03;
// Ease this fraction of the way toward the target fold angle per iteration.
constexpr double kFoldBoostFraction = 0.25;
} // namespace

// ===========================================================================
// Constructor / Destructor
// ===========================================================================
CCDIKSolverNode::CCDIKSolverNode() {}
CCDIKSolverNode::~CCDIKSolverNode() {}

// ===========================================================================
// MPxIkSolverNode overrides
// ===========================================================================
MString CCDIKSolverNode::solverTypeName() const
{
    return MString(kNodeName);
}

MStatus CCDIKSolverNode::doSolve()
{
    const double eps = 1.0e-10;
    const double tolerance = 1.0e-4;

    // ── Read attributes ─────────────────────────────────────────────
    MObject thisObj = thisMObject();
    MFnDependencyNode fnDep(thisObj);

    MPlug maxItersPlug = fnDep.findPlug("maxIterations", true);
    int const maxIters = maxItersPlug.asInt();

    MPlug limitPlug = fnDep.findPlug(aLimitRadian, true);
    double const limitRadian = limitPlug.asDouble();
    double const maxAngleRad = (limitRadian > 0.0) ? limitRadian : M_PI / 2.0;

    // ── Get handle group ─────────────────────────────────────────────
    MIkHandleGroup* handleGroup = this->handleGroup();
    if (handleGroup == nullptr)
        return MS::kSuccess;

    MObject handle = handleGroup->handle(0);
    MDagPath handlePath;
    MStatus handlePathStat = MDagPath::getAPathTo(handle, handlePath);
    if (!handlePathStat)
        return handlePathStat;
    MFnIkHandle fnHandle(handlePath);

    MDagPath endEffector;
    fnHandle.getEffector(endEffector);
    MFnIkEffector effectorFn(endEffector);

    MDagPath startJoint;
    fnHandle.getStartJoint(startJoint);
    MString startJointName = startJoint.fullPathName();

    // ── Chain root position (used by the fold boost) ─────────────────
    MFnIkJoint startJointFn(startJoint);
    MPoint startJointPos = startJointFn.rotatePivot(MSpace::kWorld);

    // ── Target position ──────────────────────────────────────────────
    MPoint handlePos = fnHandle.rotatePivot(MSpace::kWorld);

    // ── Build joint chain (effector → root) ──────────────────────────
    std::vector<MDagPath> jointPaths;
    {
        MDagPath temp(endEffector);
        temp.pop(); // go from effector node to its parent joint
        while (true)
        {
            jointPaths.push_back(MDagPath(temp));
            if (temp.fullPathName() == startJointName)
                break;
            temp.pop();
        }
    }

    // ── Read link limits map ─────────────────────────────────────────
    std::unordered_map<int, LinkLimit> limitsMap = readLinkLimitsMap(fnDep);

    // ── Cache bone indices ───────────────────────────────────────────
    std::unordered_map<std::string, int> boneIdxCache;
    for (auto& jp : jointPaths)
    {
        int const bi = mmd::maya::jointPmxBoneIndex(jp);
        boneIdxCache[jp.partialPathName().asChar()] = bi;
    }

    // ── Per-joint plane angle state ──────────────────────────────────
    std::unordered_map<std::string, double> planeAngleState;

    // Initialise the accumulated-angle state from each constrained joint's
    // CURRENT rotation.  Without this, repeated doSolve() calls (e.g. while
    // dragging an IK handle, which triggers a fresh solve per move) assume
    // every joint starts at rest, so the clamped state and the joint's real
    // rotation drift apart after the first solve — causing the effector to
    // overshoot the target and, as the drift grows, the hinge to flip and
    // bend in the wrong direction.
    for (auto& jp : jointPaths)
    {
        int bi = -1;
        auto itBI = boneIdxCache.find(jp.partialPathName().asChar());
        if (itBI != boneIdxCache.end())
            bi = itBI->second;
        auto limitsIt = limitsMap.find(bi);
        const LinkLimit* limits = (limitsIt != limitsMap.end()) ? &limitsIt->second : nullptr;
        int const limitAxis = getSingleAxisIndex(limits);
        if (limitAxis >= 0)
        {
            MFnIkJoint jfn(jp);
            planeAngleState[jp.partialPathName().asChar()] = getCurrentAxisAngle(jfn, limitAxis);
        }
    }

    // ── Best-distance tracking ───────────────────────────────────────
    double bestDistance = std::numeric_limits<double>::infinity();
    int stallCount = 0;

    // ── CCD Loop ─────────────────────────────────────────────────────
    for (int it = 0; it < maxIters; ++it)
    {
        MPoint effectorPos = effectorFn.rotatePivot(MSpace::kWorld);
        MVector toTarget = handlePos - effectorPos;
        double const currDist = toTarget.length();
        if (currDist < tolerance)
            break;

        for (auto& jointPath : jointPaths)
        {
            MFnIkJoint jfn(jointPath);
            MPoint jpos = jfn.rotatePivot(MSpace::kWorld);

            MVector toEff = effectorPos - jpos;
            MVector toTgt = handlePos - jpos;

            double const lenEff = toEff.length();
            double const lenTgt = toTgt.length();
            if (lenEff < eps || lenTgt < eps)
                continue;

            MVector dEff = toEff / lenEff;
            MVector dTgt = toTgt / lenTgt;

            double const dotEffTgt = dEff * dTgt;
            if (dotEffTgt > 1.0 - eps)
                continue;

            // ── Per-joint axis constraint ────────────────────────────
            int bi = -1;
            auto itBI = boneIdxCache.find(jointPath.partialPathName().asChar());
            if (itBI != boneIdxCache.end())
                bi = itBI->second;

            auto limitsIt = limitsMap.find(bi);
            const LinkLimit* limits = (limitsIt != limitsMap.end()) ? &limitsIt->second : nullptr;
            int const limitAxis = getSingleAxisIndex(limits);

            if (limitAxis >= 0)
            {
                // LOCAL-SPACE plane constraint
                MQuaternion worldQuat;
                jfn.getRotation(worldQuat, MSpace::kWorld);
                MQuaternion invQuat = worldQuat.conjugate();

                MVector dEffLocal = dEff.rotateBy(invQuat);
                MVector dTgtLocal = dTgt.rotateBy(invQuat);

                double dotLocal = dEffLocal * dTgtLocal;
                dotLocal = std::max(-1.0, std::min(1.0, dotLocal));
                double angle = acos(dotLocal);
                angle = std::min(angle, maxAngleRad);
                if (angle < 1.0e-6)
                    continue;

                // Constraint axis in local space
                MVector laLocal;
                if (limitAxis == 0)
                    laLocal = MVector(1.0, 0.0, 0.0);
                else if (limitAxis == 1)
                    laLocal = MVector(0.0, 1.0, 0.0);
                else
                    laLocal = MVector(0.0, 0.0, 1.0);

                // Test +angle and -angle
                MQuaternion quatPos(angle, laLocal);
                MQuaternion quatNeg(-angle, laLocal);
                MVector testPos = dEffLocal.rotateBy(quatPos);
                MVector testNeg = dEffLocal.rotateBy(quatNeg);

                double signedAngle = (testPos * dTgtLocal > testNeg * dTgtLocal) ? angle : -angle;

                // Straight-leg stability bias
                if (angle < M_PI / 180.0 && limits != nullptr)
                {
                    double const loMmd = limits->lo[limitAxis];
                    double const hiMmd = limits->hi[limitAxis];
                    double const loMaya = -hiMmd;
                    double const hiMaya = -loMmd;
                    double const half = (loMaya + hiMaya) * 0.5;
                    if (fabs(half - signedAngle) > fabs(half + signedAngle))
                        signedAngle = -signedAngle;
                }

                // ── Fold boost (near "straight leg" singularity) ─────
                // When the alignment angle is tiny but the effector is still
                // far from the target, the target is nearly in line with the
                // chain and pure angular CCD only inches forward each pass.
                // Drive the clamped state toward the fold angle that places
                // the effector at the target's radius from the chain root so
                // the chain can start folding; normal CCD takes over once the
                // geometry opens up (angle grows past the threshold).
                std::string const jpName = jointPath.partialPathName().asChar();
                double const prevAngle = planeAngleState[jpName];

                if (angle < kFoldBoostThreshold && limits != nullptr)
                {
                    MVector distErrVec = effectorPos - handlePos;
                    if (distErrVec.length() > kFoldMinDistErr)
                    {
                        double const loMmd = limits->lo[limitAxis];
                        double const hiMmd = limits->hi[limitAxis];
                        double const loMaya = -hiMmd;
                        double const hiMaya = -loMmd;
                        double tReq = computeFoldAngle(startJointPos, jpos, effectorPos, handlePos);
                        tReq = std::max(loMaya, std::min(hiMaya, tReq));
                        signedAngle = (tReq - prevAngle) * kFoldBoostFraction;
                    }
                }

                // Accumulate angle
                double newAngle = prevAngle + signedAngle;

                // Resolve sign ambiguity on first iteration
                if (it == 0 && prevAngle == 0.0 && limits != nullptr)
                {
                    double const loMmd = limits->lo[limitAxis];
                    double const hiMmd = limits->hi[limitAxis];
                    double const loMaya = -hiMmd;
                    double const hiMaya = -loMmd;
                    if (newAngle < loMaya || newAngle > hiMaya)
                    {
                        if (-newAngle > loMaya && -newAngle < hiMaya)
                        {
                            newAngle = -newAngle;
                        }
                        else
                        {
                            double const half = (loMaya + hiMaya) * 0.5;
                            if (fabs(half - newAngle) > fabs(half + newAngle))
                                newAngle = -newAngle;
                        }
                    }
                }

                // Clamp to limits
                if (limits != nullptr)
                {
                    double const loMmd = limits->lo[limitAxis];
                    double const hiMmd = limits->hi[limitAxis];
                    double const loMaya = -hiMmd;
                    double const hiMaya = -loMmd;
                    newAngle = std::max(loMaya, std::min(hiMaya, newAngle));
                }

                double const deltaAngle = newAngle - prevAngle;
                planeAngleState[jpName] = newAngle;

                if (fabs(deltaAngle) < 1.0e-8)
                    continue;

                // Apply delta in LOCAL space via rotateBy(kTransform)
                MQuaternion qDeltaLocal(deltaAngle, laLocal);
                jfn.rotateBy(qDeltaLocal, MSpace::kTransform);
            }
            else
            {
                // Standard world-space CCD (no constraint)
                MVector axis = dEff ^ dTgt;
                double const axisLen = axis.length();
                if (axisLen < eps)
                {
                    // Antiparallel case
                    if (fabs(dEff.x) < 0.9)
                        axis = MVector(1.0, 0.0, 0.0) ^ dEff;
                    else
                        axis = MVector(0.0, 1.0, 0.0) ^ dEff;
                    axis.normalize();
                }
                else
                {
                    axis.normalize();
                }
                double const angle = dEff.angle(dTgt);
                double const clampedAngle = std::min(angle, maxAngleRad);
                MQuaternion quat(clampedAngle, axis);
                jfn.rotateBy(quat, MSpace::kWorld);
            }

            // Update effector position after joint rotation
            effectorPos = effectorFn.rotatePivot(MSpace::kWorld);
        }

        // ── Best-distance tracking ───────────────────────────────────
        MVector postDistVec = effectorPos - handlePos;
        double const postDist = postDistVec.length();
        if (postDist < bestDistance - 1.0e-10)
        {
            bestDistance = postDist;
            stallCount = 0;
            if (postDist <= tolerance)
                break;
        }
        else
        {
            stallCount++;
            if (stallCount >= 2)
                break;
        }
    }
    return MS::kSuccess;
}

// ===========================================================================
// Static helpers
// ===========================================================================
std::unordered_map<int, CCDIKSolverNode::LinkLimit>
CCDIKSolverNode::readLinkLimitsMap(MFnDependencyNode& fnDep)
{
    std::unordered_map<int, LinkLimit> result;
    try
    {
        MPlug limitsPlug = fnDep.findPlug(aIkLinkLimits, true);
        const unsigned int numElems = limitsPlug.numElements();
        for (unsigned int i = 0; i < numElems; ++i)
        {
            try
            {
                MPlug elem = limitsPlug.elementByPhysicalIndex(i);
                int const boneIdx = elem.child(0).asInt();
                bool const enabled = elem.child(1).asBool();
                if (!enabled)
                    continue;

                MPlug minPlug = elem.child(2);
                MPlug maxPlug = elem.child(3);

                LinkLimit ll{};
                ll.lo[0] = minPlug.child(0).asDouble();
                ll.lo[1] = minPlug.child(1).asDouble();
                ll.lo[2] = minPlug.child(2).asDouble();
                ll.hi[0] = maxPlug.child(0).asDouble();
                ll.hi[1] = maxPlug.child(1).asDouble();
                ll.hi[2] = maxPlug.child(2).asDouble();

                result[boneIdx] = ll;
            }
            catch (...)
            {
                continue;
            }
        }
    }
    catch (...)
    {
        // No link limits attribute — this is a normal state, not an error.
        return result;
    }
    return result;
}

int CCDIKSolverNode::getSingleAxisIndex(const LinkLimit* limits)
{
    if (limits == nullptr)
        return -1;

    int singleAxis = -1;
    int nzCount = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (limits->lo[i] != 0.0 || limits->hi[i] != 0.0)
        {
            singleAxis = i;
            ++nzCount;
        }
    }

    if (nzCount == 1)
    {
        int const ax = singleAxis;
        // Verify other axes are zero
        for (int j = 0; j < 3; ++j)
        {
            if (j != ax)
            {
                if (limits->lo[j] != 0.0 || limits->hi[j] != 0.0)
                    return -1;
            }
        }
        return ax;
    }
    return -1;
}

double CCDIKSolverNode::getCurrentAxisAngle(MFnIkJoint& jfn, int axis)
{
    // The solver accumulates rotation into the joint's local rotation via
    // rotateBy(..., kTransform), so the joint's current rotation around the
    // constraint axis IS the accumulated angle.  PMX plain bones keep
    // jointOrient = 0, so the local (parent) frame matches the constraint
    // frame used by the solver.
    try
    {
        MQuaternion localQuat;
        jfn.getRotation(localQuat, MSpace::kTransform);

        // Extract the signed angle around the constraint axis.  Because the
        // constrained joint only ever rotates around its constraint axis,
        // its local rotation axis is (anti-)parallel to that axis, and
        // getAxisAngle() gives the rotation angle (radians).
        MVector quatAxis;
        double quatAngle = 0.0;
        localQuat.getAxisAngle(quatAxis, quatAngle);

        MVector axisDir;
        if (axis == 0)
            axisDir = MVector(1.0, 0.0, 0.0);
        else if (axis == 1)
            axisDir = MVector(0.0, 1.0, 0.0);
        else
            axisDir = MVector(0.0, 0.0, 1.0);

        double ang = (quatAxis * axisDir >= 0.0) ? quatAngle : -quatAngle;
        // Normalise to [-pi, pi] so the state stays bounded.
        while (ang > M_PI)
            ang -= 2.0 * M_PI;
        while (ang < -M_PI)
            ang += 2.0 * M_PI;
        return ang;
    }
    catch (...)
    {
        return 0.0;
    }
}

double CCDIKSolverNode::computeFoldAngle(const MPoint& root, const MPoint& joint,
                                         const MPoint& effector, const MPoint& target)
{
    // Bend angle (0 = straight) that places the effector at the same radius
    // from the chain root as the target, via the law of cosines on the
    // root-joint-effector triangle.  Used only as a convergence boost inside
    // the near-"straight leg" regime; the angular CCD still resolves the
    // actual direction the effector must swing.
    const double eps = 1.0e-10;
    MVector vJoint = joint - root;
    MVector vShin = effector - joint;
    double const lf = vJoint.length();
    double const ls = vShin.length();
    if (lf < eps || ls < eps)
        return 0.0;
    MVector vTarget = target - root;
    double const dt = vTarget.length();
    double c = ((dt * dt) - (lf * lf) - (ls * ls)) / (2.0 * lf * ls);
    c = std::max(-1.0, std::min(1.0, c));
    return acos(c);
}

// ===========================================================================
// Creator / Initializer
// ===========================================================================
void* CCDIKSolverNode::creator()
{
    return new CCDIKSolverNode();
}

MStatus CCDIKSolverNode::initialize()
{
    try
    {
        MFnNumericAttribute nAttr;

        // aLimitRadian
        aLimitRadian = nAttr.create("limitRadian", "lr", MFnNumericData::kDouble, 0.0);
        nAttr.setMin(0.0);
        nAttr.setKeyable(true);
        nAttr.setStorable(true);
        addAttribute(aLimitRadian);

        // aHasIkLinkLimits
        aHasIkLinkLimits = nAttr.create("hasIkLinkLimits", "hill", MFnNumericData::kBoolean, 0.0);
        nAttr.setKeyable(true);
        nAttr.setStorable(true);
        nAttr.setReadable(true);

        // aIkLinkLimitMin (3 double)
        aIkLinkLimitMin = nAttr.create("ikLinkLimitMin", "illmin", MFnNumericData::k3Double, 0.0);
        nAttr.setKeyable(true);
        nAttr.setStorable(true);
        nAttr.setReadable(true);

        // aIkLinkLimitMax (3 double)
        aIkLinkLimitMax = nAttr.create("ikLinkLimitMax", "illmax", MFnNumericData::k3Double, 0.0);
        nAttr.setKeyable(true);
        nAttr.setStorable(true);
        nAttr.setReadable(true);

        // aIkLinkBoneIndex
        aIkLinkBoneIndex = nAttr.create("ikLinkBoneIndex", "illidx", MFnNumericData::kInt, -1);
        nAttr.setKeyable(true);
        nAttr.setStorable(true);
        nAttr.setReadable(true);

        // aIkLinkLimits (compound, array)
        MFnCompoundAttribute cAttr;
        aIkLinkLimits = cAttr.create("ikLinkLimits", "illimits");
        cAttr.setArray(true);
        cAttr.setKeyable(true);
        cAttr.setStorable(true);
        cAttr.setReadable(true);
        cAttr.setUsesArrayDataBuilder(true);
        cAttr.addChild(aIkLinkBoneIndex);
        cAttr.addChild(aHasIkLinkLimits);
        cAttr.addChild(aIkLinkLimitMin);
        cAttr.addChild(aIkLinkLimitMax);
        addAttribute(aIkLinkLimits);

        return MS::kSuccess;
    }
    catch (...)
    {
        return MS::kFailure;
    }
}
