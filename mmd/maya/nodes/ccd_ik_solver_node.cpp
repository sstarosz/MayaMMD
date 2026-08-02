/*
 * ccd_ik_solver_node.cpp
 *
 * C++ CCD IK Solver implementation — ported from Python ccd_ik_solver_node.py.
 */

#include "ccd_ik_solver_node.h"

#include <maya/MFnIkJoint.h>
#include <maya/MFnIkHandle.h>
#include <maya/MFnIkEffector.h>
#include <maya/MIkHandleGroup.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MFnDagNode.h>
#include <maya/MPlug.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MPoint.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>
#include <string>

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
    int maxIters = maxItersPlug.asInt();

    MPlug limitPlug = fnDep.findPlug(aLimitRadian, true);
    double limitRadian = limitPlug.asDouble();
    double maxAngleRad = (limitRadian > 0.0) ? limitRadian : M_PI / 2.0;

    // ── Get handle group ─────────────────────────────────────────────
    MIkHandleGroup *handleGroup = this->handleGroup();
    if (!handleGroup)
        return MS::kSuccess;

    MObject handle = handleGroup->handle(0);
    MDagPath handlePath = MDagPath::getAPathTo(handle);
    MFnIkHandle fnHandle(handlePath);

    MDagPath endEffector;
    fnHandle.getEffector(endEffector);
    MFnIkEffector effectorFn(endEffector);

    MDagPath startJoint;
    fnHandle.getStartJoint(startJoint);
    MString startJointName = startJoint.fullPathName();

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
    for (auto &jp : jointPaths)
    {
        int bi = getJointPmxBoneIndex(jp);
        boneIdxCache[jp.partialPathName().asChar()] = bi;
    }

    // ── Per-joint plane angle state ──────────────────────────────────
    std::unordered_map<std::string, double> planeAngleState;

    // ── Best-distance tracking ───────────────────────────────────────
    double bestDistance = std::numeric_limits<double>::infinity();
    int stallCount = 0;

    // ── CCD Loop ─────────────────────────────────────────────────────
    for (int it = 0; it < maxIters; ++it)
    {
        MPoint effectorPos = effectorFn.rotatePivot(MSpace::kWorld);
        MVector toTarget = handlePos - effectorPos;
        double currDist = toTarget.length();
        if (currDist < tolerance)
            break;

        for (auto &jointPath : jointPaths)
        {
            MFnIkJoint jfn(jointPath);
            MPoint jpos = jfn.rotatePivot(MSpace::kWorld);

            MVector toEff = effectorPos - jpos;
            MVector toTgt = handlePos - jpos;

            double lenEff = toEff.length();
            double lenTgt = toTgt.length();
            if (lenEff < eps || lenTgt < eps)
                continue;

            MVector dEff = toEff / lenEff;
            MVector dTgt = toTgt / lenTgt;

            double dotEffTgt = dEff * dTgt;
            if (dotEffTgt > 1.0 - eps)
                continue;

            // ── Per-joint axis constraint ────────────────────────────
            int bi = -1;
            auto itBI = boneIdxCache.find(jointPath.partialPathName().asChar());
            if (itBI != boneIdxCache.end())
                bi = itBI->second;

            auto limitsIt = limitsMap.find(bi);
            const LinkLimit *limits = (limitsIt != limitsMap.end()) ? &limitsIt->second : nullptr;
            int limitAxis = getSingleAxisIndex(limits);

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
                if (angle < M_PI / 180.0 && limits)
                {
                    double loMmd = limits->lo[limitAxis];
                    double hiMmd = limits->hi[limitAxis];
                    double loMaya = -hiMmd;
                    double hiMaya = -loMmd;
                    double half = (loMaya + hiMaya) * 0.5;
                    if (fabs(half - signedAngle) > fabs(half + signedAngle))
                        signedAngle = -signedAngle;
                }

                // Accumulate angle
                std::string jpName = jointPath.partialPathName().asChar();
                double prevAngle = planeAngleState[jpName];
                double newAngle = prevAngle + signedAngle;

                // Resolve sign ambiguity on first iteration
                if (it == 0 && prevAngle == 0.0 && limits)
                {
                    double loMmd = limits->lo[limitAxis];
                    double hiMmd = limits->hi[limitAxis];
                    double loMaya = -hiMmd;
                    double hiMaya = -loMmd;
                    if (newAngle < loMaya || newAngle > hiMaya)
                    {
                        if (-newAngle > loMaya && -newAngle < hiMaya)
                            newAngle = -newAngle;
                        else
                        {
                            double half = (loMaya + hiMaya) * 0.5;
                            if (fabs(half - newAngle) > fabs(half + newAngle))
                                newAngle = -newAngle;
                        }
                    }
                }

                // Clamp to limits
                if (limits)
                {
                    double loMmd = limits->lo[limitAxis];
                    double hiMmd = limits->hi[limitAxis];
                    double loMaya = -hiMmd;
                    double hiMaya = -loMmd;
                    newAngle = std::max(loMaya, std::min(hiMaya, newAngle));
                }

                double deltaAngle = newAngle - prevAngle;
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
                double axisLen = axis.length();
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
                double angle = dEff.angle(dTgt);
                double clampedAngle = std::min(angle, maxAngleRad);
                MQuaternion quat(clampedAngle, axis);
                jfn.rotateBy(quat, MSpace::kWorld);
            }

            // Update effector position after joint rotation
            effectorPos = effectorFn.rotatePivot(MSpace::kWorld);
        }

        // ── Best-distance tracking ───────────────────────────────────
        MVector postDistVec = effectorPos - handlePos;
        double postDist = postDistVec.length();
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
int CCDIKSolverNode::getJointPmxBoneIndex(const MDagPath &jointPath)
{
    try
    {
        MFnDependencyNode fnNode(jointPath.node());
        MPlug plug = fnNode.findPlug("pmxBoneIndex", true);
        return plug.asInt();
    }
    catch (...)
    {
        return -1;
    }
}

std::unordered_map<int, CCDIKSolverNode::LinkLimit>
CCDIKSolverNode::readLinkLimitsMap(MFnDependencyNode &fnDep)
{
    std::unordered_map<int, LinkLimit> result;
    try
    {
        MPlug limitsPlug = fnDep.findPlug(aIkLinkLimits, true);
        int numElems = limitsPlug.numElements();
        for (int i = 0; i < numElems; ++i)
        {
            try
            {
                MPlug elem = limitsPlug.elementByPhysicalIndex(i);
                int boneIdx = elem.child(0).asInt();
                bool enabled = elem.child(1).asBool();
                if (!enabled)
                    continue;

                MPlug minPlug = elem.child(2);
                MPlug maxPlug = elem.child(3);

                LinkLimit ll;
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
        // No link limits attribute
    }
    return result;
}

int CCDIKSolverNode::getSingleAxisIndex(const LinkLimit *limits)
{
    if (!limits)
        return -1;

    int nonzero[3];
    int nzCount = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (limits->lo[i] != 0.0 || limits->hi[i] != 0.0)
        {
            nonzero[nzCount++] = i;
        }
    }

    if (nzCount == 1)
    {
        int ax = nonzero[0];
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

// ===========================================================================
// Creator / Initializer
// ===========================================================================
void *CCDIKSolverNode::creator()
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
        aHasIkLinkLimits = nAttr.create("hasIkLinkLimits", "hill", MFnNumericData::kBoolean, false);
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
