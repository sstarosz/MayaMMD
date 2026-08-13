/*
 * ccd_ik_solver_node.h
 *
 * C++ CCD IK Solver node — MMD-Compatible Cyclic Coordinate Descent IK.
 *
 * Ported from Python ccd_ik_solver_node.py to eliminate the Python API 1.0
 * registration requirement and MStatus SWIG memory leak warnings.
 *
 * Registered natively by MayaMMD.mll's initializePlugin.
 */

#pragma once

#include <maya/MDagPath.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnIkJoint.h>
#include <maya/MObject.h>
#include <maya/MPoint.h>
#include <maya/MPxIkSolverNode.h>
#include <maya/MQuaternion.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MVector.h>

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

// ===========================================================================
// CCDIKSolverNode
// ===========================================================================
class CCDIKSolverNode : public MPxIkSolverNode
{
  public:
    // Unique Maya node type ID (same as Python version: 0x00080052)
    static const MTypeId kTypeId;
    static constexpr const char* kNodeName = "ccdIKSolverNode";

    CCDIKSolverNode();
    ~CCDIKSolverNode() override;

    // MPxIkSolverNode overrides
    MString solverTypeName() const override;
    MStatus doSolve() override;

    // Node registration helpers
    static void* creator();
    static MStatus initialize();

    // Custom attributes
    static MObject aLimitRadian;
    static MObject aIkLinkLimits;
    static MObject aIkLinkBoneIndex;
    static MObject aHasIkLinkLimits;
    static MObject aIkLinkLimitMin;
    static MObject aIkLinkLimitMax;

  private:
    // Per-joint limit data
    struct LinkLimit
    {
        double lo[3];
        double hi[3];
    };

    // Internal helpers
    static std::unordered_map<int, LinkLimit> readLinkLimitsMap(MFnDependencyNode& fnDep);
    static int getSingleAxisIndex(const LinkLimit* limits);

    // Current accumulated rotation (radians, normalized to [-pi, pi]) of a
    // joint around its local constraint axis.  Used to initialise the
    // per-joint plane-angle state at the start of each solve so repeated
    // solves don't drift from the joint's real rotation.
    static double getCurrentAxisAngle(MFnIkJoint& jfn, int axis);

    // Bend angle (0 = straight) that places the effector at the same radius
    // from the chain root as the target (law of cosines on the
    // root-joint-effector triangle).  Used to fold the chain out of the
    // near-"straight leg" singularity where pure angular CCD stalls.
    static double computeFoldAngle(const MPoint& root, const MPoint& joint, const MPoint& effector,
                                   const MPoint& target);
};
