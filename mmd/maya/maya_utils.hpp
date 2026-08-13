/**
 * @file maya_utils.hpp
 * @brief Small C++ utilities shared by the Maya plugin sources.
 */

#pragma once

#include <maya/MDGModifier.h>
#include <maya/MDagPath.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnNumericData.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MStatus.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MVector.h>

#include "common.hpp"
#include "physics_math.hpp"

namespace mmd::maya
{
// A joint's stored PMX bone index (pmxBoneIndex), or -1.  The bone builder
// stamps it on every joint, and the DAG IS the PMX bone hierarchy — shared
// by pmxRigidBody (resolveBone) and pmxPhysicsNode (readBodyData's message +
// DAG resolution).  Every failure mode is handled explicitly (no exceptions):
// an invalid path, a non-dependency node, or a missing attribute reports -1.
inline int jointPmxBoneIndex(const MDagPath& jointPath)
{
    if (!jointPath.isValid())
        return -1;
    MStatus stat;
    MFnDependencyNode fn(jointPath.node(), &stat);
    if (stat != MS::kSuccess)
        return -1;
    MPlug plug = fn.findPlug("pmxBoneIndex", true, &stat);
    if (plug.isNull())
        return -1;
    return plug.asInt();
}

// Same behaviour as Maya's CHECK_MSTATUS macro — log a pAPIerror (with the
// caller's file/line) when `status` is not kSuccess — but as a function: the
// status is taken by const reference, so no `MStatus _maya_status = (status)`
// local copy is created (that copy lives inside the SDK macro and is what
// clang-tidy's performance-unnecessary-copy-initialization flags on every
// call site).
inline void checkMStatus(const MStatus& status, const char* file, int line)
{
    if (MStatus::kSuccess != status)
    {
        status.pAPIerror(file, line);
    }
}

// MPlug has no setValue(MMatrix) overload — wrap the matrix in an
// MFnMatrixData MObject (the standard pattern for matrix plugs).
// (MPlug is passed by value: child() returns a temporary, and setValue is
// non-const — MSVC tolerates the temporary binding, clang does not.)
inline void setPlugMatrixValue(MPlug plug, const MMatrix& m)
{
    MFnMatrixData data;
    const MObject obj = data.create(m);
    plug.setValue(obj);
}

// MPlug has no setValue3Double — wrap 3 doubles in an MFnNumericData object.
inline void setPlugDouble3(MPlug plug, const mmd::core::Double3& v)
{
    MFnNumericData data;
    const MObject obj = data.create(MFnNumericData::k3Double);
    data.setData3Double(v.x, v.y, v.z);
    plug.setValue(obj);
}

// 4x4 row-vector matrix from translate + XYZ euler DEGREES.  Shared by the
// pmxRigidBody / pmxRigidBodyConstraint commands for the world-space rest /
// frame conversions (deliberately does NOT re-read a scale component — the
// callers build the matrix from PMX T/R only).
inline MMatrix matrixFromTR(const mmd::core::Double3& t, const mmd::core::Double3& r)
{
    MTransformationMatrix mt;
    mt.setTranslation(MVector(t.x, t.y, t.z), MSpace::kTransform);
    double rot[3] = {mmd::core::physics_math::deg2rad(r.x), mmd::core::physics_math::deg2rad(r.y),
                     mmd::core::physics_math::deg2rad(r.z)};
    // &rot[0] instead of the bare array: passing `rot` would decay it to a
    // pointer (cppcoreguidelines-pro-bounds-array-to-pointer-decay).
    mt.setRotation(&rot[0], MTransformationMatrix::kXYZ);
    return mt.asMatrix();
}

// Connect src → dst, first disconnecting any other source already driving dst.
// Idempotent: if dst is already driven by the SAME source, nothing is changed
// and kSuccess is returned (the anchor-world input is re-connected by every
// kinematic body create — this makes the repeated call a cheap no-op).
inline MStatus connectOrReplace(const MPlug& src, const MPlug& dst)
{
    MPlugArray sources;
    dst.connectedTo(sources, true, false);
    for (unsigned int i = 0; i < sources.length(); ++i)
    {
        if (sources[i] == src)
            return MS::kSuccess; // already connected to the requested source
    }
    MDGModifier mod;
    for (unsigned int i = 0; i < sources.length(); ++i)
        mod.disconnect(sources[i], dst);
    if (mod.connect(src, dst) != MS::kSuccess)
        return MS::kFailure;
    return mod.doIt();
}

} // namespace mmd::maya

// Drop-in replacement for the Maya SDK's CHECK_MSTATUS macro: forwards to
// mmd::maya::checkMStatus() with the call site's file/line.  A function-like
// macro is the standard way to capture __FILE__/__LINE__ at the call site
// (the cppcoreguidelines-macro-usage check's canonical exception).
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MMD_CHECK_MSTATUS(_status) ::mmd::maya::checkMStatus((_status), __FILE__, __LINE__)
