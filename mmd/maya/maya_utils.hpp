/**
 * @file maya_utils.hpp
 * @brief Small C++ utilities shared by the Maya plugin sources.
 */

#pragma once

#include <maya/MDGModifier.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnNumericData.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MStatus.h>

#include "common.hpp"

namespace mmd::maya
{
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

// Connect src → dst, first disconnecting any other source already driving dst.
inline MStatus connectOrReplace(const MPlug& src, const MPlug& dst)
{
    MDGModifier mod;
    if (dst.isConnected())
    {
        MPlugArray sources;
        dst.connectedTo(sources, true, false);
        for (unsigned int i = 0; i < sources.length(); ++i)
        {
            if (sources[i] != src)
                mod.disconnect(sources[i], dst);
        }
    }
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
