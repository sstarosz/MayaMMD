/**
 * @file maya_utils.hpp
 * @brief Small C++ utilities shared by the Maya plugin sources.
 */

#pragma once

#include <maya/MStatus.h>

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

} // namespace mmd::maya

// Drop-in replacement for the Maya SDK's CHECK_MSTATUS macro: forwards to
// mmd::maya::checkMStatus() with the call site's file/line.  A function-like
// macro is the standard way to capture __FILE__/__LINE__ at the call site
// (the cppcoreguidelines-macro-usage check's canonical exception).
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MMD_CHECK_MSTATUS(_status) ::mmd::maya::checkMStatus((_status), __FILE__, __LINE__)
