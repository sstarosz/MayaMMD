/*
 * SPDX-License-Identifier: MIT
 *
 * pmx_rigid_body_cmd_utils.hpp
 *
 * Shared helpers for the native ``pmxRigidBody`` / ``pmxRigidBodyConstraint``
 * commands.  Both commands resolve the same "solver node or model root"
 * target argument, convert the same MMD -> Maya rest transforms and name
 * their nodes by the same policy, so that logic lives here instead of being
 * copy-pasted in each command.
 *
 * Header-only (all functions inline): the helpers are small and both commands
 * are compiled into the same plugin.
 */

#pragma once

#include <maya/MFn.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MString.h>

#include "nodes/rigid_body_node.hpp"
#include "physics_math.hpp"

#include <algorithm>
#include <string>

namespace mmd::maya
{

/// PMXNamingManager._sanitize_name: every character outside [A-Za-z0-9_]
/// becomes '_', runs of '_' collapse, the ends are trimmed, a leading digit
/// gets a "Model_" prefix, and an empty result becomes "Model".
inline std::string sanitizePmxName(const std::string& name)
{
    std::string out;
    for (const char ch : name)
    {
        const unsigned char c = static_cast<unsigned char>(ch);
        const bool keep = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
        if (keep)
            out.push_back(ch);
        else if (!out.empty() && out.back() != '_')
            out.push_back('_');
    }
    while (!out.empty() && out.back() == '_')
        out.pop_back();
    if (!out.empty() && out[0] >= '0' && out[0] <= '9')
        out = "Model_" + out;
    return out.empty() ? "Model" : out;
}

/// A rigid body's or joint's DAG node name when the caller passes none (the
/// importer passes PMXNamingManager's through -nodeName), by the same
/// policy: `{model}_{name}{suffix}`, where name is the local name when it is
/// ASCII-safe, else the universal name, else `fallback`.  The model prefix
/// comes from the solver's name "{model}_RigidBodySolver".  Maya
/// auto-uniquifies.
inline MString pmxNodeName(const MObject& solverNode, const MString& nameLocal,
                           const MString& nameUniversal, const std::string& fallback,
                           const char* suffix = "")
{
    MFnDependencyNode fn(solverNode);
    MString prefix = fn.name();
    const int at = prefix.rindexW("_RigidBodySolver");
    if (at >= 0)
        prefix = prefix.substringW(0, at - 1);

    std::string segment = fallback;
    for (const MString& name : {nameLocal, nameUniversal})
    {
        const std::string utf8 = name.asUTF8();
        const bool asciiSafe = std::all_of(utf8.begin(), utf8.end(), [](char c)
                                           { return static_cast<unsigned char>(c) < 0x80; });
        if (!utf8.empty() && asciiSafe)
        {
            segment = sanitizePmxName(utf8);
            break;
        }
    }
    return prefix + "_" + segment.c_str() + suffix;
}

/// Resolve *target* to a pmxRigidBodyNode MObject (direct node or model root).
inline bool resolveSolver(const MString& target, MObject& outSolver)
{
    MSelectionList sel;
    if (sel.add(target) != MS::kSuccess || sel.length() == 0)
        return false;
    MObject obj;
    if (sel.getDependNode(0, obj) != MS::kSuccess)
        return false;
    if (!obj.hasFn(MFn::kDependencyNode))
        return false;

    MFnDependencyNode fn(obj);
    if (fn.typeName() == RigidBodyNode::kNodeName)
    {
        outSolver = obj;
        return true;
    }
    // Model root: resolve the pmxRigidBodyNode string attribute.
    MPlug p = fn.findPlug("pmxRigidBodyNode", true);
    if (!p.isNull())
    {
        const MString solverName = p.asString();
        if (solverName.length() > 0)
        {
            MSelectionList sel2;
            if (sel2.add(solverName) == MS::kSuccess && sel2.length() > 0)
            {
                MObject obj2;
                if (sel2.getDependNode(0, obj2) == MS::kSuccess && obj2.hasFn(MFn::kDependencyNode))
                {
                    MFnDependencyNode fn2(obj2);
                    if (fn2.typeName() == RigidBodyNode::kNodeName)
                    {
                        outSolver = obj2;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/// MMD -> Maya world-space translation: the Z-flip.
inline mmd::core::Double3 mmdToMayaTranslate(const mmd::core::Double3& t)
{
    return mmd::core::Double3(t.x, t.y, -t.z);
}

/// MMD -> Maya world-space rotation: MMD radians to Maya degrees with the
/// handedness flip (X/Y negated, Z unchanged).
inline mmd::core::Double3 mmdToMayaRotateDeg(const mmd::core::Double3& r)
{
    return mmd::core::Double3(-mmd::core::physics_math::rad2deg(r.x),
                              -mmd::core::physics_math::rad2deg(r.y),
                              mmd::core::physics_math::rad2deg(r.z));
}

} // namespace mmd::maya
