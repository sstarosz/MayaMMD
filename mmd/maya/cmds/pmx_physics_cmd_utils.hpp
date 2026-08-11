/*
 * SPDX-License-Identifier: MIT
 *
 * pmx_physics_cmd_utils.hpp
 *
 * Shared helpers for the native ``pmxRigidBody`` / ``pmxRigidBodyConstraint``
 * commands.  Both commands resolve the same "solver node or model root"
 * target argument and convert the same MMD -> Maya rest transforms, so that
 * logic lives here instead of being copy-pasted in each command.
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

#include "nodes/physics_node.h"
#include "physics_math.hpp"

namespace mmd::maya
{

/// Resolve *target* to an pmxPhysicsNode MObject (direct node or model root).
inline bool resolveSolver(const MString& target, MObject& outNode)
{
    try
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
        if (fn.typeName() == PhysicsNode::kNodeName)
        {
            outNode = obj;
            return true;
        }
        // Model root: resolve the pmxPhysicsNode string attribute.
        MStatus stat;
        MPlug p = fn.findPlug("pmxPhysicsNode", true, &stat);
        if (!p.isNull())
        {
            const MString solverName = p.asString();
            if (solverName.length() > 0)
            {
                MSelectionList sel2;
                if (sel2.add(solverName) == MS::kSuccess && sel2.length() > 0)
                {
                    MObject obj2;
                    if (sel2.getDependNode(0, obj2) == MS::kSuccess &&
                        obj2.hasFn(MFn::kDependencyNode))
                    {
                        MFnDependencyNode fn2(obj2);
                        if (fn2.typeName() == PhysicsNode::kNodeName)
                        {
                            outNode = obj2;
                            return true;
                        }
                    }
                }
            }
        }
    }
    // Resolution failure is reported through the bool return.
    // NOLINTNEXTLINE(bugprone-empty-catch)
    catch (...)
    {
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
