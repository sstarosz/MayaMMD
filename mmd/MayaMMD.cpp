/*
 * SPDX-License-Identifier: MIT
 *
 * MayaMMD.cpp
 *
 * MayaMMD — Maya C++ plugin entry point (.mll).
 *
 * initializePlugin does TWO things on the same plugin handle:
 *   1. Registers native C++ nodes via MFnPlugin::registerNode
 *   2. Calls Python (via MGlobal::executePythonCommand) to register
 *      Python-based nodes/commands and set up the UI
 *      Python entry point is mmd.plugin.initializePlugin()
 *
 * Everything registers under the same "MayaMMD" identity —
 * one Plugin Manager entry.
 */

#include <maya/MDrawRegistry.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MStatus.h>
#include <maya/MString.h>

#include "maya/nodes/ccd_ik_solver_node.h"
#include "maya/nodes/mmd_physics_draw_override.h"
#include "maya/nodes/mmd_physics_node.h"
#include "version.hpp"

// Draw-override registration (see MHWRender::MDrawRegistry): the override is
// keyed by a draw-database CLASSIFICATION plus a unique REGISTRANT id, not the
// node's MTypeId.
static const MString kPhysicsDrawClassification("drawdb/geometry/mmdPhysicsNode");
static const MString kPhysicsDrawRegistrant("MayaMMD");

// ===========================================================================
// Python-side initialization — called from C++ via executePythonCommand
// ===========================================================================
// The .mod file handles MAYA_PLUG_IN_PATH so Maya finds the .mll.
// We inject the project root into sys.path before importing mmd.plugin
// so Python can locate the mmd/ package.  PYTHONPATH is NOT set globally
// (that would crash Maya by loading Maya-API imports too early).
static MStatus _run_python_initialization()
{
    // importlib.reload ensures plugin.py source changes take effect
    // when the .mll is reloaded without restarting Maya (the bare
    // ``import mmd.plugin`` would be a no-op on a cached module).
    MStatus stat = MGlobal::executePythonCommand("import sys; "
                                                 "sys.path.insert(0, '" PROJECT_ROOT_DIR "'); "
                                                 "import importlib, mmd.plugin; "
                                                 "importlib.reload(mmd.plugin); "
                                                 "mmd.plugin.initializePlugin()");
    if (!stat)
    {
        MGlobal::displayWarning("  Python initialization failed — "
                                "Python components unavailable");
    }
    return stat;
}

static MStatus _run_python_uninitialization()
{
    MGlobal::executePythonCommand("import sys; "
                                  "sys.path.insert(0, '" PROJECT_ROOT_DIR "'); "
                                  "import importlib, mmd.plugin; "
                                  "importlib.reload(mmd.plugin); "
                                  "mmd.plugin.uninitializePlugin()");
    return MS::kSuccess;
}

// ===========================================================================
// Plugin entry points
// ===========================================================================
PLUGIN_EXPORT MStatus initializePlugin(MObject mobject)
{
    MStatus stat;
    MFnPlugin plugin(mobject, "Sebastian Starosz", PROJECT_VERSION, "Any", &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

    // 1. Register C++ CCD IK solver node natively
    {
        MString classification("ikSolver/ccd");
        stat = plugin.registerNode(CCDIKSolverNode::kNodeName, CCDIKSolverNode::kTypeId,
                                   CCDIKSolverNode::creator, CCDIKSolverNode::initialize,
                                   MPxNode::kIkSolverNode, &classification);
    }
    if (!stat)
        MGlobal::displayWarning("  ⚠ CCD IK solver registration failed");

    // 1b. Register the native rigid-body physics node (embedded Bullet).
    //     An MPxLocatorNode the evaluation manager steps on every time change
    //     — this is the MMD secondary-movement engine that replaces mayaBullet.
    //     As a locator it also draws its own guide visualization (see the draw
    //     override registration below).
    {
        MString classification(MMDPhysicsNode::kNodeClassify);
        stat = plugin.registerNode(MMDPhysicsNode::kNodeName, MMDPhysicsNode::kTypeId,
                                   MMDPhysicsNode::creator, MMDPhysicsNode::initialize,
                                   MPxNode::kLocatorNode, &classification);
    }
    if (!stat)
        MGlobal::displayWarning("  ⚠ mmdPhysicsNode registration failed");

    // 1c. Register the viewport draw override that renders the node's guide
    //     visualization (wireframe box/sphere/capsule per body, group-colored)
    //     directly from the node's internal Bullet state.
    {
        stat = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
            kPhysicsDrawClassification, kPhysicsDrawRegistrant, MMDPhysicsDrawOverride::creator);
    }
    if (!stat)
        MGlobal::displayWarning("  ⚠ mmdPhysicsNode draw override registration failed");

    // 2. Call Python to register Python nodes/commands and set up UI.
    //    PYTHONPATH is set by Maya's .mod file (or Maya.env) before
    //    plugin loading, so mmd/ is already importable.
    stat = _run_python_initialization();
    if (!stat)
    {
        MGlobal::displayWarning("MayaMMD: Python initialization failed — "
                                "boneMorphNode, boneBlendShape, and UI will be unavailable");
    }

    return MS::kSuccess;
}

PLUGIN_EXPORT MStatus uninitializePlugin(MObject mobject)
{
    MFnPlugin plugin(mobject);

    // 1. Deregister Python components first
    _run_python_uninitialization();

    // 2. Deregister C++ nodes and commands
    //    (the draw override creator must be removed before its node type).
    MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(kPhysicsDrawClassification,
                                                            kPhysicsDrawRegistrant);
    plugin.deregisterNode(MMDPhysicsNode::kTypeId);
    plugin.deregisterNode(CCDIKSolverNode::kTypeId);

    return MS::kSuccess;
}
