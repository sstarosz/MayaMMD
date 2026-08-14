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

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MStatus.h>
#include <maya/MString.h>

#include "maya/cmds/rigid_body_cmd.hpp"
#include "maya/cmds/rigid_body_constraint_cmd.hpp"
#include "maya/nodes/ccd_ik_solver_node.h"
#include "maya/nodes/rigid_body_node.hpp"
#include "version.hpp"

// ===========================================================================
// Python-side initialization — called from C++ via executePythonCommand
// ===========================================================================
// The .mod file handles MAYA_PLUG_IN_PATH so Maya finds the .mll.
// We inject the project root into sys.path before importing mmd.plugin
// so Python can locate the mmd/ package.  PYTHONPATH is NOT set globally
// (that would crash Maya by loading Maya-API imports too early).
static MStatus run_python_initialization()
{
    // importlib.reload ensures plugin.py source changes take effect
    // when the .mll is reloaded without restarting Maya (the bare
    // ``import mmd.plugin`` would be a no-op on a cached module).
    MStatus stat;
    stat = MGlobal::executePythonCommand("import sys; "
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

static MStatus run_python_uninitialization()
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
    // stat is written through the &stat out-parameter of the MFnPlugin ctor
    // (Maya idiom) — the copy-init check is a false positive here, reported
    // through the CHECK_MSTATUS macro below.
    MStatus stat;
    MFnPlugin plugin(mobject, "Sebastian Starosz", PROJECT_VERSION, "Any", &stat);
    // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
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
    //     An MPxLocatorNode that owns a Maya-free Bullet world and steps it on
    //     every time change — this is the MMD secondary-movement engine that
    //     replaces mayaBullet.  (A draw override for the guide visualization
    //     is planned but intentionally not added yet.)
    {
        MString classification(RigidBodyNode::kNodeClassify);
        stat =
            plugin.registerNode(RigidBodyNode::kNodeName, RigidBodyNode::kTypeId, RigidBodyNode::creator,
                                RigidBodyNode::initialize, MPxNode::kLocatorNode, &classification);
    }
    if (!stat)
        MGlobal::displayWarning("  ⚠ pmxRigidBodyNode registration failed");

    // 1c. Register the native rigid-body command (pmxRigidBody).  It lives in
    //     C++ (not Python) because the Python command layer crashed inside
    //     OpenMaya's lazy MSyntax creation in mayapy 2026.  Create mode only
    //     for now — body data + kinematic anchors + the baked write-back K
    //     offset; the Python builder wires the solver and the outputs.
    {
        stat = plugin.registerCommand(RigidBodyCmd::kName, RigidBodyCmd::creator,
                                      RigidBodyCmd::syntaxCreator);
    }
    if (!stat)
        MGlobal::displayWarning("  ⚠ pmxRigidBody command registration failed");

    // 1d. Register the native rigid-body-constraint command (pmxRigidBodyConstraint).
    //     Same C++ rationale as pmxRigidBody.  Create mode only — writes the
    //     joint DATA; the node holds the full constraint set and the Python
    //     builder wires it into the time-driven solver.
    {
        stat =
            plugin.registerCommand(RigidBodyConstraintCmd::kName, RigidBodyConstraintCmd::creator,
                                   RigidBodyConstraintCmd::syntaxCreator);
    }
    if (!stat)
        MGlobal::displayWarning("  ⚠ pmxRigidBodyConstraint command registration failed");

    // 2. Call Python to register Python nodes/commands and set up UI.
    //    PYTHONPATH is set by Maya's .mod file (or Maya.env) before
    //    plugin loading, so mmd/ is already importable.
    stat = run_python_initialization();
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
    run_python_uninitialization();

    // 2. Deregister C++ nodes and commands
    plugin.deregisterNode(RigidBodyNode::kTypeId);
    plugin.deregisterNode(CCDIKSolverNode::kTypeId);
    plugin.deregisterCommand(RigidBodyCmd::kName);
    plugin.deregisterCommand(RigidBodyConstraintCmd::kName);

    return MS::kSuccess;
}
