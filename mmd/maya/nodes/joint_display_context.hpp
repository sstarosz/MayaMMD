/*
 * SPDX-License-Identifier: MIT
 *
 * joint_display_context.hpp
 *
 * Session-wide state shared by every joint and body viewport override, so
 * that work which depends on the SELECTION (not on one node) happens once per
 * event instead of once per node:
 *
 *   - the selected joints (lead first) and the A/B roles of their bodies,
 *     resolved on SelectionChanged; only the joints and bodies whose display
 *     actually changes are dirtied;
 *   - model-editor changes (isolate select on or off, among others): every
 *     rigid-body shape and joint is redrawn.  VP2 could leave plug-in shapes
 *     that had been isolated out undrawn until something else refreshed them
 *     (re-selecting them, or a parent); this also updates the selected
 *     joints' fallback outlines;
 *   - one shared stock shader per colour (lit surface and flat wire).
 *
 * Simulation steps need no callback: every joint's solverStep input is
 * connected to its solver's outStep, and the joint node marks its drawing
 * dirty after each step (RigidBodyJoint::postEvaluation).
 *
 * Everything here runs on the main thread (DG preparation and event
 * callbacks).  install()/uninstall() are called from the plugin entry points.
 */

#pragma once

#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MObject.h>
#include <maya/MObjectHandle.h>
#include <maya/MShaderManager.h>

#include <vector>

namespace mmd::maya::joint_display
{
// A selected joint as the viewport needs it: its connected body shapes (null
// handles when an end is unconnected).
struct SelectedJoint
{
    MDagPath path;
    MObjectHandle joint;
    MObjectHandle solver;
    MObjectHandle bodyA;
    MObjectHandle bodyB;
};

void install();
void uninstall();

// Selected, drawn joints in lead-first order (Maya's lead is the LAST entry of
// the active selection list).
const std::vector<SelectedJoint>& selectedJoints();
bool isSelectedJoint(const MObject& joint);

// Roles of a body shape across the selected joints (joint_palette.hpp role bits).
unsigned int bodyRoles(const MObject& bodyShape);

// Shared stock shaders with this colour (owned here): lit surface for shaded
// hubs (surface_shader.hpp), flat colour for wireframe hubs, dashed flat
// colour for inactive hubs.
MHWRender::MShaderInstance* solidShader(const MColor& color);
MHWRender::MShaderInstance* wireShader(const MColor& color);
MHWRender::MShaderInstance* dashedWireShader(const MColor& color);
} // namespace mmd::maya::joint_display
