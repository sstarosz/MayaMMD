/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_constraint_cmd.hpp
 *
 * RigidBodyConstraintCmd — native C++ command for appending PMX joints
 * (rigid-body CONSTRAINTS) to an pmxPhysicsNode's ``joints`` array.
 *
 * WHY C++ (not a Python MPxCommand): identical rationale to ``pmxRigidBody``
 * (see rigid_body_cmd.hpp) — the Python command layer crashes inside
 * OpenMaya's lazy MSyntax creation in mayapy 2026, and the Python
 * MArgParser multi-double flag reads were flaky.  A native command has none
 * of that.
 *
 * Maya command convention: create / edit / query (default = create).
 *
 * v1.0 — CREATE MODE ONLY (create is the default — no -create flag):
 *
 *     pmxRigidBodyConstraint <solver | modelRoot>
 *         -index <int>              optional target index (must be the next
 *                                   free index; omit to auto-append)
 *         -bodyA <int> -bodyB <int>   PMX rigid-body indices the joint links
 *                                   (validated against the current body
 *                                   count)
 *         -type <int>               PMX joint type 0..5 (validated)
 *                                   SPRING_6DOF=0 / 6DOF=1 / P2P=2 /
 *                                   CONETWIST=3 / SLIDER=4 / HINGE=5
 *         -position <x y z>         joint frame position (MMD space; Z-flip)
 *         -rotation <x y z>         joint frame rotation (MMD radians;
 *                                   handedness flip to Maya degrees)
 *         -linearMin <x y z> -linearMax <x y z>
 *                                   PMX verbatim (position limits, PMX units)
 *         -angularMin <x y z> -angularMax <x y z>
 *                                   PMX radians VERBATIM — the node hands
 *                                   angular limits to Bullet unchanged
 *                                   (they are NOT converted to degrees)
 *         -linearSpring <x y z> -angularSpring <x y z>
 *                                   PMX verbatim (spring stiffness)
 *
 * SIMULATION IS DISABLED: create writes the joint DATA so the node holds the
 * full constraint set; no solver stepping happens here (the ``time`` input
 * is connected by the Python builder once bodies AND joints exist).
 * Edit/query/remove and batch create are later steps.
 */

#pragma once

#include <maya/MPxCommand.h>
#include <maya/MString.h>

class MSyntax;
class MArgList;

class RigidBodyConstraintCmd : public MPxCommand
{
  public:
    static constexpr const char* kName = "pmxRigidBodyConstraint";

    RigidBodyConstraintCmd() = default;
    ~RigidBodyConstraintCmd() override = default;

    static void* creator();
    static MSyntax syntaxCreator();

    MStatus doIt(const MArgList& args) override;
    bool isUndoable() const override { return false; }
};
