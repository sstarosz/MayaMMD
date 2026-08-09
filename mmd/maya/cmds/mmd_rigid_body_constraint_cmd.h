/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_rigid_body_constraint_cmd.h
 *
 * MmdRigidBodyConstraintCmd — native C++ command for appending PMX joints
 * (rigid-body CONSTRAINTS) to an mmdPhysicsNode's ``joints`` array.
 *
 * This is the native replacement for the former Python
 * ``rigid_body_builder._set_joint_attributes`` (the single path by which PMX
 * joints — constraints BETWEEN rigid bodies — enter the node).  It follows
 * the same Maya create/edit/query convention as ``mmdRigidBody`` and lives in
 * C++ for the same reason: the Python command layer crashes inside OpenMaya's
 * lazy MSyntax creation in mayapy 2026.
 *
 * Create mode (default):
 *
 *     mmdRigidBodyConstraint <solver | modelRoot>
 *         -index <int>            optional target index (must be the next
 *                                 free index; omit to auto-append)
 *         -bodyA <int> -bodyB <int>   PMX rigid-body indices the joint links
 *         -type <int>             PMX joint type 0..5
 *         -position <x y z>       joint frame position (MMD space; Z-flip)
 *         -rotation <x y z>       joint frame rotation (MMD radians; handedness)
 *         -linearMin <x y z> -linearMax <x y z>
 *         -angularMin <x y z> -angularMax <x y z>
 *         -linearSpring <x y z> -angularSpring <x y z>
 *
 * SIMULATION IS DISABLED: the joint is stored as DATA so the node holds the
 * full constraint set; no solver stepping happens here.
 */

#pragma once

#include <maya/MPxCommand.h>
#include <maya/MString.h>

class MSyntax;
class MArgList;
class MArgParser;
class MObject;

class MmdRigidBodyConstraintCmd : public MPxCommand
{
  public:
    static constexpr const char* kName = "mmdRigidBodyConstraint";

    MmdRigidBodyConstraintCmd() = default;
    ~MmdRigidBodyConstraintCmd() override = default;

    static void* creator();
    static MSyntax syntaxCreator();

    MStatus doIt(const MArgList& args) override;
    bool isUndoable() const override { return false; }

  private:
    // Resolve *target* to an mmdPhysicsNode MObject (direct node or model root).
    static bool resolveSolver(const MString& target, MObject& outNode);
    // Create mode: append one joint; returns the new index.
    MStatus doCreate(const MArgParser& parser, const MObject& solverNode, int& outIndex);
};
