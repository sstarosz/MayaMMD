/*
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_cmd.hpp
 *
 * RigidBodyCmd — native C++ command for operating on a pmxPhysicsNode.
 *
 * WHY C++ (not a Python MPxCommand): the Python command layer in this
 * environment is fragile — the lazy creation of the command's MSyntax
 * (Maya calls syntaxCreator() the first time the command is invoked) crashed
 * the process inside OpenMaya's MSyntax constructor in mayapy 2026, and the
 * Python MArgParser multi-double flag reads were flaky.  A native command has
 * none of that: MSyntax and MArgParser are plain C++ here.
 *
 * Maya command convention: create / edit / query (default = create, with
 * -e/-edit and -q/-query enabled in the syntax).
 *
 * v1.0 — CREATE MODE ONLY (create is the default — no -create flag):
 *
 *     pmxRigidBody <solver | modelRoot>
 *         -index <int>              optional target index (must be the next
 *                                   free index; omit to auto-append)
 *         -name <string>            PMX body name (local) → bodies[i].bodyNameLocal
 *         -nameUniversal <string>   PMX body name (universal) → bodies[i].bodyNameUniversal
 *         -bone <joint | pmxBoneIdx>  related joint (Maya name/path or PMX
 *                                   bone index) — drives the bone binding
 *         -shape <sphere|box|capsule>
 *         -size <x y z>             PMX shape_size VERBATIM (full size — box
 *                                   extents are full, not half).  Stored in
 *                                   bodies[i].bodyShapeSize; the node derives
 *                                   the engine radius/extents/length by
 *                                   collider type (mmd::core::applyShapeSize).
 *         -position <x y z>         PMX shape position (MMD space; Z-flip applied)
 *         -rotation <x y z>         PMX shape rotation (MMD radians; handedness flip)
 *         -mass <double>
 *         -linearDamping <double>
 *         -angularDamping <double>
 *         -friction <double>
 *         -restitution <double>
 *         -group <int>             PMX collision group 0..15 (clamped)
 *         -mask <int>              collide-with mask: bit i set = collides with
 *                                  group i (the PMX non_collision_group field
 *                                  stored verbatim; written into
 *                                  bodies[i].bodyMaskGroup0..15).  Default 0xFFFF.
 *         -physicsMode <followBone|physics|physicsBone>
 *
 * SIMULATION IS DISABLED: create writes the body DATA and binds FOLLOW_BONE
 * bodies to their related joint via the kinematic-anchor input (so the
 * collider lives on the correct bone and displays from its rest pose).  No
 * write-back wiring and no solver stepping — dynamic bodies are data-only
 * for now.  Edit/query/remove and batch create are later steps.
 */

#pragma once

#include <maya/MPxCommand.h>
#include <maya/MString.h>

class MSyntax;
class MArgList;

class RigidBodyCmd : public MPxCommand
{
  public:
    static constexpr const char* kName = "pmxRigidBody";

    RigidBodyCmd() = default;
    ~RigidBodyCmd() override = default;

    static void* creator();
    static MSyntax syntaxCreator();

    MStatus doIt(const MArgList& args) override;
    bool isUndoable() const override { return false; }
};
