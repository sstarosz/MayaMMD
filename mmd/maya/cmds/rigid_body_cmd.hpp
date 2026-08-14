/**
 * SPDX-License-Identifier: MIT
 *
 * rigid_body_cmd.hpp
 *
 * RigidBodyCmd — native C++ command for operating on a pmxPhysicsNode.
 *
 * Create mode only (create is the default; -edit/-query are enabled in the
 * syntax but not implemented yet and are rejected):
 *
 *     pmxRigidBody <solver | modelRoot>
 *         -i, -index <int>         optional target index (must be the next free
 *                                  index; omit to auto-append)
 *         -n, -name <string>       PMX body name (local) → bodies[i].bodyNameLocal
 *         -nu, -nameUniversal <string>  PMX body name (universal)
 *         -b, -bone <joint | pmxBoneIdx>  related joint — drives the bone binding
 *         -sh, -shape <sphere|box|capsule>  PMX collider
 *         -sz, -size <x y z>       PMX shape_size VERBATIM (full size — box
 *                                  extents are full, not half; the node derives
 *                                  the engine radius/extents/length by collider
 *                                  type via mmd::core::applyShapeSize)
 *         -p, -position <x y z>    PMX shape position (MMD space; Z-flip applied)
 *         -rot, -rotation <x y z>  PMX shape rotation (MMD radians; handedness flip)
 *         -m, -mass <double>
 *         -ld, -linearDamping <double>
 *         -ad, -angularDamping <double>
 *         -f, -friction <double>
 *         -re, -restitution <double>
 *         -g, -group <int>         PMX collision group 0..15 (clamped)
 *         -msk, -mask <int>        collide-with mask: bit i set = collides with
 *                                  group i (the PMX non_collision_group field
 *                                  stored verbatim; written into
 *                                  bodies[i].bodyMaskGroup0..15).  Default 0xFFFF.
 *         -pm, -physicsMode <followBone|physics|physicsBone>
 *
 * Each -create appends one bodies[i] element: the body DATA, the related
 * joint as a MESSAGE (bodies[i].bodyJoint), and — for a FOLLOW_BONE body —
 * the kinematic-anchor INPUT (joint.worldMatrix[0] →
 * bodies[i].bodyAnchorWorld; a boneless FOLLOW_BONE body pins its rest world
 * instead).  A dynamic body on a bone ALWAYS gets outTranslate/outRotate
 * connected STRAIGHT into the joint (PHYSICS_BONE is rotation-only); the node
 * computes the joint-local pose itself and derives the write-back offset
 * K = jointRestWorld * bodyRestWorld^-1 at world build from the joints'
 * pmxRest attributes plus jointOrient.  Static colliders get no wiring.
 */

#pragma once

#include <maya/MPxCommand.h>

class MSyntax;
class MArgList;

class RigidBodyCmd : public MPxCommand
{
  public:
    static constexpr const char* kName = "pmxRigidBody";

    static void* creator();
    static MSyntax syntaxCreator();

    MStatus doIt(const MArgList& args) override;
    bool isUndoable() const override { return false; }
};
