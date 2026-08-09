/*
 * SPDX-License-Identifier: MIT
 *
 * mmd_rigid_body_cmd.h
 *
 * MmdRigidBodyCmd — native C++ command for operating on an mmdPhysicsNode.
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
 *     mmdRigidBody <solver | modelRoot>
 *         -index <int>              optional target index (must be the next
 *                                   free index; omit to auto-append)
 *         -name <string>            PMX body name (local) → bodies[i].bodyNameLocal
 *         -nameUniversal <string>   PMX body name (universal) → bodies[i].bodyNameUniversal
 *         -bone <joint | pmxBoneIdx>  related joint (Maya name/path or PMX
 *                                   bone index) — drives the bone binding
 *         -shape <sphere|box|capsule>
 *         -size <x y z>             PMX shape size (radius / extents / length)
 *         -position <x y z>         PMX shape position (MMD space; Z-flip applied)
 *         -rotation <x y z>         PMX shape rotation (MMD radians; handedness flip)
 *         -mass <double>
 *         -linearDamping <double>
 *         -angularDamping <double>
 *         -friction <double>
 *         -restitution <double>
 *         -group <int>
 *         -nonCollisionGroup <int>
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

#include "mmd_common.h"

class MSyntax;
class MArgList;
class MArgParser;
class MObject;
class MPlug;
class MDagPath;
class MFnDependencyNode;
class MMatrix;

class MmdRigidBodyCmd : public MPxCommand
{
  public:
    static constexpr const char* kName = "mmdRigidBody";

    MmdRigidBodyCmd() = default;
    ~MmdRigidBodyCmd() override = default;

    static void* creator();
    static MSyntax syntaxCreator();

    MStatus doIt(const MArgList& args) override;
    bool isUndoable() const override { return false; }

  private:
    // Resolve *target* to an mmdPhysicsNode MObject (direct node or model root).
    static bool resolveSolver(const MString& target, MObject& outNode);
    // Create mode: append one body (data + bone binding); returns the index.
    MStatus doCreate(const MArgParser& parser, const MObject& solverNode, int& outIndex);

    // ------------------------------------------------------------------
    // Helpers (implemented in the .cpp)
    // ------------------------------------------------------------------
    // 4x4 row-vector matrix from translate + XYZ euler degrees.
    static MMatrix matrixFromTR(const Double3& t, const Double3& r);
    // A DAG node's world (inclusive) matrix.
    static MMatrix worldMatrix(const MDagPath& path);
    // Connect src → dst, replacing any existing source on dst.
    static MStatus connectOrReplace(const MPlug& src, const MPlug& dst);
    // A joint's stored PMX bone index (pmxBoneData), or -1.
    static int jointPmxBoneIndex(const MDagPath& jointPath);
    // Resolve the -bone argument to a joint dag path (or leave it empty).
    static MDagPath resolveBone(const MString& bone, const MDagPath& groupPath);
};
