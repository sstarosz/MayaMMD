"""
rigid_body_builder.py — rigid bodies for PMX models.

The single rigid-body module (formerly split across the Phase-1 visual-guide
builder and physics_builder.py).  It owns:

* the **rigid-body build functions** that create one native ``mmdPhysicsNode``
  (embedded Bullet) per model: the node's ``bodies`` / ``joints`` compound
  arrays are filled through the native ``mmdRigidBody`` and
  ``mmdRigidBodyConstraint`` commands — there is no Python wiring.
  FOLLOW_BONE bodies are bound to their related joint through the
  kinematic-anchor input, and the NODE draws the colliders itself through its
  draw override — no guide transforms exist.

SIMULATION IS DISABLED: the node is not time-driven (no ``time1.outTime``
connection) and no write-back wiring is created — import only stores the
bodies / joints data and displays the colliders from their rest pose.  The
headless stepping helpers (:func:`step_physics`, :func:`write_back_physics`)
remain for when the simulation is re-enabled.

Run it by calling :func:`create_physics_from_pmx_data` (``build_pmx_scene``
builds physics for every model automatically).  The scene is the source of
truth: reconstruct physics state later with the ``mmd.maya.pmx_model_utils``
discovery helpers (wrapped by ``ModelContext.physics*`` getters).

This module is part of the mmd.maya.pmx package and runs inside Autodesk Maya
(requires maya.api.OpenMaya, maya.cmds).
"""

from __future__ import annotations

import logging
from typing import Optional, Sequence

import maya.api.OpenMaya as om
import maya.cmds as cmds

from mmd.core.data_types import PhysicsMode, PmxModel, ShapeType, Vec3
from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


# ===========================================================================
# Physics binding — one native mmdPhysicsNode (embedded Bullet) per model
# ===========================================================================

_NODE_TYPE = "mmdPhysicsNode"

# PMX shape -> mmdRigidBody -shape name (the native command owns the enum).
_COLLIDER_NAME: dict[ShapeType, str] = {
    ShapeType.SPHERE: "sphere",
    ShapeType.BOX: "box",
    ShapeType.CAPSULE: "capsule",
}

# PMX physics mode -> mmdRigidBody -physicsMode name.
_PHYSICS_MODE_NAME: dict[PhysicsMode, str] = {
    PhysicsMode.FOLLOW_BONE: "followBone",
    PhysicsMode.PHYSICS: "physics",
    PhysicsMode.PHYSICS_BONE: "physicsBone",
}

# Gravity — MMD's physics engine uses exactly -9.8 (Bullet's default) in the
# model's own unit scale.  We must match that: using -98 (a 10x guess for the
# "18-unit" Tololo model) made EVERY force 10x too strong — the huge PMX hair
# masses (3276.8 at the root) × 10x gravity overloaded the rigid-weld
# constraints, so hair/skirt chains sagged visibly (a rigid bang extended ~1.1
# units) and collision pushes were 10x too violent.  -9.8 matches MMD exactly.
_DEFAULT_GRAVITY_Y = -9.8

# NOTE: the node derives dt from the scene's CURRENT time unit itself (MTime →
# seconds in mmd_physics_node.cpp compute()), so there is no Python fps
# plumbing anymore — it adapts automatically if the playback rate changes.

# Collision-mask resolution lives in the NODE (Phase 2): Python feeds the raw
# PMX data (bodyGroupId + bodyNonCollisionGroup) and the C++ node derives the
# effective group bit + mask with the same proximity + cloth-on-cloth
# corrections — mmd/maya/nodes/mmd_physics_masks.h is the exact port of the
# former _compute_collision_masks (see that header for the MMD-intent reasoning).


def _joint_names_for(joints: Sequence[om.MObject]) -> dict[int, str]:
    """Map PMX bone index -> full joint path name (for body→bone bindings).

    ``joints`` is the list of joint MObjects in PMX bone order produced by the
    bone builder; null entries (bones that failed to create) are skipped.
    """
    names: dict[int, str] = {}
    for b_idx, j_obj in enumerate(joints):
        if not j_obj.isNull():
            try:
                names[b_idx] = om.MFnDagNode(j_obj).fullPathName()
            except Exception as e:
                log.debug("Could not resolve joint %d path: %s", b_idx, e)
    return names


# ---------------------------------------------------------------------------
# Build functions — pure Maya-object creation (no class).  The scene is the
# source of truth; reconstruct handles with mmd.maya.pmx_model_utils discovery
# (wrapped by ModelContext.physics* getters).
# ---------------------------------------------------------------------------


def _create_physics_group(
    name_registry: PMXNamingManager, root_transform_obj: Optional[om.MObject]
) -> str:
    """Create the ``{model}_Physics`` transform group under the model root."""
    group_name = name_registry.get_physics_group_name()
    parent_name = None
    if root_transform_obj is not None and not root_transform_obj.isNull():
        try:
            parent_name = om.MFnDependencyNode(root_transform_obj).name()
        except Exception:
            parent_name = None
    if parent_name:
        return cmds.createNode("transform", name=group_name, parent=parent_name)
    return cmds.createNode("transform", name=group_name)


def _create_physics_solver(
    name_registry: PMXNamingManager, parent_group: Optional[str] = None
) -> str:
    """Create the ``mmdPhysicsNode`` (a locator shape) under the physics group.

    The node is an ``MPxLocatorNode``: it owns the Bullet world AND draws its
    own guide visualization (wireframe box/sphere/capsule per body, colored by
    collision group) through a C++ draw override — no scene guide meshes.  It
    is parented under the physics group at the origin, so the Bullet world runs
    in the group's local space and the guides are drawn there.

    SIMULATION IS DISABLED: the node is NOT connected to ``time1.outTime``,
    so it never evaluates or steps — it only stores the bodies / joints data
    and displays the colliders (the draw override falls back to the plugs' rest
    poses when the world was never built).
    """
    solver_name = name_registry.get_physics_solver_name()
    if parent_group:
        node = cmds.createNode(_NODE_TYPE, name=solver_name, parent=parent_group)
    else:
        node = cmds.createNode(_NODE_TYPE, name=solver_name)
    # Simulation is DISABLED: the node is not time-driven (no time1.outTime →
    # node.time connection) so it never steps — it only holds the bodies /
    # joints DATA and displays the colliders from their rest pose.
    cmds.setAttr(f"{node}.gravity", 0.0, _DEFAULT_GRAVITY_Y, 0.0)
    # dt is derived inside the C++ node from the scene's current time unit
    # (MTime → seconds), so there is no fps attribute to configure — it adapts
    # automatically if the playback rate changes.
    return node


# The mmdPhysicsNode DRAWS the colliders and, when the simulation is
# re-enabled, writes the solved pose directly into the related joints.  The
# write-back math (K / M_parent offsets, DG fallbacks) lives in the native
# mmdRigidBody command — see mmd/maya/cmds/mmd_rigid_body_cmd.cpp and
# docs/PhysicsImplementation.md.


def _vec3(v: Optional[Vec3]) -> tuple[float, float, float]:
    """Normalize a PMX vector child to ``(x, y, z)`` (zeros when ``None``)."""
    if v is None:
        return (0.0, 0.0, 0.0)
    return (v.x, v.y, v.z)


# TODO remove in the future
def step_physics(node: Optional[str]) -> None:
    """Force a fresh solver evaluation at the current time (headless use).

    Only needed for headless/batch use (or to manually advance the sim) once
    the simulation is re-enabled — it is currently DISABLED at import (the
    node is not time-driven and no outputs are connected).

    The node is an ``MPxLocatorNode``; a bare ``dgeval(node)`` does NOT reliably
    pull its custom solver outputs (it evaluates the DAG shape, not the
    ``outTranslate``/``outRotate`` plugs — verified with the Phase 1 locator
    conversion, where the sim only advanced when a guide transform was read).
    Demanding an output plug explicitly forces ``compute()`` to run.
    """
    if not node:
        return
    try:
        cmds.dgdirty(node)
        cmds.dgeval(f"{node}.outTranslate")
    except Exception:
        try:
            cmds.dgeval(node)
        except Exception as e:
            log.debug("physics step dgeval failed: %s", e)


# TODO remove in the future
def write_back_physics(
    node: Optional[str], driven_joints: Optional[dict[int, str]] = None
) -> None:
    """Propagate the solved pose to the driven joints (headless use).

    The node writes the joint-local pose straight into the joints, so
    "write-back" is just stepping the solver (:func:`step_physics`) and
    re-evaluating the driven joints.  This exists for headless/batch stepping
    when the simulation is re-enabled (currently disabled at import).
    """
    step_physics(node)
    for joint in (driven_joints or {}).values():
        try:
            cmds.dgdirty(joint)
            cmds.dgeval(joint)
        except Exception as e:
            log.debug("physics joint write_back failed: %s", e)


def create_physics_from_pmx_data(
    pmx_data: PmxModel,
    joints: Sequence[om.MObject],
    name_registry: PMXNamingManager,
    root_transform_obj: Optional[om.MObject] = None,
) -> Optional[str]:
    """Build the rigid-body data for a PMX model (no in-memory handle).

    Creates the ``{model}_Physics`` group and the ``mmdPhysicsNode`` solver,
    then fills the ``bodies`` and ``joints`` arrays through the native
    ``mmdRigidBody`` / ``mmdRigidBodyConstraint`` commands (the single
    body-modification path — no Python wiring).  FOLLOW_BONE bodies are bound
    to their related joint via the kinematic-anchor input, and the node draws
    the colliders from their rest pose.  SIMULATION IS DISABLED: no write-back
    wiring and no solver stepping (import cannot drive any joint).

    The SCENE is the source of truth: discover the built graph later with
    ``mmd.maya.pmx_model_utils`` (wrapped by ``ModelContext.physics*``
    getters) — no handle is kept in memory.

    Args:
        pmx_data:            Parsed PMX model.
        joints:              Joint MObjects in PMX bone order (from bone builder).
        name_registry:       Naming manager for unique names.
        root_transform_obj:  MObject the physics group is parented under.

    Returns:
        The solver node name (so the caller can stamp ``pmxPhysicsNode`` on
        the root), or ``None`` if the model has no rigid bodies.
    """
    group = _create_physics_group(name_registry, root_transform_obj)
    # The solver is a locator shape parented under the physics group — its
    # object space is the group's local space, which is the Bullet world frame.
    node = _create_physics_solver(name_registry, parent_group=group)
    joint_names = _joint_names_for(joints)

    # 1) RIGID BODIES — every body through the native mmdRigidBody command
    #    (create is the default mode): DATA + bone binding.  SIMULATION IS
    #    DISABLED — no write-back wiring, no solver stepping, so import
    #    cannot drive (or explode) any joint.  Bodies are appended in PMX
    #    order (the command auto-increments the index to the PMX rigid-body
    #    index the constraints below reference) and display from their rest
    #    pose on the correct bone.
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        size = body.shape_size
        bone = ""
        if body.related_bone_index >= 0 and body.related_bone_index in joint_names:
            bone = joint_names[body.related_bone_index]
        try:
            cmds.mmdRigidBody(
                node,
                name=body.name_local or "",
                nameUniversal=body.name_universal or "",
                bone=bone,
                shape=_COLLIDER_NAME.get(body.shape, "sphere"),
                size=(size.x, size.y, size.z),
                position=(
                    body.shape_position.x,
                    body.shape_position.y,
                    body.shape_position.z,
                ),
                rotation=(
                    body.shape_rotation.x,
                    body.shape_rotation.y,
                    body.shape_rotation.z,
                ),
                mass=body.mass,
                linearDamping=body.move_attenuation,
                angularDamping=body.rotation_damping,
                friction=body.friction_force,
                restitution=body.repulsion,
                group=body.group_id,
                nonCollisionGroup=body.non_collision_group & 0xFFFF,
                physicsMode=_PHYSICS_MODE_NAME.get(body.physics_mode, "physics"),
            )
        except Exception as exc:
            log.warning("Could not create body %d: %s", rb_idx, exc)

    # 2) RIGID BODY CONSTRAINTS — every PMX joint through the native
    #    mmdRigidBodyConstraint command (the C++ replacement for the former
    #    _set_joint_attributes).  PMX joints are CONSTRAINTS between rigid
    #    bodies (rigid_body_index_a/b), so they come after the bodies they
    #    reference.
    for jt_idx, joint in enumerate(pmx_data.joints):
        try:
            cmds.mmdRigidBodyConstraint(
                node,
                bodyA=int(joint.rigid_body_index_a),
                bodyB=int(joint.rigid_body_index_b),
                type=int(joint.type.value),
                position=(joint.position.x, joint.position.y, joint.position.z),
                rotation=(joint.rotation.x, joint.rotation.y, joint.rotation.z),
                linearMin=_vec3(joint.position_min),
                linearMax=_vec3(joint.position_max),
                angularMin=_vec3(joint.rotation_min),
                angularMax=_vec3(joint.rotation_max),
                linearSpring=_vec3(joint.position_spring_constant),
                angularSpring=_vec3(joint.rotation_spring_constant),
            )
        except Exception as exc:
            log.warning("Could not create joint %d: %s", jt_idx, exc)

    log.info(
        "Physics: %d FOLLOW_BONE, %d dynamic bodies, %d joints (simulation disabled)",
        sum(
            1
            for b in pmx_data.rigid_bodies
            if b.physics_mode == PhysicsMode.FOLLOW_BONE
        ),
        sum(
            1
            for b in pmx_data.rigid_bodies
            if b.physics_mode != PhysicsMode.FOLLOW_BONE
        ),
        len(pmx_data.joints),
    )
    return node
