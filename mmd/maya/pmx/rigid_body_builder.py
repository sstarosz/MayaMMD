"""
rigid_body_builder.py — rigid bodies for PMX models.

Creates the native ``pmxPhysicsNode`` (embedded Bullet) for a PMX model.

MILESTONE (this PR): the ``{model}_Physics`` group, one ``pmxPhysicsNode``
per model, gravity, and the ``bodies`` + ``joints`` compound arrays
POPULATED through the native ``pmxRigidBody`` and ``pmxRigidBodyConstraint``
commands — one entry per PMX rigid body (data + bone binding for FOLLOW_BONE
bodies via the kinematic-anchor input) and one per PMX joint (rigid-body
constraint data).  SIMULATION IS DISABLED: the ``time`` input is NOT
connected and no write-back wiring happens — the bodies and constraints are
present and inspectable, and the solver has nothing to step.

The node is an ``MPxLocatorNode`` (a locator shape) that owns a Maya-free
Bullet world from ``mmd/core``.  It is parented under the physics group at
the origin, so the Bullet world runs in the group's local space — the same
layout the full body population later fills in.

Called from ``build_pmx_scene`` so every imported model gets its node with
bodies.  The scene is the source of truth: the solver node name is stamped
on the model root (``pmxPhysicsNode`` string attribute) so discovery can
find it directly.

This module is part of the mmd.maya.pmx package and runs inside Autodesk Maya
(requires maya.api.OpenMaya, maya.cmds).
"""

from __future__ import annotations

import logging
from typing import Optional, Sequence

import maya.api.OpenMaya as om
import maya.cmds as cmds

from mmd.core.data_types import PhysicsMode, PmxModel, ShapeType
from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


# ===========================================================================
# Physics binding — one native pmxPhysicsNode (embedded Bullet) per model
# ===========================================================================

_NODE_TYPE = "pmxPhysicsNode"

# PMX shape -> pmxRigidBody -shape name (the native command owns the enum).
# Keyed by enum VALUE (int), not the enum class: mayapy can load mmd.core twice
# (the plugin's sys.path insert uses a different path spelling than the test
# runner), which produces two distinct ShapeType/PhysicsMode classes that are
# not hash-equal — int keys are immune to that.
_COLLIDER_NAME: dict[int, str] = {
    ShapeType.SPHERE.value: "sphere",
    ShapeType.BOX.value: "box",
    ShapeType.CAPSULE.value: "capsule",
}

# PMX physics mode -> pmxRigidBody -physicsMode name.
_PHYSICS_MODE_NAME: dict[int, str] = {
    PhysicsMode.FOLLOW_BONE.value: "followBone",
    PhysicsMode.PHYSICS.value: "physics",
    PhysicsMode.PHYSICS_BONE.value: "physicsBone",
}

# Gravity — MMD's physics engine uses exactly -9.8 (Bullet's default) in the
# model's own unit scale.  We must match that: using -98 (a 10x guess) made
# EVERY force 10x too strong — the huge PMX hair masses (3276.8 at the root)
# times 10x gravity overloaded the rigid-weld constraints, so hair/skirt
# chains sagged visibly and collision pushes were 10x too violent.  -9.8
# matches MMD exactly.
_DEFAULT_GRAVITY_Y = -9.8


# ---------------------------------------------------------------------------
# Build functions — pure Maya-object creation (no class).  The scene is the
# source of truth; reconstruct handles later with the model_utils discovery
# helpers (wrapped by ModelContext.physics* getters).
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
    """Create the ``pmxPhysicsNode`` (a locator shape) under the physics group.

    The node is an ``MPxLocatorNode``: it owns the Bullet world (``mmd/core``
    Simulation) and will draw its own guide visualization through a C++ draw
    override (planned, redesigned).  It is parented under the physics group at
    the origin, so the Bullet world runs in the group's local space.

    NOTE: ``time1.outTime`` is intentionally NOT connected yet — SIMULATION
    IS DISABLED (body/joint data only), and a ``time`` connection would make
    compute() step an unwired world every frame.  The simulation-wiring PR
    connects it together with the physics group's world inverse once into
    ``groupInverseWorldMatrix`` and the Phase-3 write-back.
    """
    solver_name = name_registry.get_physics_solver_name()
    if parent_group:
        node = cmds.createNode(_NODE_TYPE, name=solver_name, parent=parent_group)
    else:
        node = cmds.createNode(_NODE_TYPE, name=solver_name)
    cmds.setAttr(f"{node}.gravity", 0.0, _DEFAULT_GRAVITY_Y, 0.0)
    return node


def _joint_names_for(joints: Sequence[om.MObject]) -> dict[int, str]:
    """Map PMX bone index -> full joint path name (for body->bone bindings).

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


def _populate_rigid_bodies(
    node: str, pmx_data: PmxModel, joint_names: dict[int, str]
) -> None:
    """Append one body per PMX rigid body through the native ``pmxRigidBody`` command.

    Data + bone binding only — SIMULATION IS DISABLED (no write-back, no
    time connection).  Bodies are appended in PMX order so the body index
    matches the PMX rigid-body index that the constraint command references.
    FOLLOW_BONE bodies get their kinematic-anchor input here; dynamic bodies
    are data-only.
    """
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        size = body.shape_size
        bone = ""
        if body.related_bone_index >= 0 and body.related_bone_index in joint_names:
            bone = joint_names[body.related_bone_index]
        try:
            cmds.pmxRigidBody(
                node,
                name=body.name_local or "",
                nameUniversal=body.name_universal or "",
                bone=bone,
                shape=_COLLIDER_NAME.get(body.shape.value, "sphere"),
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
                # The PMX non_collision_group field IS the "collides with"
                # mask (bit i set = the body collides with group i) — MMD feeds
                # it to Bullet directly.  Use it verbatim; do NOT invert.
                mask=body.non_collision_group & 0xFFFF,
                physicsMode=_PHYSICS_MODE_NAME.get(body.physics_mode.value, "physics"),
            )
        except Exception as exc:
            log.warning("Could not create body %d: %s", rb_idx, exc)


def _populate_rigid_body_constraints(node: str, pmx_data: PmxModel) -> None:
    """Append one joint per PMX rigid-body constraint through the native
    ``pmxRigidBodyConstraint`` command.

    PMX joints are CONSTRAINTS between rigid bodies (``rigid_body_index_a`` /
    ``rigid_body_index_b``), so this MUST run after ``_populate_rigid_bodies``
    (the command validates the referenced body indices against the node's
    current body count).  Joints are appended in PMX order so the joint index
    matches the PMX joint index.  Data only — SIMULATION IS DISABLED.
    """
    for jt_idx, joint in enumerate(pmx_data.joints):
        try:
            cmds.pmxRigidBodyConstraint(
                node,
                name=joint.name_local or "",
                nameUniversal=joint.name_universal or "",
                bodyA=int(joint.rigid_body_index_a),
                bodyB=int(joint.rigid_body_index_b),
                type=int(joint.type.value),
                position=(joint.position.x, joint.position.y, joint.position.z),
                rotation=(joint.rotation.x, joint.rotation.y, joint.rotation.z),
                linearMin=(
                    joint.position_min.x,
                    joint.position_min.y,
                    joint.position_min.z,
                ),
                linearMax=(
                    joint.position_max.x,
                    joint.position_max.y,
                    joint.position_max.z,
                ),
                angularMin=(
                    joint.rotation_min.x,
                    joint.rotation_min.y,
                    joint.rotation_min.z,
                ),
                angularMax=(
                    joint.rotation_max.x,
                    joint.rotation_max.y,
                    joint.rotation_max.z,
                ),
                linearSpring=(
                    joint.position_spring_constant.x,
                    joint.position_spring_constant.y,
                    joint.position_spring_constant.z,
                ),
                angularSpring=(
                    joint.rotation_spring_constant.x,
                    joint.rotation_spring_constant.y,
                    joint.rotation_spring_constant.z,
                ),
            )
        except Exception as exc:
            log.warning("Could not create joint %d: %s", jt_idx, exc)


def create_physics_from_pmx_data(
    pmx_data: PmxModel,
    joints: Sequence[om.MObject],
    name_registry: PMXNamingManager,
    root_transform_obj: Optional[om.MObject] = None,
) -> Optional[str]:
    """Create the physics graph for a PMX model (no in-memory handle).

    MILESTONE: creates the ``{model}_Physics`` group and one ``pmxPhysicsNode``
    solver under it — per model — and POPULATES the ``bodies`` array through
    the native ``pmxRigidBody`` command (one body per PMX rigid body, in PMX
    order) and the ``joints`` array through the native
    ``pmxRigidBodyConstraint`` command (one joint per PMX rigid-body
    constraint, in PMX order).  SIMULATION IS DISABLED: the ``time`` input is
    NOT connected and there is no write-back wiring — the bodies and
    constraints are present and inspectable, but the solver has nothing to
    step.

    Args:
        pmx_data:            Parsed PMX model (rigid bodies + joints).
        joints:              Joint MObjects in PMX bone order (from bone
                             builder; used to bind FOLLOW_BONE bodies).
        name_registry:       Naming manager for unique names.
        root_transform_obj:  MObject the physics group is parented under.

    Returns:
        The solver node name (the caller stamps it on the model root), or
        ``None`` if the node could not be created.
    """
    group = _create_physics_group(name_registry, root_transform_obj)
    # The solver is a locator shape parented under the physics group — its
    # object space is the group's local space, which is the Bullet world frame.
    try:
        node = _create_physics_solver(name_registry, parent_group=group)
    except Exception as e:  # pragma: no cover - Maya-side failure path
        log.warning(
            "Could not create physics node for %s: %s",
            name_registry.get_model_name(),
            e,
        )
        return None
    joint_names = _joint_names_for(joints)
    _populate_rigid_bodies(node, pmx_data, joint_names)
    # Constraints reference bodies by index, so they come AFTER every body.
    _populate_rigid_body_constraints(node, pmx_data)
    return node
