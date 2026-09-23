"""
rigid_body_builder.py — rigid bodies for PMX models.

Creates the ``{model}_RigidBodies`` group (with the ``{model}_Bodies`` and
``{model}_Joints`` identity sub-groups) and one ``pmxRigidBodyNode`` solver
per PMX model, then creates one ``pmxRigidBodyShape`` locator node per rigid
body through the native ``pmxRigidBody`` command (one entry per PMX rigid
body / joint, in PMX order), wired into the solver's ``bodyShapes[]`` message
array.  The solver is
time-driven (``time1.outTime``); the node computes the solved joint-local
poses internally (bone-world write-back) and the command wires them straight
into the related joints — there is no separate finalize step.  The Bullet
world runs in WORLD space, so the solver's own location never matters.

Part of the mmd.maya.pmx package; runs inside Autodesk Maya (maya.api.OpenMaya
/ maya.cmds).
"""

from __future__ import annotations

import logging
from collections.abc import Sequence

import maya.api.OpenMaya as om
from maya import cmds

from mmd.core.data_types import PhysicsMode, PmxModel, ShapeType
from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


# Keyed by enum VALUE (int), not the enum member: dev-mode module reloads
# (mmd/plugin.py) recreate mmd.core's enum classes, so the int values are the
# only key stable across reloads.
_COLLIDER_NAME: dict[int, str] = {
    ShapeType.SPHERE.value: "sphere",
    ShapeType.BOX.value: "box",
    ShapeType.CAPSULE.value: "capsule",
}

_PHYSICS_MODE_NAME: dict[int, str] = {
    PhysicsMode.FOLLOW_BONE.value: "followBone",
    PhysicsMode.PHYSICS.value: "physics",
    PhysicsMode.PHYSICS_BONE.value: "physicsBone",
}

# MMD uses exactly -9.8 (Bullet's default) in the model's own unit scale; a
# 10x guess (-98) made every force 10x too strong.
_DEFAULT_GRAVITY_Y = -9.8


def _create_rigid_bodies_group(
    name_registry: PMXNamingManager, root_transform_obj: om.MObject
) -> str:
    """Create the ``{model}_RigidBodies`` transform group under the model root."""
    parent = om.MFnDependencyNode(root_transform_obj).name()
    return cmds.createNode(
        "transform", name=name_registry.get_rigid_bodies_group_name(), parent=parent
    )


def _create_subgroup(parent_group: str, name: str) -> str:
    """Create an IDENTITY transform sub-group under ``parent_group``.

    The ``{model}_Bodies`` / ``{model}_Joints`` sub-groups are pure Outliner
    organisation and MUST stay at identity: the solver drives the per-body
    guide transforms in the ``{model}_RigidBodies`` group's space, so an
    offset between that group and the guides would displace every collider
    in the viewport.
    """
    return cmds.createNode("transform", name=name, parent=parent_group)


def _children(group: str) -> set[str]:
    """Full DAG paths of ``group``'s current direct children."""
    return set(cmds.listRelatives(group, children=True, fullPath=True) or [])


def _move_new_children(parent_group: str, previous: set[str], destination: str) -> None:
    """Move the nodes created under ``parent_group`` since ``previous`` into ``destination``.

    The native commands create every per-body guide / per-joint transform
    directly under their parent group (``{model}_RigidBodies``); the Outliner
    organisation (``{model}_Bodies`` / ``{model}_Joints``) is applied here, in
    one batched ``parent`` call per batch.  ``destination`` is an IDENTITY
    child of ``parent_group``, so the move is done with ``relative=True`` —
    every LOCAL value (the guides' driven translate/rotate, the joints'
    world-frame-compensated transform) stays untouched, and connections (guide
    driving, bodyShapes/jointShapes wiring) survive reparenting.
    """
    created = sorted(_children(parent_group) - previous)
    if not created:
        return
    try:
        cmds.parent(created, destination, relative=True)
    except Exception as exc:  # pragma: no cover - Maya-side failure path
        log.warning(
            "Could not move %d nodes into %s: %s", len(created), destination, exc
        )


def _create_rigid_body_solver(
    name_registry: PMXNamingManager, parent_group: str
) -> str:
    """Create the ``pmxRigidBodyNode`` solver (a locator shape) and make it time-driven.

    The node is an ``MPxLocatorNode``: it owns the Bullet world (``mmd/core``
    RigidBodySimulation) and drives the per-body ``pmxRigidBodyShape`` guides
    (each drawn by a C++ geometry override) to their current pose every frame.

    Connecting ``time1.outTime`` makes the evaluation manager step the solver
    every frame (the same path as a parentConstraint, so it works under
    Cached Playback — the node also declares itself non-cacheable via
    getCacheSetup).  An empty bodies/joints world is a valid no-op, so
    connecting time before the arrays are populated is safe.
    """
    solver_name = name_registry.get_rigid_body_solver_name()
    node = cmds.createNode("pmxRigidBodyNode", name=solver_name, parent=parent_group)
    try:
        cmds.connectAttr("time1.outTime", f"{node}.time")
    except Exception as e:
        log.warning("Could not connect time1 to node time: %s", e)

    try:
        cmds.setAttr(f"{node}.gravity", 0.0, _DEFAULT_GRAVITY_Y, 0.0)
    except Exception as e:
        log.warning("Could not set node gravity: %s", e)
    return node


def _populate_rigid_bodies(
    node: str, pmx_data: PmxModel, joints: Sequence[om.MObject]
) -> int:
    """Append one body per PMX rigid body through the native ``pmxRigidBody`` command.

    Data + bone binding.  Bodies are appended in PMX order so the body index
    matches the PMX rigid-body index that the constraint command references.
    FOLLOW_BONE bodies get their kinematic-anchor INPUT here (the command
    connects joint.worldMatrix -> shape.bodyAnchorWorld); dynamic bodies
    with a related joint get their outTranslate/outRotate connected into the
    related joint at creation (the command always wires a dynamic body on a
    bone).  The write-back K offset (jointRestWorld * bodyRestWorld^-1) is
    NOT baked anymore — the node derives it internally from the joints'
    pmxRest* attributes.  Each body is a ``pmxRigidBodyShape`` guide: the
    guide TRANSFORM is driven to the body's CURRENT pose by the solver each
    frame (so the collider follows the animation), and the REST pose lives in
    the shape's ``bodyRestTranslate``/``bodyRestRotate`` attributes.  The
    caller moves the new guides into the ``{model}_Bodies`` sub-group.

    Returns the number of bodies successfully appended — the caller must
    compare it against ``len(pmx_data.rigid_bodies)``: if a body fails, every
    later PMX body index silently shifts (body i+1 lands at Maya index i-1),
    so the constraints and the write-back would reference WRONG bodies.
    """
    created = 0
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        size = body.shape_size
        bone = ""
        bone_idx = body.related_bone_index
        if 0 <= bone_idx < len(joints) and not joints[bone_idx].isNull():
            try:
                bone = om.MFnDagNode(joints[bone_idx]).fullPathName()
            except Exception as e:
                log.debug("Could not resolve joint %d path: %s", bone_idx, e)
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
                # PMX non_collision_group IS the collide-with mask (bit i set
                # = collides with group i) — pass verbatim, do NOT invert.
                mask=body.non_collision_group & 0xFFFF,
                physicsMode=_PHYSICS_MODE_NAME.get(body.physics_mode.value, "physics"),
            )
            created += 1
        except Exception as exc:
            log.warning("Could not create body %d: %s", rb_idx, exc)
    return created


def _populate_rigid_body_constraints(node: str, pmx_data: PmxModel) -> None:
    """Create one joint per PMX rigid-body constraint via ``pmxRigidBodyConstraint``.

    PMX joints are CONSTRAINTS between rigid bodies (``rigid_body_index_a`` /
    ``rigid_body_index_b``), so this MUST run after ``_populate_rigid_bodies``
    (the command validates the referenced body indices against the node's
    current body count).  Joints are appended in PMX order so the joint index
    matches the PMX joint index.  Each call creates a selectable
    ``pmxRigidBodyJoint`` node — its transform IS the joint frame, so the
    constraint is visible and editable in the viewport — wired into the
    solver's ``jointShapes[]`` message array.  The caller moves the new joint
    nodes into the ``{model}_Joints`` sub-group.

    Unlike bodies, a failed joint needs no count check: nothing downstream
    references joints by index, so a dropped joint just means one constraint
    is missing — later joints are unaffected.
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


def create_rigid_bodies_from_pmx_data(
    pmx_data: PmxModel,
    joints: Sequence[om.MObject],
    name_registry: PMXNamingManager,
    root_transform_obj: om.MObject,
) -> str | None:
    """Create the rigid-body graph for a PMX model (no in-memory handle).

    Creates the ``{model}_RigidBodies`` group with one ``pmxRigidBodyNode``
    solver directly under it plus the ``{model}_Bodies`` / ``{model}_Joints``
    identity sub-groups, then populates the solver through the native
    commands: one ``pmxRigidBody`` per PMX rigid body (guides under
    ``{model}_Bodies``) and one ``pmxRigidBodyConstraint``
    per PMX joint (nodes under ``{model}_Joints``), both in PMX order.
    Everything is wired in this single
    pass — there is no separate finalize step.

    Args:
        pmx_data:            Parsed PMX model (rigid bodies + joints).
        joints:              Joint MObjects in PMX bone order (from bone
                             builder; used to bind FOLLOW_BONE bodies).
        name_registry:       Naming manager for unique names.
        root_transform_obj:  MObject the rigid bodies group is parented under.

    Returns:
        The solver node name (the caller stamps it on the model root), or
        ``None`` if the node could not be created — the (empty) rigid bodies
        group is left in the scene in that case.
    """
    group = _create_rigid_bodies_group(name_registry, root_transform_obj)
    # Outliner organisation: the solver stays at the top of the rigid bodies
    # group; the per-body guides and the per-joint nodes each get their own
    # identity sub-group so bodies and joints are separate in the Outliner.
    bodies_group = _create_subgroup(group, name_registry.get_bodies_group_name())
    joints_group = _create_subgroup(group, name_registry.get_joints_group_name())
    try:
        node = _create_rigid_body_solver(name_registry, parent_group=group)
    except Exception as e:  # pragma: no cover - Maya-side failure path
        log.warning(
            "Could not create rigid body node for %s: %s",
            name_registry.get_model_name(),
            e,
        )
        return None
    bodies_before = _children(group)
    created = _populate_rigid_bodies(node, pmx_data, joints)
    _move_new_children(group, bodies_before, bodies_group)
    # A failed body shifts every later PMX body index — the constraint
    # references would point at the WRONG bodies.
    if created != len(pmx_data.rigid_bodies):
        log.warning(
            "Rigid bodies: created %d/%d rigid bodies — PMX body indices no longer "
            "match Maya body indices, so constraints are unreliable for this model",
            created,
            len(pmx_data.rigid_bodies),
        )
    joints_before = _children(group)
    _populate_rigid_body_constraints(node, pmx_data)
    _move_new_children(group, joints_before, joints_group)
    return node
