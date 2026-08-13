"""
rigid_body_builder.py — rigid bodies for PMX models.

Creates the native ``pmxPhysicsNode`` (embedded Bullet) for a PMX model:
the ``{model}_Physics`` group, one solver per model, gravity, and the
``bodies`` + ``joints`` compound arrays populated through the native
``pmxRigidBody`` and ``pmxRigidBodyConstraint`` commands — one entry per PMX
rigid body (data + bone binding for FOLLOW_BONE bodies via the
kinematic-anchor input) and one per PMX joint (rigid-body constraint data).
The solver is driven by ``time1.outTime`` and the solved pose is written
STRAIGHT into the related joints — the node computes a solved bone world per
bone (``bodyLocal · K``) and divides by the parent bone's solved world via
the bone hierarchy (resolved internally from each body's ``bodyJoint``
message + the joint DAG); there is no separate finalize step; import wires
everything in one pass.

The node is an ``MPxLocatorNode`` (a locator shape) that owns a Maya-free
Bullet world from ``mmd/core``.  The Bullet world runs in WORLD space, so
the solver's own location (and the physics group's transform) never matters —
the user is free to move the skeleton without breaking the simulation.

Called from ``build_pmx_scene`` so every imported model gets its node with
bodies.  The scene is the source of truth: the solver node name is stamped
on the model root (``pmxPhysicsNode`` string attribute) so discovery can
find it directly.

This module is part of the mmd.maya.pmx package and runs inside Autodesk Maya
(requires maya.api.OpenMaya, maya.cmds).
"""

from __future__ import annotations

import logging
from collections.abc import Sequence

import maya.api.OpenMaya as om
from maya import cmds

from mmd.core.data_types import PhysicsMode, PmxModel, ShapeType
from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


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
# source of truth; discovery finds the solver through the ``pmxPhysicsNode``
# root attribute stamped by the caller.
# ---------------------------------------------------------------------------


def _create_physics_group(
    name_registry: PMXNamingManager, root_transform_obj: om.MObject | None
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
    name_registry: PMXNamingManager, parent_group: str | None = None
) -> str:
    """Create the ``pmxPhysicsNode`` (a locator shape) and make it time-driven.

    The node is an ``MPxLocatorNode``: it owns the Bullet world (``mmd/core``
    Simulation) and will draw its own guide visualization through a C++ draw
    override (planned).  The Bullet world runs in WORLD space, so the
    solver's own location never matters.

    Connecting ``time1.outTime`` makes the evaluation manager step the solver
    every frame (the same path as a parentConstraint, so it works under
    Cached Playback — the node also declares itself non-cacheable via
    getCacheSetup).  An empty bodies/joints world is a valid no-op, so
    connecting time before the arrays are populated is safe.
    """
    solver_name = name_registry.get_physics_solver_name()
    if parent_group:
        node = cmds.createNode("pmxPhysicsNode", name=solver_name, parent=parent_group)
    else:
        node = cmds.createNode("pmxPhysicsNode", name=solver_name)
    # Time-driven: the evaluation manager steps the solver every frame (the
    # same path as a parentConstraint, so it works under Cached Playback — the
    # node also declares itself non-cacheable via getCacheSetup).
    try:
        cmds.connectAttr("time1.outTime", f"{node}.time")
    except Exception as e:
        log.warning("Could not connect time1 to node time: %s", e)
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
) -> int:
    """Append one body per PMX rigid body through the native ``pmxRigidBody`` command.

    Data + bone binding.  Bodies are appended in PMX order so the body index
    matches the PMX rigid-body index that the constraint command references.
    FOLLOW_BONE bodies get their kinematic-anchor input here; dynamic bodies
    with a related joint get their write-back K offset (``bodyWriteBackOffset``
    = jointRestWorld * bodyRestWorld^-1) baked by the command and their
    outTranslate/outRotate connected into the related joint at creation (the
    command always wires a dynamic body on a bone).

    Returns the number of bodies successfully appended — the caller must
    compare it against ``len(pmx_data.rigid_bodies)``: if a body fails, every
    later PMX body index silently shifts (body i+1 lands at Maya index i-1),
    so the constraints and the write-back would reference WRONG bodies.
    """
    created = 0
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
            created += 1
        except Exception as exc:
            log.warning("Could not create body %d: %s", rb_idx, exc)
    return created


def _populate_rigid_body_constraints(node: str, pmx_data: PmxModel) -> None:
    """Append one joint per PMX rigid-body constraint through the native
    ``pmxRigidBodyConstraint`` command.

    PMX joints are CONSTRAINTS between rigid bodies (``rigid_body_index_a`` /
    ``rigid_body_index_b``), so this MUST run after ``_populate_rigid_bodies``
    (the command validates the referenced body indices against the node's
    current body count).  Joints are appended in PMX order so the joint index
    matches the PMX joint index.  Data only — the solver is time-driven and
    each body's write-back outputs are wired at creation by the command.
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


# ---------------------------------------------------------------------------
# Write-back — the native pmxRigidBody command owns ALL per-body wiring: it
# connects the body's related joint as a MESSAGE (bodies[i].bodyJoint),
# bakes K (bodyWriteBackOffset) and ALWAYS connects outTranslate/outRotate
# STRAIGHT into the related joint for dynamic bodies (PHYSICS_BONE is
# rotation-only) — every body is created on a bone, so the connection is a
# deterministic consequence of the bone.  The node computes the joint-local
# pose internally via the bone hierarchy (resolved from the message + the
# joint DAG); a body whose parent bone has no rigid body is still connected
# (the node falls back to the raw solved world pose for it — rare in
# well-formed chains).  No post-pass wiring remains in this module.
# ---------------------------------------------------------------------------


def create_physics_from_pmx_data(
    pmx_data: PmxModel,
    joints: Sequence[om.MObject],
    name_registry: PMXNamingManager,
    root_transform_obj: om.MObject | None = None,
) -> str | None:
    """Create the physics graph for a PMX model (no in-memory handle).

    Creates the ``{model}_Physics`` group and one ``pmxPhysicsNode`` solver
    under it — per model — and POPULATES the ``bodies`` array through the
    native ``pmxRigidBody`` command (one body per PMX rigid body, in PMX
    order) and the ``joints`` array through the native
    ``pmxRigidBodyConstraint`` command (one joint per PMX rigid-body
    constraint, in PMX order).  The solver is driven by ``time1.outTime``
    and the solved pose is written STRAIGHT into the related joints — the
    node computes a solved bone world per bone (``bodyLocal · K``) and
    divides by the parent bone's solved world via the bone hierarchy
    (resolved internally from each body's ``bodyJoint`` message + the joint
    DAG) — there is no separate finalize step; import wires everything in
    one pass.

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
    # The solver is a locator shape parented under the physics group.  The
    # Bullet world runs in WORLD space, so the group's transform is irrelevant
    # to the simulation (it is just an organizational container).
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

    # Each body's related joint is connected as a MESSAGE by the native
    # pmxRigidBody command (bodies[i].bodyJoint -> joint.message); the node
    # resolves the bone index and the hierarchy from it + the joint DAG — no
    # dump or wiring needed here.
    created = _populate_rigid_bodies(node, pmx_data, joint_names)
    # A body that failed to create shifts every later PMX body index, which
    # would make the constraint (and future write-back) references point at
    # the WRONG bodies — fail loudly instead of importing a silently-corrupt
    # constraint set.
    if created != len(pmx_data.rigid_bodies):
        log.warning(
            "Physics: created %d/%d rigid bodies — PMX body indices no longer "
            "match Maya body indices, so constraints are unreliable for this model",
            created,
            len(pmx_data.rigid_bodies),
        )
    # Constraints reference bodies by index, so they come AFTER every body.
    _populate_rigid_body_constraints(node, pmx_data)
    return node
