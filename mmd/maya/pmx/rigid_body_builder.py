"""
rigid_body_builder.py — rigid bodies for PMX models.

Creates the native ``pmxPhysicsNode`` (embedded Bullet) for a PMX model.

MILESTONE (this PR): the ``{model}_Physics`` group, one ``pmxPhysicsNode``
per model, gravity, and the ``bodies`` + ``joints`` compound arrays
POPULATED through the native ``pmxRigidBody`` and ``pmxRigidBodyConstraint``
commands — one entry per PMX rigid body (data + bone binding for FOLLOW_BONE
bodies via the kinematic-anchor input) and one per PMX joint (rigid-body
constraint data).  SIMULATION IS ENABLED: the solver is driven by
``time1.outTime`` and the solved pose is written STRAIGHT into the related
joints (Phase 3 direct write-back: ``boneLocal = K · bodyLocal ·
B_parent⁻¹ · M_parent⁻¹``) — there is no separate finalize step; import
wires everything in one pass.  The headless stepping helper
(:func:`step_physics`) remains for batch use.

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
    """Create the ``pmxPhysicsNode`` (a locator shape) and make it time-driven.

    The node is an ``MPxLocatorNode``: it owns the Bullet world (``mmd/core``
    Simulation) and draws its own guide visualization through a C++ draw
    override (planned, redesigned).  The Bullet world runs in WORLD space, so
    the solver's own location never matters.

    Connecting ``time1.outTime`` makes the evaluation manager step the solver
    every frame (the same path as a parentConstraint, so it works under
    Cached Playback — the node also declares itself non-cacheable via
    getCacheSetup).  An empty bodies/joints world is a valid no-op, so
    connecting time before the arrays are populated is safe.
    """
    solver_name = name_registry.get_physics_solver_name()
    if parent_group:
        node = cmds.createNode(_NODE_TYPE, name=solver_name, parent=parent_group)
    else:
        node = cmds.createNode(_NODE_TYPE, name=solver_name)
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
    = jointRestWorld * bodyRestWorld^-1) baked by the command.  The solver is
    wired and time-driven later (see :func:`_wire_dynamic_write_back`).

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
    matches the PMX joint index.  Data only — the solver is wired and stepped
    later (see :func:`_wire_dynamic_write_back`).
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
# Phase 3 direct write-back — the node writes the solved JOINT-LOCAL pose
# straight into the related joints (no guide transforms, no -finalize step).
#   boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1
#   K        = jointRestWorld * bodyRestWorld^-1              (bodyWriteBackOffset)
#   M_parent = K[parentBodyIndex]                             (the same constant as the
#               parent body's K — no separate parent-offset array)
#
# K is baked by the native pmxRigidBody -create command (it knows the related
# joint and the body rest); this module only resolves the parent body index,
# the scrub-back reset anchors and the output connections.  Bodies whose
# parent bone has no rigid body are left undriven (the old DG fallback is
# gone).
# ---------------------------------------------------------------------------


def _compute_reset_anchor_map(
    pmx_data: PmxModel, kinematic_order: list[int]
) -> dict[int, int]:
    """Map each dynamic body to the anchor that drives its scrub-back reset.

    When time is scrubbed backwards the C++ node teleports dynamic bodies to
    their rest pose transformed by the CURRENT skeleton pose.  For each dynamic
    body we use the anchor of its NEAREST KINEMATIC ANCESTOR bone (walking the
    PMX parent chain), so hair uses the head anchor, skirt uses the pelvis
    anchor, etc.

    Returns ``{rb_index: anchor_index}`` (anchor_index = position in
    ``kinematic_order``); dynamic bodies without a kinematic ancestor are
    omitted (no reset).
    """
    follow_bone = PhysicsMode.FOLLOW_BONE.value
    bone_to_anchor: dict[int, int] = {}
    for a, rb_idx in enumerate(kinematic_order):
        rb = pmx_data.rigid_bodies[rb_idx]
        if rb.related_bone_index >= 0:
            bone_to_anchor.setdefault(rb.related_bone_index, a)

    def _find_anchor(bone_idx: int) -> int:
        seen: set[int] = set()
        while bone_idx >= 0 and bone_idx not in seen:
            seen.add(bone_idx)
            if bone_idx in bone_to_anchor:
                return bone_to_anchor[bone_idx]
            if bone_idx >= len(pmx_data.bones):
                return -1
            bone_idx = pmx_data.bones[bone_idx].parentIndex
        return -1

    result: dict[int, int] = {}
    for rb_idx, rb in enumerate(pmx_data.rigid_bodies):
        if rb.physics_mode.value == follow_bone or rb.related_bone_index < 0:
            continue
        anchor = _find_anchor(rb.related_bone_index)
        if anchor >= 0:
            result[rb_idx] = anchor
    return result


def _wire_dynamic_write_back(
    node: str,
    pmx_data: PmxModel,
    joint_names: dict[int, str],
    kinematic_order: list[int],
) -> None:
    """Drive the related JOINTS from the node's solved pose (Phase 3).

    Called AFTER every body and joint exists (no -finalize step).  The body
    data and the write-back K offsets (``bodyWriteBackOffset`` =
    jointRestWorld * bodyRestWorld^-1) were already baked by ``pmxRigidBody``;
    here we only resolve the per-body wiring that needs the WHOLE model:

    * ``bodies[i].bodyParentBodyIndex`` — the parent bone's rigid body, so the
      node derives the parent joint's world from the PARENT BODY's solved
      Bullet transform (M_parent = K[parentBodyIndex]) with no DG dependency on
      node-driven parent joints (that was the feedback cycle that exploded the
      sim).  Bodies whose parent bone has no rigid body are left UNDRIVEN (the
      old DG ``bodyParentInverseMatrix`` fallback is gone);
    * ``bodies[i].bodyResetAnchorIndex`` for scrub-back rewind (nearest
      kinematic ancestor);
    * ``outTranslate``/``outRotate`` -> joint.translate/rotate — LAST, so the
      first evaluation (triggered by these connections) sees complete data.
      PHYSICS_BONE (mode 2) is rotation-only.
    """
    follow_bone = PhysicsMode.FOLLOW_BONE.value
    physics_bone = PhysicsMode.PHYSICS_BONE.value

    # PMX bone index -> rigid-body index (only bodies with a related joint can
    # be referenced as a write-back parent).
    bone_of_body: dict[int, int] = {}
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        if body.related_bone_index >= 0 and body.related_bone_index in joint_names:
            bone_of_body.setdefault(body.related_bone_index, rb_idx)

    # Parent body resolution (needs the WHOLE model: the parent body may be
    # created later in the array).  A body whose parent bone has no rigid body
    # (parent_rb = -1) is left UNDRIVEN — the old DG
    # ``bodyParentInverseMatrix`` fallback is gone, so the node cannot write a
    # joint-local pose for it.
    parent_body: dict[int, int] = {}
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        if body.physics_mode.value == follow_bone:
            continue
        bone_idx = body.related_bone_index
        if bone_idx < 0 or bone_idx not in joint_names:
            continue  # no related joint -> static collider, no write-back
        parent_rb = -1
        if (
            0 <= bone_idx < len(pmx_data.bones)
            and pmx_data.bones[bone_idx].parentIndex >= 0
        ):
            parent_rb = bone_of_body.get(pmx_data.bones[bone_idx].parentIndex, -1)
        parent_body[rb_idx] = parent_rb
        cmds.setAttr(f"{node}.bodies[{rb_idx}].bodyParentBodyIndex", int(parent_rb))

    # Scrub-back reset anchors (dynamic body -> nearest kinematic ancestor).
    for rb_idx, anchor_idx in _compute_reset_anchor_map(
        pmx_data, kinematic_order
    ).items():
        cmds.setAttr(f"{node}.bodies[{rb_idx}].bodyResetAnchorIndex", int(anchor_idx))

    # Solved pose -> joints LAST (triggers the first evaluation, so every
    # input above is already in place).  Only bodies with a parent body are
    # driven — the node derives the parent inverse from the PARENT BODY's
    # solved transform, and the old DG fallback for no-parent-body bodies is
    # gone, so those joints stay at their animated pose.
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        if body.physics_mode.value == follow_bone:
            continue
        bone_idx = body.related_bone_index
        if bone_idx < 0 or bone_idx not in joint_names:
            continue
        if parent_body.get(rb_idx, -1) < 0:
            continue  # no parent body -> node cannot write back; leave free
        jpath = joint_names[bone_idx]
        try:
            # Compound-to-compound connections: the node's outTranslate[i] /
            # outRotate[i] children are UNIT-TYPED (kDistance/kAngle, like
            # transform.translate/rotate), so Maya connects them DIRECTLY to
            # joint.translate/rotate with NO auto-inserted unitConversion.
            if body.physics_mode.value != physics_bone:
                cmds.connectAttr(
                    f"{node}.outTranslate[{rb_idx}]",
                    f"{jpath}.translate",
                    force=True,
                )
            cmds.connectAttr(
                f"{node}.outRotate[{rb_idx}]",
                f"{jpath}.rotate",
                force=True,
            )
        except Exception as e:
            log.warning(
                "Could not connect dynamic output %d (%s): %s", rb_idx, jpath, e
            )

    # No `caching` override here: the node's getCacheSetup() already declares
    # it non-cacheable (it is STATEFUL — caching its outputs would freeze the
    # sim), so the attribute can stay at its default.


def step_physics(node: Optional[str]) -> None:
    """Force a fresh solver evaluation at the current time (headless use).

    Only needed for headless/batch use (or to manually advance the sim) — in
    interactive Maya the node is time-driven and steps on its own.

    The node is an ``MPxLocatorNode``; a bare ``dgeval(node)`` does NOT
    reliably pull its custom solver outputs (it evaluates the DAG shape, not
    the ``outTranslate``/``outRotate`` plugs).  Demanding an output plug
    explicitly forces ``compute()`` to run.
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
    constraint, in PMX order).  SIMULATION IS ENABLED: the solver is driven
    by ``time1.outTime`` and the solved pose is written STRAIGHT into the
    related joints (Phase 3 direct write-back: ``boneLocal = K · bodyLocal ·
    B_parent⁻¹ · M_parent⁻¹``) — there is no separate finalize step; import
    wires everything in one pass.

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

    # Phase-3 write-back — AFTER every body and joint exists, so the first
    # evaluation (triggered by the output connections) sees complete data.
    kinematic_order = [
        rb_idx
        for rb_idx, b in enumerate(pmx_data.rigid_bodies)
        if b.physics_mode.value == PhysicsMode.FOLLOW_BONE.value
    ]
    _wire_dynamic_write_back(node, pmx_data, joint_names, kinematic_order)
    return node
