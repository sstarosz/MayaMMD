"""
rigid_body_builder.py — rigid bodies for PMX models.

The single rigid-body module (formerly split across the Phase-1 visual-guide
builder and physics_builder.py).  It owns:

* the **rigid-body build functions** that create one native ``mmdPhysicsNode``
  (embedded Bullet) per model: the node's ``bodies`` / ``joints`` compound
  arrays are filled through the native ``mmdRigidBody`` and
  ``mmdRigidBodyConstraint`` commands, and the node is then wired for
  simulation.  FOLLOW_BONE bodies are bound to their related joint through the
  kinematic-anchor input, and the NODE draws the colliders itself through its
  draw override — no guide transforms exist.

SIMULATION IS ENABLED: the solver is driven by ``time1.outTime`` and the
solved pose is written STRAIGHT into the related joints (Phase 3 direct
write-back: ``boneLocal = K · bodyLocal · B_parent⁻¹ · M_parent⁻¹``) — there
is NO separate finalize step; import wires everything in one pass.  The
headless stepping helpers (:func:`step_physics`, :func:`write_back_physics`)
remain for batch use.

Run it by calling :func:`create_physics_from_pmx_data` (``build_pmx_scene``
builds physics for every model automatically).  The scene is the source of
truth: reconstruct physics state later with the ``mmd.maya.pmx_model_utils``
discovery helpers (wrapped by ``ModelContext.physics*`` getters).

This module is part of the mmd.maya.pmx package and runs inside Autodesk Maya
(requires maya.api.OpenMaya, maya.cmds).
"""

from __future__ import annotations

import logging
import math
from typing import Optional, Sequence

import maya.api.OpenMaya as om
import maya.cmds as cmds

from mmd.core.data_types import PhysicsMode, PmxModel, PMXRigidBody, ShapeType, Vec3
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

# Collision mask: the PMX non_collision_group field IS the "collides with"
# mask (bit i set = the body collides with group i) — MikuMikuDance feeds it to
# Bullet directly, so Python stores it VERBATIM (bodyMaskGroup0..15 bools,
# True = collides with that group) and the C++ node uses it directly.
#
# NOTE (2026-08-09): earlier code inverted it (~non_collision_group), which
# flipped every model's masks into "own group only" (skirt->legs pass-through,
# ghost kinematic colliders); the proximity + cloth-on-cloth corrections that
# used to live in mmd_physics_masks.h were a workaround for that inversion bug
# and have been removed.


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
    """Create the ``mmdPhysicsNode`` (a locator shape) and make it time-driven.

    The node is an ``MPxLocatorNode``: it owns the Bullet world AND draws its
    own guide visualization (wireframe box/sphere/capsule per body, colored by
    collision group) through a C++ draw override — no scene guide meshes.  It
    is parented under the physics group at the origin, so the Bullet world runs
    in the group's local space and the guides are drawn there.  Connecting
    ``time1.outTime`` to its ``time`` input makes the evaluation manager step
    it on every frame.
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
    # dt is derived inside the C++ node from the scene's current time unit
    # (MTime → seconds), so there is no fps attribute to configure — it adapts
    # automatically if the playback rate changes.
    return node


# The mmdPhysicsNode DRAWS the colliders AND writes the solved pose directly
# into the related joints (Phase 3 direct write-back).  The write-back math
# (K / M_parent offsets, DG fallbacks) is baked by :func:`_wire_dynamic_write_back`
# below — see docs/PhysicsImplementation.md.


def _vec3(v: Optional[Vec3]) -> tuple[float, float, float]:
    """Normalize a PMX vector child to ``(x, y, z)`` (zeros when ``None``)."""
    if v is None:
        return (0.0, 0.0, 0.0)
    return (v.x, v.y, v.z)


# ---------------------------------------------------------------------------
# Phase 3 direct write-back — the node writes the solved JOINT-LOCAL pose
# straight into the related joints (no guide transforms, no -finalize step).
#   boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1
#   K        = jointRestWorld * bodyRestWorld^-1              (bodyWriteBackOffset)
#   M_parent = parentJointRestWorld * parentBodyRestWorld^-1  (bodyParentJointOffset)
# FALLBACK (parent bone has no body — that parent is never node-driven):
#   boneLocal = K * bodyLocal * groupWorld * jointParentInverse
# ---------------------------------------------------------------------------

# Dense-array default value (identity 4x4, row-major) — every body-indexed
# matrix array the node reads with jumpToArrayElement needs an element at
# EVERY index (a sparse array would read the wrong physical slot).
_IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def _matrix_from_tr(t, r) -> om.MMatrix:
    """4x4 ROW-vector matrix from translate + XYZ euler degrees (Maya)."""
    mt = om.MTransformationMatrix()
    mt.setTranslation(om.MVector(t[0], t[1], t[2]), om.MSpace.kTransform)
    mt.setRotation(
        om.MEulerRotation(
            math.radians(r[0]),
            math.radians(r[1]),
            math.radians(r[2]),
            om.MEulerRotation.kXYZ,
        )
    )
    return mt.asMatrix()


def _joint_world_rest(jpath: str) -> om.MMatrix:
    """A joint's REST world matrix, read at build time (before any solve)."""
    return om.MMatrix(cmds.getAttr(f"{jpath}.worldMatrix[0]"))


def _pmx_body_world_rest(body: PMXRigidBody) -> om.MMatrix:
    """A PMX body's rest pose in WORLD space (MMD → Maya Z-flip + handedness).

    Mirrors the conversion in the native ``mmdRigidBody -create`` command
    (world pose, then `local = world * groupWorld⁻¹`), so the baked write-back
    offsets use exactly the same world frame as the node's bodies.
    """
    world_t = (body.shape_position.x, body.shape_position.y, -body.shape_position.z)
    world_r = (
        -math.degrees(body.shape_rotation.x),
        -math.degrees(body.shape_rotation.y),
        math.degrees(body.shape_rotation.z),
    )
    return _matrix_from_tr(world_t, world_r)


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
        if rb.physics_mode == PhysicsMode.FOLLOW_BONE or rb.related_bone_index < 0:
            continue
        anchor = _find_anchor(rb.related_bone_index)
        if anchor >= 0:
            result[rb_idx] = anchor
    return result


def _wire_dynamic_write_back(
    node: str,
    group: str,
    pmx_data: PmxModel,
    joint_names: dict[int, str],
    kinematic_order: list[int],
) -> list[str]:
    """Drive the related JOINTS from the node's solved pose (Phase 3).

    Called AFTER every body and joint exists (no -finalize step): it bakes the
    write-back inputs and connects ``outTranslate``/``outRotate`` straight into
    the joints.  PHYSICS_BONE (mode 2) is rotation-only (translate not
    connected).

    CYCLE FIX: the parent inverse is derived from the PARENT BODY's solved
    Bullet transform inside the node (no DG dependency on the parent JOINT).
    For every dynamic body whose parent BONE has a rigid body we bake
    ``bodies[i].bodyParentBodyIndex`` + ``bodyParentJointOffset[i]`` =
    M_parent.  The DG ``joint.parentInverseMatrix -> bodyParentInverseMatrix``
    connection is kept ONLY for bodies whose parent bone has no body — that
    parent is never node-driven, so it cannot feed back.

    The write-back matrix arrays are DENSE (every body index 0..n-1 set) — the
    C++ node reads them with ``jumpToArrayElement(bodyIndex)``, which for a
    SPARSE array reads the wrong physical slot for high body indices.

    Returns the list of driven joint paths (for cache exclusion).
    """
    # The node needs the physics group's world matrix to lift the solved
    # group-space pose into world space for the joint-local write-back.
    try:
        cmds.connectAttr(f"{group}.worldMatrix[0]", f"{node}.groupWorldMatrix", force=True)
    except Exception as e:
        log.warning("Could not connect groupWorldMatrix: %s", e)

    n = len(pmx_data.rigid_bodies)
    # Dense arrays: every body index gets an element (identity for bodies that
    # have no write-back, real values for the dynamic ones).
    for i in range(n):
        try:
            cmds.setAttr(f"{node}.bodyWriteBackOffset[{i}]", _IDENTITY, type="matrix")
            cmds.setAttr(f"{node}.bodyParentInverseMatrix[{i}]", _IDENTITY, type="matrix")
            cmds.setAttr(f"{node}.bodyParentJointOffset[{i}]", _IDENTITY, type="matrix")
        except Exception:
            pass

    # PMX bone index -> rigid-body index (only bodies with a related joint can
    # be referenced as a write-back parent).
    bone_of_body: dict[int, int] = {}
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        if (
            body.related_bone_index >= 0
            and body.related_bone_index in joint_names
        ):
            bone_of_body.setdefault(body.related_bone_index, rb_idx)

    # 1) Bake ALL write-back inputs first, so the first evaluation (triggered
    #    by the output connections below) sees complete data.
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        if body.physics_mode == PhysicsMode.FOLLOW_BONE:
            continue
        bone_idx = body.related_bone_index
        if bone_idx < 0 or bone_idx not in joint_names:
            continue  # no related joint -> static collider, no write-back
        jpath = joint_names[bone_idx]
        try:
            k = _joint_world_rest(jpath) * _pmx_body_world_rest(body).inverse()
            cmds.setAttr(f"{node}.bodyWriteBackOffset[{rb_idx}]", list(k), type="matrix")

            parent_rb = -1
            if (
                0 <= bone_idx < len(pmx_data.bones)
                and pmx_data.bones[bone_idx].parentIndex >= 0
            ):
                parent_rb = bone_of_body.get(pmx_data.bones[bone_idx].parentIndex, -1)
            cmds.setAttr(f"{node}.bodies[{rb_idx}].bodyParentBodyIndex", int(parent_rb))
            if parent_rb >= 0:
                parent_body = pmx_data.rigid_bodies[parent_rb]
                parent_joint = joint_names[parent_body.related_bone_index]
                m_parent = _joint_world_rest(
                    parent_joint
                ) * _pmx_body_world_rest(parent_body).inverse()
                cmds.setAttr(
                    f"{node}.bodyParentJointOffset[{rb_idx}]",
                    list(m_parent),
                    type="matrix",
                )
            else:
                # Parent bone has no rigid body: read its parent inverse from
                # the DG.  That parent is never node-driven (no dynamic
                # ancestor in the PMX chain), so this cannot feed back.
                try:
                    cmds.connectAttr(
                        f"{jpath}.parentInverseMatrix[0]",
                        f"{node}.bodyParentInverseMatrix[{rb_idx}]",
                        force=True,
                    )
                except Exception as e:
                    log.warning("Could not connect parent inverse for body %d: %s", rb_idx, e)
        except Exception as e:
            log.warning("Could not bake write-back data for body %d: %s", rb_idx, e)

    # 2) Scrub-back reset anchors (dynamic body -> nearest kinematic ancestor).
    reset_map = _compute_reset_anchor_map(pmx_data, kinematic_order)
    for rb_idx, anchor_idx in reset_map.items():
        try:
            cmds.setAttr(f"{node}.bodies[{rb_idx}].bodyResetAnchorIndex", int(anchor_idx))
        except Exception:
            pass

    # 3) Connect the solved pose -> joints LAST (this triggers the first
    #    evaluation, so every input above must already be in place).
    driven: list[str] = []
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        if body.physics_mode == PhysicsMode.FOLLOW_BONE:
            continue
        bone_idx = body.related_bone_index
        if bone_idx < 0 or bone_idx not in joint_names:
            continue
        jpath = joint_names[bone_idx]
        try:
            if body.physics_mode != PhysicsMode.PHYSICS_BONE:
                cmds.connectAttr(
                    f"{node}.outTranslate[{rb_idx}].outTranslateValue",
                    f"{jpath}.translate",
                    force=True,
                )
            cmds.connectAttr(
                f"{node}.outRotate[{rb_idx}].outRotateValue",
                f"{jpath}.rotate",
                force=True,
            )
            driven.append(jpath)
        except Exception as e:
            log.warning("Could not connect dynamic output %d (%s): %s", rb_idx, jpath, e)
    return driven


def _exclude_from_dg_cache(node: Optional[str], driven_joints) -> None:
    """Disable the DG value cache for the physics subgraph.

    The node already opts out of Cached Playback natively (``getCacheSetup``).
    This additionally sets ``caching=0`` on the solver and the physics-driven
    joints so the classic DG cache never reuses stale solver outputs either.
    """
    nodes: list[str] = [node] if node else []
    nodes.extend(driven_joints)
    for n in nodes:
        if not n or not cmds.objExists(n):
            continue
        try:
            cmds.setAttr(f"{n}.caching", 0)
        except Exception:
            pass


# TODO remove in the future
def step_physics(node: Optional[str]) -> None:
    """Force a fresh solver evaluation at the current time (headless use).

    Only needed for headless/batch use (or to manually advance the sim) — in
    interactive Maya the node is time-driven and steps on its own.

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
    re-evaluating the driven joints.  Exists for headless/batch stepping.
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
    """Build the full physics graph for a PMX model (no in-memory handle).

    Creates the ``{model}_Physics`` group and the ``mmdPhysicsNode`` solver,
    fills the ``bodies`` and ``joints`` arrays through the native
    ``mmdRigidBody`` / ``mmdRigidBodyConstraint`` commands, and then WIRES THE
    SIMULATION in one pass (no -finalize step): the solver is time-driven and
    the solved pose is written STRAIGHT into the related joints (Phase 3
    direct write-back: ``boneLocal = K · bodyLocal · B_parent⁻¹ · M_parent⁻¹``).
    FOLLOW_BONE bodies are bound to their related joint via the kinematic-anchor
    input, and the node draws the colliders itself.

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
    #    (create is the default mode): DATA + bone binding.  FOLLOW_BONE
    #    bodies get their kinematic-anchor input (joint.worldMatrix ->
    #    anchorWorldMatrix + baked offset) here; dynamic bodies are data-only
    #    until the write-back wiring below.  Bodies are appended in PMX order
    #    (the command auto-increments the index to the PMX rigid-body index
    #    the constraints below reference).
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
                # The PMX non_collision_group field IS the "collides with"
                # mask (bit i set = the body collides with group i) — MMD feeds
                # it to Bullet directly.  Use it verbatim; do NOT invert.
                mask=body.non_collision_group & 0xFFFF,
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

    # 3) DIRECT JOINT WRITE-BACK — bake the K / M_parent offsets, set the
    #    scrub-back reset anchors, and connect outTranslate/outRotate -> the
    #    related joints (Phase 3).  This comes AFTER both the bodies and the
    #    joints exist, so the first evaluation (triggered by the output
    #    connections) sees complete data — no -finalize step is needed.
    kinematic_order = [
        rb_idx
        for rb_idx, b in enumerate(pmx_data.rigid_bodies)
        if b.physics_mode == PhysicsMode.FOLLOW_BONE
    ]
    driven_joints = _wire_dynamic_write_back(
        node, group, pmx_data, joint_names, kinematic_order
    )
    _exclude_from_dg_cache(node, driven_joints)

    log.info(
        "Physics: %d FOLLOW_BONE, %d dynamic bodies, %d joints (%d driven)",
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
        len(driven_joints),
    )
    return node
