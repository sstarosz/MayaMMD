"""
rigid_body_builder.py — rigid bodies for PMX models.

The single rigid-body module (formerly split across the Phase-1 visual-guide
builder and physics_builder.py).  It owns:

* the MMD → Maya coordinate conversions shared by the rigid-body code
  (Z-flip positions, ``(-rx, -ry, +rz)`` handedness rotation),
* the **rigid-body build functions** that drive one native ``mmdPhysicsNode``
  (embedded Bullet) per model: the node's ``bodies`` / ``joints`` compound
  arrays, the kinematic-anchor connections from the joints, and the direct
  write-back of the solved pose into the related joints.  The NODE draws the
  colliders itself through its draw override — no guide transforms and no
  write-back constraints exist.

The C++ node is time-driven (``time1.outTime -> node.time``) and evaluated by
the evaluation manager on every time step; it declares itself non-cacheable so
Cached Playback always re-evaluates it (see docs/PhysicsImplementation.md).
The Bullet world advances inside the node's ``compute()`` — no solver plugin,
no scriptJob, no pairBlend, no external stateful nodes.

Run it by calling :func:`create_physics_from_pmx_data` (``build_pmx_scene``
builds physics for every model automatically).  The scene is the source of
truth: reconstruct physics state later with the ``mmd.maya.pmx_model_utils``
discovery helpers (wrapped by ``ModelContext.physics*`` getters).  Headless
stepping live here as :func:`step_physics` and :func:`write_back_physics`.

This module is part of the mmd.maya.pmx package and runs inside Autodesk Maya
(requires maya.api.OpenMaya, maya.cmds).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional

import maya.api.OpenMaya as om
import maya.cmds as cmds

from mmd.core.data_types import PhysicsMode, PmxModel, ShapeType

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------


def mmd_euler_to_maya_degrees(
    rx_rad: float, ry_rad: float, rz_rad: float
) -> tuple[float, float, float]:
    """Convert MMD-space Euler angles (radians) to Maya rotate degrees.

    PMX/MMD is left-handed (+Z toward the viewer); Maya is right-handed.  The
    exact Maya-space rigid-body rotation is ``R_maya = F·R_mmd·F`` with the
    reflection ``F = diag(1, 1, -1)``, and it reproduces exactly as::

        rotateX = -rx, rotateY = -ry, rotateZ = +rz

    (verified numerically to machine precision over random rotations and
    against real rigid-body data).

    Do NOT replace this with a quaternion round-trip: Maya's
    ``MEulerRotation(x, y, z, kXYZ).asMatrix()`` is the *transpose* of the
    standard ``Rx·Ry·Rz`` product, so ``asQuaternion()`` reconstructs a negated
    rotation and produces wrong orientations after any handedness flip.

    Returns:
        Tuple of (rotateX, rotateY, rotateZ) in degrees.
    """
    return (
        math.degrees(-rx_rad),
        math.degrees(-ry_rad),
        math.degrees(rz_rad),
    )


# ===========================================================================
# Physics binding — one native mmdPhysicsNode (embedded Bullet) per model
# ===========================================================================

_NODE_TYPE = "mmdPhysicsNode"

# PMX shape -> node bodyColliderType (sphere=2, box=1, capsule=3)
_PMX_TO_COLLIDER_TYPE: dict[ShapeType, int] = {
    ShapeType.SPHERE: 2,
    ShapeType.BOX: 1,
    ShapeType.CAPSULE: 3,
}

# Scalar (non-3double) body children set through the OpenMaya plug API for
# speed (see _set_body_attributes).  The 3double children are set separately.
_BODY_ATTR_NAMES = (
    "bodyMass",
    "bodyLinearDamping",
    "bodyAngularDamping",
    "bodyFriction",
    "bodyRestitution",
    "bodyColliderType",
    "bodyRadius",
    "bodyLength",
    "bodyGroupId",
    "bodyNonCollisionGroup",
    "bodyKinematic",
    "bodyPhysicsMode",
    "bodyResetAnchorIndex",
)

# Gravity — MMD's physics engine uses exactly -9.8 (Bullet's default) in the
# model's own unit scale.  We must match that: using -98 (a 10x guess for the
# "18-unit" Tololo model) made EVERY force 10x too strong — the huge PMX hair
# masses (3276.8 at the root) × 10x gravity overloaded the rigid-weld
# constraints, so hair/skirt chains sagged visibly (a rigid bang extended ~1.1
# units) and collision pushes were 10x too violent.  -9.8 matches MMD exactly.
_DEFAULT_GRAVITY_Y = -9.8

# Playback frames per second — the node converts Maya frame deltas to seconds
# with this.
_DEFAULT_FPS = 30.0

# Maya time-unit names -> playback frames per second (cmds.currentUnit's named
# units).  Custom "NNNfps" units are parsed numerically instead.
_MAYA_TIME_UNIT_FPS: dict[str, float] = {
    "film": 24.0,
    "game": 30.0,
    "ntsc": 30.0,
    "pal": 25.0,
    "show": 48.0,
    "palf": 50.0,
    "ntscf": 60.0,
}


def _time_unit_to_fps(unit: Optional[str]) -> Optional[float]:
    """Convert a Maya time-unit string to frames-per-second (PURE).

    ``unit`` is what ``cmds.currentUnit(query=True, time=True)`` returns — a
    named unit (``"film"``, ``"game"``, ``"ntsc"``, ``"pal"``, ``"show"``,
    ``"palf"``, ``"ntscf"``) or an explicit custom rate like ``"30fps"``.
    Returns ``None`` when the string cannot be resolved (the caller falls back
    to :data:`_DEFAULT_FPS`).  Pure function of the string — unit-testable
    without Maya.
    """
    if not unit:
        return None
    named = _MAYA_TIME_UNIT_FPS.get(unit)
    if named is not None:
        return named
    if unit.endswith("fps"):
        try:
            return float(unit[: -len("fps")])
        except ValueError:
            return None
    return None


def _scene_fps(default: float = _DEFAULT_FPS) -> float:
    """Resolve the current Maya scene playback frame rate (fps).

    Used to configure the physics node's ``fps`` so its frame→seconds
    conversion matches the timeline.  Falls back to ``default`` (30, MMD's
    rate) when the time unit cannot be resolved.
    """
    try:
        unit = cmds.currentUnit(query=True, time=True)
        fps = _time_unit_to_fps(unit)
        return fps if fps is not None else default
    except Exception as e:
        log.debug("Could not resolve scene time unit: %s", e)
        return default


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


# Collision-mask resolution lives in the NODE (Phase 2): Python feeds the raw
# PMX data (bodyGroupId + bodyNonCollisionGroup) and the C++ node derives the
# effective group bit + mask with the same proximity + cloth-on-cloth
# corrections — mmd/maya/nodes/mmd_physics_masks.h is the exact port of the
# former _compute_collision_masks (see that header for the MMD-intent reasoning).


def _joint_names_for(joints) -> dict[int, str]:
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


def _create_physics_group(name_registry, root_transform_obj) -> str:
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


def _create_physics_solver(name_registry, parent_group: Optional[str] = None) -> str:
    """Create the ``mmdPhysicsNode`` (a locator shape) and make it time-driven.

    The node is an ``MPxLocatorNode``: it owns the Bullet world AND draws its
    own guide visualization (wireframe box/sphere/capsule per body, colored by
    collision group) through a C++ draw override — no scene guide meshes.  It
    is parented under the physics group at the origin, so the Bullet world runs
    in the group's local space and the guides are drawn there.  Connecting
    ``time1.outTime`` to its ``time`` input makes the evaluation manager step
    it on every frame.
    """
    kwargs: dict = {"name": name_registry.get_physics_solver_name()}
    if parent_group:
        kwargs["parent"] = parent_group
    node = cmds.createNode(_NODE_TYPE, **kwargs)
    try:
        cmds.connectAttr("time1.outTime", f"{node}.time")
    except Exception as e:
        log.warning("Could not connect time1 to node time: %s", e)
    cmds.setAttr(f"{node}.gravity", 0.0, _DEFAULT_GRAVITY_Y, 0.0)
    # Match the node's frame→seconds conversion to the scene's playback rate —
    # a scene at 24/25/48/50/60 fps would otherwise run the sim too fast/slow
    # against the timeline (the node's default is MMD's 30 fps).
    cmds.setAttr(f"{node}.fps", _scene_fps())
    return node


def _exclude_from_dg_cache(node: Optional[str], driven_joints) -> None:
    """Disable the DG value cache for the physics subgraph.

    The node already opts out of Cached Playback natively (``getCacheSetup``).
    This additionally sets ``caching=0`` on the solver and the physics-driven
    joints so the classic DG cache never reuses stale solver outputs either.
    """
    nodes: list[str] = [node] if node else []
    nodes.extend(driven_joints.values())
    for n in nodes:
        if not n or not cmds.objExists(n):
            continue
        try:
            cmds.setAttr(f"{n}.caching", 0)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Phase 3: no guide transforms exist.  The mmdPhysicsNode DRAWS the colliders
# and writes the solved pose DIRECTLY into the related joints.  PRIMARY path
# (the write-back parent inverse comes from the PARENT BODY's solved Bullet
# transform — no DG dependency on node-driven parent joints):
#   boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1
#   K        = jointRestWorld * bodyRestWorld^-1              (bodyWriteBackOffset)
#   M_parent = parentJointRestWorld * parentBodyRestWorld^-1  (bodyParentJointOffset)
# FALLBACK (parent bone has no body — that parent is never node-driven):
#   boneLocal = K * bodyLocal * groupWorld * jointParentInverse
# K is the exact world-frame offset parentConstraint(maintainOffset) used to
# maintain (targetWorld = K * sourceWorld — verified empirically), so rest
# poses stay EXACT and the whole model can be moved freely.
# ---------------------------------------------------------------------------


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


# Dense-array default value (identity 4x4, row-major) — every body-indexed
# array the node reads with jumpToArrayElement needs an element at EVERY
# index (a sparse array would read the wrong physical slot for high indices).
_IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def _body_world_rest(spec, group_world_rest) -> om.MMatrix:
    """A body spec's rest pose lifted into world space (restT/restR * groupWorld)."""
    return _matrix_from_tr(spec.restT, spec.restR) * group_world_rest


def _joint_world_rest(jpath: str) -> om.MMatrix:
    """A joint's REST world matrix, read at build time (before any solve)."""
    return om.MMatrix(cmds.getAttr(f"{jpath}.worldMatrix[0]"))


def _local_rest_in_group(world_t, world_r, group_world_rest):
    """Group-local rest translate+rotate for a PMX body.

    The guide transform used to be created at the PMX world pose and then
    parented under the physics group; its local translate/rotate became the
    Bullet rest pose.  Phase 3 computes the same local T/R directly from the
    world pose and the group's world matrix (row-vector: local = world *
    parentInverse), so no transform node is needed.
    """
    body_world = _matrix_from_tr(world_t, world_r)
    local = body_world * group_world_rest.inverse()
    mt = om.MTransformationMatrix(local)
    t = mt.translation(om.MSpace.kTransform)
    e = mt.rotation()
    return (t.x, t.y, t.z), (math.degrees(e.x), math.degrees(e.y), math.degrees(e.z))


@dataclass(frozen=True)
class _BodySpec:
    """A PMX rigid body baked for the node's ``bodies`` array (Phase 3).

    Produced by :func:`_make_body_spec`; consumed by the body-array writer,
    the anchor/write-back connectors, and the build driver.  ``joint`` is the
    related joint's full path (or ``None`` — a static collider with no
    write-back target).
    """

    restT: tuple[float, float, float]
    restR: tuple[float, float, float]
    mass: float
    linearDamping: float
    angularDamping: float
    friction: float
    restitution: float
    collider: int
    radius: float
    extents: tuple[float, float, float]
    length: float
    groupId: int
    nonCollisionGroup: int
    kinematic: bool
    physicsMode: int
    joint: Optional[str]
    boneIndex: int


def _make_body_spec(
    rb_idx: int, body, group_world_rest, joint_names
) -> Optional[_BodySpec]:
    """Compute a PMX rigid body's spec for the node's ``bodies`` array.

    Phase 3: no guide transform is created.  The rest pose is the PMX rest
    pose (Z-flip + handedness) in the PHYSICS GROUP's local space (the Bullet
    world frame), and the related joint path is carried in the spec for the
    direct write-back connections.
    """
    jn = body.related_bone_index
    jpath = joint_names.get(jn) if jn >= 0 else None
    mode = body.physics_mode
    kinematic = mode == PhysicsMode.FOLLOW_BONE

    try:
        world_t = (body.shape_position.x, body.shape_position.y, -body.shape_position.z)
        world_r = mmd_euler_to_maya_degrees(
            body.shape_rotation.x, body.shape_rotation.y, body.shape_rotation.z
        )
        local_t, local_r = _local_rest_in_group(world_t, world_r, group_world_rest)

        if jpath is None:
            log.warning(
                "Body %d (mode %s) has no related joint (bone %d) — left "
                "at rest under the physics group",
                rb_idx,
                mode.name,
                jn,
            )

        size = body.shape_size
        # The node computes the effective collision mask itself (Phase 2) from
        # the raw PMX data below (bodyGroupId + bodyNonCollisionGroup) — the
        # proximity + cloth-on-cloth corrections live in
        # mmd/maya/nodes/mmd_physics_masks.h.
        return _BodySpec(
            restT=local_t,
            restR=local_r,
            mass=body.mass,
            # MMD's move_attenuation / rotation_damping ARE the damping
            # coefficients (1.0 = fully damped -> the body settles; 0.0 = no
            # damping -> it swings forever).  A previous
            # `damping = 1 - attenuation` was INVERTED: high-attenuation cloth
            # (skirt ~0.96, bangs/cape/hair 1.0) got near-ZERO damping and
            # never settled — the bangs "jumped back and forth" on the
            # torso/jacket instead of resting on it.
            linearDamping=_clamp01(body.move_attenuation),
            angularDamping=_clamp01(body.rotation_damping),
            friction=_clamp01(body.friction_force),
            restitution=_clamp01(body.repulsion),
            collider=_PMX_TO_COLLIDER_TYPE.get(body.shape, 2),
            radius=size.x,
            extents=(size.x, size.y, size.z),
            length=size.y,
            groupId=body.group_id,
            # PMX non_collision_group is a 16-bit bitmask read as a SIGNED
            # int16 (so 0xFF6D comes back as -147).  Store it as unsigned so
            # the C++ node's "is raw data present" test (value != -1) and the
            # ~ncg & 0xFFFF mask math both see the true 16-bit value.
            nonCollisionGroup=body.non_collision_group & 0xFFFF,
            kinematic=kinematic,
            physicsMode=mode.value,
            # Related joint path (Phase 3 direct write-back) or None.
            joint=jpath,
            # PMX bone index of the related joint (parent-chain lookup for the
            # write-back parent inverse — Phase 3 cycle fix).
            boneIndex=body.related_bone_index,
        )
    except Exception as e:
        log.warning("Failed to create body %d: %s", rb_idx, e)
        return None

def _set_body_attributes(
    node: Optional[str], body_specs: dict[int, _BodySpec], reset_index: dict[int, int]
) -> None:
    """Write every PMX rigid body into the node's ``bodies`` array.

    Scalar children go through the OpenMaya plug API and only the 3double
    children (restT/restR/extents) use ``cmds.setAttr`` — the MEL marshalling
    of ~15k setAttr calls dominated the physics import time (14.8k calls ≈ 4.9s
    on a 300-body model).
    """
    if not node:
        return
    try:
        sel = om.MSelectionList()
        sel.add(node)
        dep = om.MFnDependencyNode(sel.getDependNode(0))
        bodies_plug = dep.findPlug("bodies", False)
        attr = {name: dep.attribute(name) for name in _BODY_ATTR_NAMES}
    except Exception as e:
        log.warning("Could not resolve physics node %s: %s", node, e)
        return
    for rb_idx, spec in body_specs.items():
        base = f"{node}.bodies[{rb_idx}]"
        try:
            el = bodies_plug.elementByLogicalIndex(rb_idx)
            el.child(attr["bodyMass"]).setDouble(float(spec.mass))
            el.child(attr["bodyLinearDamping"]).setDouble(float(spec.linearDamping))
            el.child(attr["bodyAngularDamping"]).setDouble(float(spec.angularDamping))
            el.child(attr["bodyFriction"]).setDouble(float(spec.friction))
            el.child(attr["bodyRestitution"]).setDouble(float(spec.restitution))
            el.child(attr["bodyColliderType"]).setShort(int(spec.collider))
            el.child(attr["bodyRadius"]).setDouble(float(spec.radius))
            el.child(attr["bodyLength"]).setDouble(float(spec.length))
            # Raw PMX collision input — the node derives the effective mask
            # and the Bullet group bit itself (Phase 2).
            el.child(attr["bodyGroupId"]).setShort(int(spec.groupId))
            el.child(attr["bodyNonCollisionGroup"]).setInt(
                int(spec.nonCollisionGroup)
            )
            el.child(attr["bodyKinematic"]).setBool(bool(spec.kinematic))
            el.child(attr["bodyPhysicsMode"]).setShort(int(spec.physicsMode))
            el.child(attr["bodyResetAnchorIndex"]).setInt(
                int(reset_index.get(rb_idx, -1))
            )
            # 3double children (no safe OM setter — MDataHandle construction
            # crashed Maya in a probe).
            cmds.setAttr(f"{base}.bodyRestTranslate", *spec.restT)
            cmds.setAttr(f"{base}.bodyRestRotate", *spec.restR)
            cmds.setAttr(f"{base}.bodyExtents", *spec.extents)
        except Exception as e:
            log.warning("Could not set body %d attributes: %s", rb_idx, e)


def _connect_kinematic_anchors(
    node: Optional[str],
    body_specs: dict[int, _BodySpec],
    kinematic_order: list[int],
    group: str,
    group_world_rest,
) -> None:
    """Feed the kinematic anchors from the JOINTS directly (Phase 3).

    ``anchorWorldMatrix[k]`` is the related JOINT's world matrix and
    ``anchorParentInverseMatrix[k]`` is the PHYSICS GROUP's world inverse, so
    the node computes the joint in the group's local space (the Bullet world
    frame).  ``anchorOffset[k]`` is a baked world-frame offset
    (colliderRestWorld * jointRestWorld^-1) that preserves the PMX body<->bone
    rest offset while the collider tracks the joint — the exact transform the
    old ``parentConstraint(joint, guide, maintainOffset)`` maintained (so rest
    poses stay EXACT and the model can be moved freely).  Bodies without a
    related joint get a STATIC anchor at their rest pose.
    """
    if not node:
        return
    for k, rb_idx in enumerate(kinematic_order):
        spec = body_specs[rb_idx]
        jpath = spec.joint
        try:
            if jpath is not None:
                cmds.connectAttr(
                    f"{jpath}.worldMatrix[0]",
                    f"{node}.anchorWorldMatrix[{k}]",
                    force=True,
                )
                cmds.connectAttr(
                    f"{group}.worldInverseMatrix[0]",
                    f"{node}.anchorParentInverseMatrix[{k}]",
                    force=True,
                )
                # K_kin = colliderRestWorld * jointRestWorld^-1 (worldMatrix[0]
                # is ~6x cheaper than a xform ws query and identical at rest —
                # the joints were just created).
                offset = _body_world_rest(
                    spec, group_world_rest
                ) * _joint_world_rest(jpath).inverse()
                cmds.setAttr(f"{node}.anchorOffset[{k}]", list(offset), type="matrix")
            else:
                # No related joint: a static collider pinned at its rest pose.
                cmds.setAttr(
                    f"{node}.anchorWorldMatrix[{k}]",
                    list(_body_world_rest(spec, group_world_rest)),
                    type="matrix",
                )
                cmds.setAttr(
                    f"{node}.anchorParentInverseMatrix[{k}]",
                    list(group_world_rest.inverse()),
                    type="matrix",
                )
                cmds.setAttr(f"{node}.anchorOffset[{k}]", _IDENTITY, type="matrix")
        except Exception as e:
            log.warning("Could not connect anchor %d (%s): %s", rb_idx, jpath, e)


def _compute_reset_anchor_map(
    pmx_data: PmxModel, kinematic_order: list[int]
) -> dict[int, int]:
    """Map each dynamic body to the anchor that drives its scrub-back reset.

    When time is scrubbed backwards the C++ node teleports dynamic bodies to
    their rest pose transformed by the CURRENT skeleton pose.  The skeleton
    pose is captured from the kinematic ANCHORS (FOLLOW_BONE guides —
    non-circular: the bone drives the guide, no write-back).  For each dynamic
    body we use the anchor of its NEAREST KINEMATIC ANCESTOR bone (walking the
    PMX parent chain), so hair uses the head anchor, skirt uses the pelvis
    anchor, sleeves use the shoulder/arm anchor, etc.

    Returns ``{rb_index: anchor_index}`` (anchor_index = position in
    ``kinematic_order``); dynamic bodies without a kinematic ancestor are
    omitted (no reset).
    """
    # bone index -> anchor index of the FOLLOW_BONE body bound to it.
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


def _connect_dynamic_outputs(
    node: Optional[str],
    body_specs: dict[int, _BodySpec],
    group: str,
    group_world_rest,
    pmx_data: PmxModel,
) -> None:
    """Drive the related JOINTS from the node's solved joint-local pose (Phase 3).

    The node computes ``boneLocal = K * bodyLocal * B_parent^-1 * M_parent^-1``
    and writes it to ``outTranslate[i]`` / ``outRotate[i]``, so those connect
    STRAIGHT into the joint's translate / rotate — no guide transforms, no
    write-back constraints.  PHYSICS_BONE (mode 2) is rotation-only: only
    outRotate is connected.

    CYCLE FIX: the parent inverse is derived from the PARENT BODY's solved
    Bullet transform inside the node (no DG dependency on the parent JOINT).
    Connecting ``joint.parentInverseMatrix -> bodyParentInverseMatrix`` for a
    body whose parent JOINT is also node-driven created a DG feedback cycle
    (parent.worldMatrix <- node.outRotate <- ... <- parent.parentInverseMatrix)
    that made the simulation explode — 86% of dynamic bodies in Tololo have a
    node-driven parent.  Instead, for every dynamic body whose parent BONE has
    a rigid body, Python bakes ``bodyParentBodyIndex[i]`` (that body's index)
    and ``bodyParentJointOffset[i]`` = M_parent = parentJointRestWorld *
    parentBodyRestWorld^-1 (the same world-frame constant for kinematic and
    dynamic parents; parentJointWorld = M_parent * B_parent * groupWorld).  The
    DG parentInverseMatrix connection is kept ONLY for bodies whose parent bone
    has no body — that parent is never node-driven, so it cannot feed back.

    ``bodyWriteBackOffset`` / ``bodyParentInverseMatrix`` / ``bodyParentJointOffset``
    are DENSE body-indexed arrays (every element 0..n-1 is set) — the C++ node
    reads them with ``MArrayDataHandle::jumpToArrayElement(bodyIndex)``, which
    for a SPARSE array treats the index as a PHYSICAL position (not the body
    index) and silently reads the wrong / missing element for high body
    indices.
    """
    if not node:
        return
    # The node needs the physics group's world matrix to lift the solved
    # group-space pose into world space before the joint-local write-back.
    try:
        cmds.connectAttr(f"{group}.worldMatrix[0]", f"{node}.groupWorldMatrix", force=True)
    except Exception as e:
        log.warning("Could not connect groupWorldMatrix: %s", e)
    # Dense arrays: every body index gets an element (identity for bodies that
    # have no write-back, real values for the dynamic ones).
    n = max(body_specs) + 1 if body_specs else 0
    for i in range(n):
        try:
            cmds.setAttr(f"{node}.bodyWriteBackOffset[{i}]", _IDENTITY, type="matrix")
            cmds.setAttr(f"{node}.bodyParentInverseMatrix[{i}]", _IDENTITY, type="matrix")
            cmds.setAttr(f"{node}.bodyParentJointOffset[{i}]", _IDENTITY, type="matrix")
        except Exception:
            pass
    # PMX bone index -> rigid-body index (only bodies that made it into the
    # node and have a related joint can be referenced as a write-back parent).
    bone_of_body = {
        spec.boneIndex: rb_idx
        for rb_idx, spec in body_specs.items()
        if spec.boneIndex is not None and spec.joint
    }
    for rb_idx, spec in body_specs.items():
        if spec.kinematic:
            continue
        jpath = spec.joint
        if jpath is None:
            continue
        try:
            # Related joint's rest world + baked world offset
            # K = jointRestWorld * bodyRestWorld^-1 (worldMatrix[0] is ~6x
            # cheaper than a xform ws query; the joints are at rest now).
            k = _joint_world_rest(jpath) * _body_world_rest(
                spec, group_world_rest
            ).inverse()
            cmds.setAttr(f"{node}.bodyWriteBackOffset[{rb_idx}]", list(k), type="matrix")

            # Parent joint's body (Phase 3 cycle fix): the write-back parent
            # inverse comes from that body's solved Bullet transform, so no DG
            # dependency on a node-driven parent joint.
            bone = spec.boneIndex
            parent_rb = -1
            if (
                bone is not None
                and 0 <= bone < len(pmx_data.bones)
                and pmx_data.bones[bone].parentIndex >= 0
            ):
                parent_rb = bone_of_body.get(pmx_data.bones[bone].parentIndex, -1)
            # bodyParentBodyIndex is a CHILD of the bodies compound array, so
            # its path goes through `bodies[i]` (not a top-level array).
            cmds.setAttr(
                f"{node}.bodies[{rb_idx}].bodyParentBodyIndex", int(parent_rb)
            )
            if parent_rb >= 0:
                parent_spec = body_specs[parent_rb]
                parent_joint = parent_spec.joint
                if parent_joint:
                    # M_parent = parentJointRestWorld * parentBodyRestWorld^-1
                    # (same constant for kinematic and dynamic parents).
                    m_parent = _joint_world_rest(
                        parent_joint
                    ) * _body_world_rest(parent_spec, group_world_rest).inverse()
                    cmds.setAttr(
                        f"{node}.bodyParentJointOffset[{rb_idx}]",
                        list(m_parent),
                        type="matrix",
                    )
            if parent_rb < 0:
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
                    log.warning(
                        "Could not connect parent inverse for body %d: %s",
                        rb_idx,
                        e,
                    )
            # Solved pose -> joint (mode 2 = rotation only).
            if spec.physicsMode != 2:
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
        except Exception as e:
            log.warning("Could not connect dynamic output %d (%s): %s", rb_idx, jpath, e)


def _set_joint_attributes(node: Optional[str], pmx_data: PmxModel) -> None:
    """Write every PMX joint into the node's ``joints`` array.

    Uses ``cmds.setAttr`` for EVERY child.  The OpenMaya plug shortcut
    (``joints_plug.elementByLogicalIndex(i).child(attr).setInt(...)``) was
    tried as part of the import speedup, but it produces a `joints` array the
    C++ node cannot enumerate (``numElements()`` returns 0 from the plugin —
    verified on Tololo: the scene showed 408 correctly-set elements while the
    node read 0, so the sim ran with NO constraints and every body collided
    with everything).  The scalar body children keep the OM path (the `bodies`
    array reads correctly); joints stay on cmds.setAttr.
    """
    if not node:
        return
    for jt_idx, joint in enumerate(pmx_data.joints):
        base = f"{node}.joints[{jt_idx}]"
        try:
            cmds.setAttr(f"{base}.jointBodyA", int(joint.rigid_body_index_a))
            cmds.setAttr(f"{base}.jointBodyB", int(joint.rigid_body_index_b))
            cmds.setAttr(f"{base}.jointType", int(joint.type.value))

            # Joint frame in the physics group's local space.
            jp = joint.position
            jr = joint.rotation
            frame_t = (jp.x, jp.y, -jp.z)
            frame_r = mmd_euler_to_maya_degrees(jr.x, jr.y, jr.z)

            pmin = joint.position_min
            pmax = joint.position_max
            rmin = joint.rotation_min
            rmax = joint.rotation_max
            psc = joint.position_spring_constant
            rsc = joint.rotation_spring_constant

            cmds.setAttr(f"{base}.jointFrameTranslate", *frame_t)
            cmds.setAttr(f"{base}.jointFrameRotate", *frame_r)

            # Linear limits: PMX units.  Angular limits: PMX radians — the
            # node passes them straight to Bullet (radians).
            if pmin is not None and pmax is not None:
                cmds.setAttr(f"{base}.jointLinearMin", pmin.x, pmin.y, pmin.z)
                cmds.setAttr(f"{base}.jointLinearMax", pmax.x, pmax.y, pmax.z)
            if rmin is not None and rmax is not None:
                cmds.setAttr(f"{base}.jointAngularMin", rmin.x, rmin.y, rmin.z)
                cmds.setAttr(f"{base}.jointAngularMax", rmax.x, rmax.y, rmax.z)
            if psc is not None:
                cmds.setAttr(f"{base}.jointLinearSpring", psc.x, psc.y, psc.z)
            if rsc is not None:
                cmds.setAttr(f"{base}.jointAngularSpring", rsc.x, rsc.y, rsc.z)
        except Exception as e:
            log.warning("Could not set joint %d attributes: %s", jt_idx, e)

def step_physics(node: Optional[str]) -> None:
    """Force a fresh solver evaluation at the current time (headless use).

    Only needed for headless/batch use (or to manually advance the sim).  In
    interactive Maya the simulation is pure DG — the node's output connections
    pull it on every time step, so playback advances it.

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


def write_back_physics(node: Optional[str], driven_joints=None) -> None:
    """Propagate the solved pose to the driven joints (headless use).

    Phase 3: the node writes the joint-local pose straight into the joints, so
    "write-back" is just stepping the solver (:func:`step_physics`) and
    re-evaluating the driven joints.  Interactive playback does this via DG
    automatically; this exists for headless/batch stepping.
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
    joints,
    name_registry,
    root_transform_obj=None,
) -> Optional[str]:
    """Build the full physics graph for a PMX model (no in-memory handle).

    Creates the ``{model}_Physics`` group, the ``mmdPhysicsNode`` solver, the
    ``bodies`` / ``joints`` arrays, and the Phase 3 DIRECT joint wiring — the
    kinematic anchors read the joints' world matrices (with a baked body<->
    bone rest offset), and the solved pose is written straight into the related
    joints.  NO guide transforms and NO write-back constraints are created:
    the node draws the colliders and owns the write-back.  DG caching is
    disabled on the subgraph.

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

    # Phase 3: no guide transforms exist.  The K offsets (world-frame body<->
    # bone rest offsets) are baked from the group's REST world matrix; they are
    # invariant under whole-model movement (verified against
    # parentConstraint(maintainOffset): targetWorld = K * sourceWorld), so the
    # model can be moved freely after import.
    group_world_rest = om.MMatrix(cmds.xform(group, q=True, ws=True, matrix=True))

    body_specs: dict[int, _BodySpec] = {}
    kinematic_order: list[int] = []

    # The collision-mask resolution lives in the NODE (Phase 2): each body
    # feeds the raw PMX data (bodyGroupId + bodyNonCollisionGroup) and the
    # node derives the effective group bit + mask itself (see
    # mmd/maya/nodes/mmd_physics_masks.h — exact port of the former Python
    # proximity + cloth-on-cloth corrections).
    for rb_idx, body in enumerate(pmx_data.rigid_bodies):
        spec = _make_body_spec(rb_idx, body, group_world_rest, joint_names)
        if spec is None:
            continue
        body_specs[rb_idx] = spec
        if body.physics_mode == PhysicsMode.FOLLOW_BONE:
            kinematic_order.append(rb_idx)

    # Populate the node's body compound array (indices = PMX rb index).
    reset_index = _compute_reset_anchor_map(pmx_data, kinematic_order)
    _set_body_attributes(node, body_specs, reset_index)

    # Connect the kinematic anchors from the JOINTS directly (Phase 3).
    _connect_kinematic_anchors(
        node, body_specs, kinematic_order, group, group_world_rest
    )

    # Populate the node's joint compound array BEFORE the write-back
    # connections below: connecting the node's outputs to the joints can
    # trigger a first evaluation of the solver, and if that happens before the
    # `joints` array is written the node bakes an EMPTY joints array into its
    # world (readJointData returns 0 -> the sim runs with NO constraints; seen
    # on Tololo: the scene stored 408 correctly-set joints but the node read
    # 0).  Writing the joints first guarantees the first compute sees them.
    _set_joint_attributes(node, pmx_data)

    # Direct joint write-back: groupWorldMatrix + bodyWriteBackOffset +
    # bodyParentBodyIndex/bodyParentJointOffset + outTranslate/outRotate ->
    # joints (Phase 3).  The parent inverse is derived from the parent BODY's
    # solved transform inside the node (no DG dependency on node-driven
    # parent joints — that was the write-back feedback cycle that exploded
    # the simulation during animation).
    _connect_dynamic_outputs(node, body_specs, group, group_world_rest, pmx_data)

    # Belt-and-suspenders on top of the node's native cache opt-out: never
    # cache the DG results of the physics subgraph.
    driven_joints = {
        rb_idx: spec.joint
        for rb_idx, spec in body_specs.items()
        if not spec.kinematic and spec.joint
    }
    _exclude_from_dg_cache(node, driven_joints)

    log.info(
        "Physics: %d FOLLOW_BONE, %d dynamic bodies, %d joints",
        sum(1 for b in pmx_data.rigid_bodies
            if b.physics_mode == PhysicsMode.FOLLOW_BONE),
        sum(1 for b in pmx_data.rigid_bodies
            if b.physics_mode != PhysicsMode.FOLLOW_BONE),
        len(pmx_data.joints),
    )
    return node
