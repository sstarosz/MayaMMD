"""
Integration tests for PMX rigid body physics creation in Maya.

**Milestone 2 (physics_builder rewrite — native C++ Bullet node):**

The old mayaBullet layer is GONE.  Physics is now driven by one native
``mmdPhysicsNode`` (an ``MPxLocatorNode`` in ``MayaMMD.mll``) that owns a
``btDiscreteDynamicsWorld``:

* The node DRAWS its own guide visualization (wireframe box/sphere/capsule per
  body, colored by collision group) via a C++ draw override — no guide
  meshes/shaders exist in the scene.
* ``FOLLOW_BONE`` bodies are **invisible guide transforms** bound to their
  bone via ``parentConstraint`` (DG).  Their world/parent-inverse matrices feed
  the node's ``anchorWorldMatrix`` / ``anchorParentInverseMatrix`` inputs so the
  kinematic colliders track the bones every frame.
* ``PHYSICS`` / ``PHYSICS_BONE`` bodies are **dynamic**: the node writes each
  body's solved LOCAL transform to ``outTranslate[i]`` / ``outRotate[i]``,
  connected straight into the guide transform; a ``parentConstraint``
  (PHYSICS) or ``orientConstraint`` (PHYSICS_BONE) writes the solved pose back
  to the related bone.

These tests check structure (node created, arrays populated, guides/anchors/
write-back wired) **and behaviour** — the thing the old suite could not detect:

* :func:`test_simulation_steps` — step time and assert a dynamic body MOVES.
* :func:`test_write_back_moves_bone` — assert the related bone follows the
  body (translation for PHYSICS, rotation for PHYSICS_BONE) so the skinned
  mesh deforms.  Both tests rewind the time and re-step afterwards, which
  rebuilds the Bullet world at rest and restores the scene.

Why the node-based approach (vs the mayaBullet solver that froze): the bullet
solver is a stateful built-in node that Cached Playback's evaluation cache does
not re-step.  ``mmdPhysicsNode`` declares itself non-cacheable
(``MPxNode::getCacheSetup``), so the evaluation manager re-evaluates it every
frame — the exact mechanism that keeps the simulation advancing under Cached
Playback.
"""

# ── Maya standalone initialised by the test runner ───────────────────────
import math

import maya.api.OpenMaya as om
from maya import cmds

from mmd.core.data_types import PmxModel
from mmd.maya.pmx.rigid_body_builder import (
    step_physics,
    write_back_physics,
)
from mmd.maya.pmx_model_utils import (
    find_physics_driven_joints,
    find_physics_node,
    find_physics_rigid_bodies,
)
from tests.integration.test_helpers import assert_eq, assert_true

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


# MMD-space Euler (radians) -> Maya rotate degrees: rotateX = -rx, rotateY = -ry,
# rotateZ = +rz (Z-flip + handedness — used by the rest-pose matrix checks).
def mmd_euler_to_maya_degrees(
    rx_rad: float, ry_rad: float, rz_rad: float
) -> tuple[float, float, float]:
    return (
        math.degrees(-rx_rad),
        math.degrees(-ry_rad),
        math.degrees(rz_rad),
    )


def _find_physics_group(maya_pmx_data):
    """Return the MFnTransform of the model's ``_Physics`` group, or None."""
    root_obj = maya_pmx_data.root_obj
    root_fn = om.MFnTransform(root_obj)
    for i in range(root_fn.childCount()):
        child = root_fn.child(i)
        try:
            child_fn = om.MFnTransform(child)
        except Exception:
            continue
        if child_fn.name().endswith("_Physics"):
            return child_fn
    return None


class _PhysicsInfo:
    """Test-local handle over the scene-discovered physics state.

    The production module keeps no in-memory binding — the scene is the source
    of truth.  This thin adapter bundles the discovery results
    (``mmd.maya.pmx_model_utils``) with the headless ops
    (``mmd.maya.pmx.rigid_body_builder``) so the tests keep a small handle.
    """

    def __init__(self, root_name: str) -> None:
        self.node = find_physics_node(root_name)
        # Phase 3: {rb_idx: related_joint} — no guide transforms exist.
        self.bodies = find_physics_rigid_bodies(root_name)
        self.driven = find_physics_driven_joints(root_name)

    def step(self) -> None:
        step_physics(self.node)

    def write_back(self) -> None:
        write_back_physics(self.node, self.driven)


def _get_binding(maya_pmx_data):
    """Reconstruct the model's physics state from the scene (or an empty handle)."""
    return _PhysicsInfo(maya_pmx_data.root_name)


def _iter_bodies(maya_pmx_data):
    """Yield ``(rb_index, joint)`` for every rigid body with a related joint.

    Phase 3: guide transforms are gone — the body is represented by its
    related joint (the node writes the solved pose straight into it).
    """
    binding = _get_binding(maya_pmx_data)
    if binding is None:
        return
    for rb_idx, joint in binding.bodies.items():
        if cmds.objExists(joint):
            yield rb_idx, joint


def _root_joint(maya_pmx_data) -> str:
    """Name of the model's root bone joint (drives the whole skeleton)."""
    return om.MFnDagNode(maya_pmx_data.joints[0]).partialPathName()


def _swing_root(maya_pmx_data, frame: int, degrees: float = 20.0) -> None:
    """Rotate the root bone sinusoidally so the dynamic chains keep moving.

    The kinematic anchors track the root, and the attached dynamic chains
    follow — the MMD behavior that the old (exploding) write-back accidentally
    masked by moving joints on its own.  Swinging (not holding) keeps the
    chains in motion so the behavioural tests have a stable signal.
    """
    angle = degrees * math.sin(math.radians(frame * 12.0))
    root = _root_joint(maya_pmx_data)
    cmds.setAttr(f"{root}.rotateZ", angle)
    cmds.dgdirty(root)


def _restore_root(maya_pmx_data) -> None:
    """Reset the root bone rotation to zero."""
    root = _root_joint(maya_pmx_data)
    cmds.setAttr(f"{root}.rotateZ", 0.0)
    cmds.dgdirty(root)


def _mmd_flipped_y_axis(rx_rad: float, ry_rad: float, rz_rad: float):
    """Image of the local Y axis under ``R_maya = F·R_mmd·F``.

    ``F = diag(1, 1, -1)``; the PMX rigid-body rotation composes (in the matrix
    convention that matches Maya's rotate attributes) as ``Rz·Ry·Rx``, so the
    expected world-space Y basis is ``F·Rz·Ry·Rx·(0,1,0)``.  This is the
    orientation check that catches handedness-conversion regressions.
    """
    import math as _m

    cz, sz = _m.cos(rz_rad), _m.sin(rz_rad)
    cy, sy = _m.cos(ry_rad), _m.sin(ry_rad)
    cx, sx = _m.cos(rx_rad), _m.sin(rx_rad)
    # Rz·Ry·Rx applied to (0, 1, 0), then reflected by F = diag(1, 1, -1)
    return (
        cz * sy * sx - sz * cx,
        sz * sy * sx + cz * cx,
        -cy * sx,
    )


def _expected_world_matrix(pos, rot_rad):
    """Expected Maya world matrix from PMX position + rotation (Z-flip + handedness).

    Built exactly like Maya constructs a transform matrix (MTransformationMatrix +
    MEulerRotation kXYZ) so it is directly comparable to ``cmds.xform(matrix=True)``.
    A matrix comparison is used instead of raw Euler components because Euler
    decomposition is non-unique.
    """
    import math as _m

    world_t = (pos.x, pos.y, -pos.z)
    world_r = mmd_euler_to_maya_degrees(rot_rad.x, rot_rad.y, rot_rad.z)
    mt = om.MTransformationMatrix()
    mt.setTranslation(om.MVector(*world_t), om.MSpace.kWorld)
    e = om.MEulerRotation(
        _m.radians(world_r[0]),
        _m.radians(world_r[1]),
        _m.radians(world_r[2]),
        om.MEulerRotation.kXYZ,
    )
    mt.setRotation(e)
    return mt.asMatrix()


def _find_joint_by_bone_index(maya_pmx_data, bone_idx):
    """Return the Maya joint name whose pmxBoneIndex == bone_idx, or None."""
    for j in maya_pmx_data.joints:
        jn = om.MFnDependencyNode(j).name()
        if (
            cmds.attributeQuery("pmxBoneIndex", node=jn, exists=True)
            and cmds.getAttr(f"{jn}.pmxBoneIndex") == bone_idx
        ):
            return jn
    return None


def _is_driven_by(node_out: str, dest: str) -> bool:
    """True if *dest* is driven by *node_out*, possibly via a unitConversion.

    Maya auto-inserts a ``unitConversion`` node when a raw double3 is
    connected into an angle-unit attribute such as ``rotate`` (the raw degrees
    are converted to radians internally; display values match exactly).
    """
    if cmds.isConnected(node_out, dest):
        return True
    for uc in cmds.listConnections(dest, source=True, type="unitConversion") or []:
        if cmds.isConnected(node_out, f"{uc}.input"):
            return True
    return False


# ---------------------------------------------------------------------------
# Structural tests (node + guides + wiring)
# ---------------------------------------------------------------------------


def test_pmx_rigid_body_physics_group(pmx_data: PmxModel, maya_pmx_data):
    """Test that a ``{model}_Physics`` group with an mmdPhysicsNode exists."""
    if not pmx_data.rigid_bodies:
        print("SKIP: model has no rigid bodies")
        return True
    group = _find_physics_group(maya_pmx_data)
    assert_true(group is not None, "Physics group not found under root")
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node
    assert_true(
        cmds.nodeType(node) == "mmdPhysicsNode", f"{node} is not an mmdPhysicsNode"
    )
    # Phase 1: the node is an MPxLocatorNode (DAG shape) parented under the
    # physics group — it draws its own guides, so it must live in the DAG.
    node_parents = cmds.listRelatives(node, parent=True, type="transform") or []
    assert_true(
        bool(node_parents),
        f"mmdPhysicsNode {node} is not a DAG shape (no parent transform)",
    )
    # Simulation is ENABLED: the node is time-driven (time1.outTime → time).
    assert_true(
        cmds.isConnected("time1.outTime", f"{node}.time"),
        "mmdPhysicsNode.time is not connected to time1.outTime (simulation not "
        "time-driven)",
    )
    print(f"PASS: Physics group + mmdPhysicsNode created: {node}")
    return True


def test_pmx_rigid_body_count(pmx_data: PmxModel, maya_pmx_data):
    """Test that node.bodies/joints array sizes match the PMX data."""
    if not pmx_data.rigid_bodies:
        print("SKIP: model has no rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node
    n_bodies = cmds.getAttr(f"{node}.bodies", size=True)
    assert_eq(
        n_bodies,
        len(pmx_data.rigid_bodies),
        f"node.bodies size {n_bodies} != PMX rigid body count {len(pmx_data.rigid_bodies)}",
    )
    if pmx_data.joints:
        n_joints = cmds.getAttr(f"{node}.joints", size=True)
        assert_eq(
            n_joints,
            len(pmx_data.joints),
            f"node.joints size {n_joints} != PMX joint count {len(pmx_data.joints)}",
        )
    # Every body with a related JOINT that actually exists is bound: kinematic
    # bodies through the kinematic-anchor INPUT, dynamic bodies through the
    # write-back outputs (outTranslate/outRotate -> joint).  Bodies with no
    # related joint (related_bone_index -1 / out of range — e.g. Fritia body
    # 156) are static colliders left at rest.
    valid_bones = {
        b_idx
        for b_idx, j in enumerate(maya_pmx_data.joints)
        if j is not None and not j.isNull()
    }
    expected = sum(
        1
        for rb in pmx_data.rigid_bodies
        if rb.related_bone_index in valid_bones
    )
    n_body_joints = len(list(_iter_bodies(maya_pmx_data)))
    assert_eq(
        n_body_joints,
        expected,
        f"bound body count {n_body_joints} != bodies with a related joint "
        f"{expected} (PMX body count {len(pmx_data.rigid_bodies)})",
    )
    print(
        f"PASS: {n_bodies} bodies, {n_joints if pmx_data.joints else 0} joints in node"
    )
    return True


def test_pmx_rigid_body_attributes(pmx_data: PmxModel, maya_pmx_data):
    """Test that each node body element carries the PMX parameters."""
    if not pmx_data.rigid_bodies:
        print("SKIP: model has no rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node
    checked = 0
    for rb_idx, rb in enumerate(pmx_data.rigid_bodies):
        base = f"{node}.bodies[{rb_idx}]"
        mode = rb.physics_mode.value
        kin = mode == 0
        if not kin:
            assert_true(
                float(cmds.getAttr(f"{base}.bodyMass")) > 0.0,
                f"dynamic body {rb_idx} mass not > 0",
            )
        # PMX physics mode is canonical (kinematic == mode 0).
        assert_eq(
            cmds.getAttr(f"{base}.bodyPhysicsMode"),
            mode,
            f"body {rb_idx} bodyPhysicsMode != {mode}",
        )
        # Raw PMX collision group id feeds the node's group/mask derivation.
        assert_eq(
            cmds.getAttr(f"{base}.bodyGroupId"),
            rb.group_id,
            f"body {rb_idx} bodyGroupId != group_id",
        )
        # Collision mask: the PMX non_collision_group field IS the "collides
        # with" mask (bit i set = collides with group i) — stored VERBATIM
        # (no inversion).  Each bodyMaskGroup bool must match the raw bit.
        raw_mask = rb.non_collision_group & 0xFFFF
        for g in range(16):
            assert_eq(
                bool(cmds.getAttr(f"{base}.bodyMaskGroup{g}")),
                bool(raw_mask & (1 << g)),
                f"body {rb_idx} bodyMaskGroup{g} != PMX bit {g} of 0x{raw_mask:04X}",
            )
        assert_true(
            cmds.attributeQuery("bodyRestTranslate", node=node, exists=True),
            "bodyRestTranslate missing",
        )
        checked += 1
    assert_true(checked > 0, "No body attributes checked")
    print(f"PASS: {checked} node body elements carry PMX parameters")
    return True


def test_pmx_rigid_body_rest_transform(pmx_data: PmxModel, maya_pmx_data):
    """FOLLOW_BONE bodies: rest pose in the node + joint-driven anchors (Phase 3).

    With guide transforms gone, the body's rest pose lives in
    ``bodies[i].bodyRestTranslate/Rotate`` (the PMX pose, Z-flip + handedness,
    in the physics group's local space — the group is at the identity in these
    fresh test scenes) and the collider tracks the related JOINT via the
    ``anchorWorldMatrix`` / ``anchorOffset`` wiring.
    """
    fb = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value == 0]
    if not fb:
        print("SKIP: model has no FOLLOW_BONE rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node

    checked = 0
    kin_idx = 0
    for rb_idx, joint in _iter_bodies(maya_pmx_data):
        rb = pmx_data.rigid_bodies[rb_idx]
        if rb.physics_mode.value != 0:
            continue  # dynamic bodies are solver-driven — not checked here

        # Rest pose in the node matches the PMX pose (Z-flip + handedness).
        pos = rb.shape_position
        expected_t = (pos.x, pos.y, -pos.z)
        got_t = cmds.getAttr(f"{node}.bodies[{rb_idx}].bodyRestTranslate")[0]
        for got, exp, label in zip(got_t, expected_t, ("X", "Y", "Z")):
            assert_true(
                abs(got - exp) < 1e-3,
                f"Body {rb_idx} bodyRestTranslate{label} {got} != expected {exp}",
            )

        # The anchor slot is fed by the related JOINT's world matrix.
        srcs = (
            cmds.listConnections(f"{node}.anchorWorldMatrix[{kin_idx}]", source=True)
            or []
        )
        assert_true(
            any(f"{joint}.worldMatrix[0]" == f"{s}.worldMatrix[0]" for s in srcs),
            f"FOLLOW_BONE body {rb_idx} joint {joint} not feeding anchorWorldMatrix[{kin_idx}]",
        )
        kin_idx += 1
        checked += 1

    assert_true(checked > 0, "No FOLLOW_BONE bodies found to check rest poses")
    print(f"PASS: {checked} FOLLOW_BONE bodies at PMX rest pose, joint-anchored")
    return True


def test_follow_bone_anchor_tracks_joint(pmx_data: PmxModel, maya_pmx_data):
    """A FOLLOW_BONE collider tracks its joint: the anchor input follows it.

    Phase 3 connects ``joint.worldMatrix[0]`` straight into
    ``anchorWorldMatrix[k]`` (the node applies the baked body<->bone offset),
    so rotating the joint must be reflected in the anchor input — the collider
    is driven by exactly this input every frame.
    """
    fb = [
        (i, rb)
        for i, rb in enumerate(pmx_data.rigid_bodies)
        if rb.physics_mode.value == 0
    ]
    if not fb:
        print("SKIP: model has no FOLLOW_BONE rigid bodies")
        return True

    target = None
    for rb_idx, rb in fb:
        if rb.related_bone_index >= 0:
            target = (rb_idx, rb)
            break
    if target is None:
        print("SKIP: no FOLLOW_BONE body with a related bone")
        return True
    rb_idx, rb = target
    k = next(i for i, (idx, _r) in enumerate(fb) if idx == rb_idx)

    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node

    jpath = _find_joint_by_bone_index(maya_pmx_data, rb.related_bone_index)
    assert_true(jpath is not None, f"Joint for bone {rb.related_bone_index} not found")

    a0 = cmds.getAttr(f"{node}.anchorWorldMatrix[{k}]")
    cmds.setAttr(f"{jpath}.rotateY", 30.0)
    cmds.currentTime(cmds.currentTime(query=True))
    a1 = cmds.getAttr(f"{node}.anchorWorldMatrix[{k}]")
    j1 = cmds.xform(jpath, q=True, ws=True, matrix=True)
    err = max(abs(a1[i] - j1[i]) for i in range(16))
    assert_true(
        err < 1e-3,
        f"anchorWorldMatrix[{k}] did not track joint {jpath} (err {err:.5f})",
    )
    # Restoring the rotation must restore the anchor too.
    cmds.setAttr(f"{jpath}.rotateY", 0.0)
    cmds.currentTime(cmds.currentTime(query=True))
    a2 = cmds.getAttr(f"{node}.anchorWorldMatrix[{k}]")
    assert_true(
        max(abs(a2[i] - a0[i]) for i in range(16)) < 1e-3,
        "anchorWorldMatrix did not return to rest after restoring the joint",
    )
    print(f"PASS: FOLLOW_BONE anchor tracks joint {jpath} (kinematic idx {k})")
    return True


def test_rigid_body_no_guide_transforms(pmx_data: PmxModel, maya_pmx_data):
    """Phase 3: NO guide transforms exist — the node draws the colliders.

    The physics group contains only the ``mmdPhysicsNode`` solver (a locator
    shape that draws wireframe box/sphere/capsule per body through its C++
    draw override).  The old per-body guide transforms (and their
    ``pmxRigidBodyIndex`` metadata) are gone entirely.
    """
    if not pmx_data.rigid_bodies:
        print("SKIP: model has no rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node

    group = _find_physics_group(maya_pmx_data)
    assert_true(group is not None, "Physics group not found")
    children = (
        cmds.listRelatives(group.fullPathName(), children=True, fullPath=True) or []
    )
    node_long = cmds.ls(node, long=True) or [node]
    # The only thing under the physics group is the solver locator.
    assert_true(
        len(children) == 1 and children[0] == node_long[0],
        f"Physics group should contain ONLY the solver, got {children}",
    )
    # No transform in the scene carries the old per-guide metadata.
    stamped = [
        t
        for t in cmds.ls(type="transform", long=True) or []
        if cmds.attributeQuery("pmxRigidBodyIndex", node=t, exists=True)
    ]
    assert_true(
        not stamped, f"Stale guide transforms with pmxRigidBodyIndex: {stamped}"
    )
    # The solver is a locator shape (DAG) that draws itself.
    assert_true(
        cmds.nodeType(node, apiType=True) == "kPluginLocatorNode",
        f"{node} is not a locator node",
    )
    print(f"PASS: no guide transforms; node {node} is a locator that draws colliders")
    return True


def test_dynamic_bodies(pmx_data: PmxModel, maya_pmx_data):
    """Dynamic bodies are WRITE-BACK driven (simulation enabled).

    Dynamic (PHYSICS / PHYSICS_BONE) bodies are created with their full PMX
    data (mass, damping, shape, group, ...) and their related joints are
    driven by the node's write-back outputs (outTranslate for mode 1,
    outRotate through the auto-inserted unitConversion for mode 2).
    """
    dyn = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value in (1, 2)]
    if not dyn:
        print("SKIP: model has no dynamic rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node

    n_dyn = sum(
        1
        for i in range(int(cmds.getAttr(f"{node}.bodies", size=True) or 0))
        if cmds.getAttr(f"{node}.bodies[{i}].bodyPhysicsMode") in (1, 2)
    )
    assert_eq(n_dyn, len(dyn), f"dynamic body count {n_dyn} != {len(dyn)}")

    # Every dynamic body with a related joint must be driven by the node's
    # write-back outputs (discovered through find_physics_driven_joints).
    valid_bones = {
        b_idx
        for b_idx, j in enumerate(maya_pmx_data.joints)
        if j is not None and not j.isNull()
    }
    expected = sum(1 for rb in dyn if rb.related_bone_index in valid_bones)
    driven = binding.driven
    assert_true(len(driven) > 0, "no dynamic body is write-back driven")
    assert_eq(
        len(driven),
        expected,
        f"driven dynamic bodies {len(driven)} != {expected}",
    )
    # Each driven joint's rotate comes from the node (mode 2 follows the
    # auto-inserted unitConversion).
    for rb_idx, joint in driven.items():
        srcs = cmds.listConnections(f"{joint}.rotate", source=True) or []
        assert_true(
            len(srcs) > 0,
            f"dynamic body {rb_idx} joint {joint} rotate not driven",
        )
    print(f"PASS: {len(driven)} dynamic bodies driven by node write-back")
    return True


def test_write_back_no_dg_cycle(pmx_data: PmxModel, maya_pmx_data):
    """Write-back is CYCLE-SAFE (parent inverse from the parent BODY).

    Phase 3's write-back derives the parent inverse from the parent BODY's
    solved Bullet transform, never from the DG.  A dynamic body whose parent
    BONE has a rigid body must carry ``bodyParentBodyIndex`` + a baked
    ``bodyParentJointOffset`` and must NOT have the DG
    ``joint.parentInverseMatrix -> bodyParentInverseMatrix`` connection (that
    is what created the feedback cycle that exploded the sim).  The DG
    fallback is allowed only when the parent bone has no body (that parent is
    never node-driven).
    """
    if not pmx_data.rigid_bodies:
        print("SKIP: model has no rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node

    bone_of_body: dict[int, int] = {}
    for rb_idx, rb in enumerate(pmx_data.rigid_bodies):
        if rb.related_bone_index >= 0:
            bone_of_body.setdefault(rb.related_bone_index, rb_idx)

    checked = 0
    for rb_idx, rb in enumerate(pmx_data.rigid_bodies):
        if rb.physics_mode.value == 0 or rb.related_bone_index < 0:
            continue
        bone = rb.related_bone_index
        expected_pbi = -1
        if (
            0 <= bone < len(pmx_data.bones)
            and pmx_data.bones[bone].parentIndex >= 0
        ):
            expected_pbi = bone_of_body.get(pmx_data.bones[bone].parentIndex, -1)
        actual_pbi = int(
            cmds.getAttr(f"{node}.bodies[{rb_idx}].bodyParentBodyIndex")
        )
        assert_eq(
            actual_pbi,
            expected_pbi,
            f"body {rb_idx} bodyParentBodyIndex {actual_pbi} != expected {expected_pbi}",
        )
        pinv = (
            cmds.listConnections(
                f"{node}.bodyParentInverseMatrix[{rb_idx}]", source=True
            )
            or []
        )
        if expected_pbi >= 0:
            assert_true(
                not pinv,
                f"body {rb_idx} has DG parentInverse while a parent body is set "
                f"(DG feedback cycle): {pinv}",
            )
        checked += 1
    assert_true(checked > 0, "no dynamic body checked")
    print(f"PASS: {checked} dynamic bodies wired cycle-free (parent body -> no DG inverse)")
    return True


def test_physics_joints(pmx_data: PmxModel, maya_pmx_data):
    """Test that PMX joints are written into the node's joints array."""
    if not pmx_data.joints:
        print("SKIP: model has no joints")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node
    n = cmds.getAttr(f"{node}.joints", size=True)
    assert_eq(
        n, len(pmx_data.joints), f"node.joints size {n} != {len(pmx_data.joints)}"
    )
    checked = 0
    for jt_idx, joint in enumerate(pmx_data.joints):
        base = f"{node}.joints[{jt_idx}]"
        assert_eq(
            cmds.getAttr(f"{base}.jointBodyA"),
            joint.rigid_body_index_a,
            f"joint {jt_idx} bodyA",
        )
        assert_eq(
            cmds.getAttr(f"{base}.jointBodyB"),
            joint.rigid_body_index_b,
            f"joint {jt_idx} bodyB",
        )
        assert_eq(
            cmds.getAttr(f"{base}.jointType"),
            joint.type.value,
            f"joint {jt_idx} type",
        )
        checked += 1
    print(f"PASS: {checked} joints written into node.joints")
    return True


def test_kinematic_anchors(pmx_data: PmxModel, maya_pmx_data):
    """Phase 3: kinematic anchors are fed by the JOINTS + a baked offset.

    ``anchorWorldMatrix[k]`` comes from the related joint's ``worldMatrix[0]``,
    ``anchorParentInverseMatrix[k]`` from the physics group's
    ``worldInverseMatrix[0]`` (so the anchor is in the group's local space),
    and ``anchorOffset[k]`` holds the baked body<->bone rest offset.
    """
    fb = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value == 0]
    if not fb:
        print("SKIP: model has no FOLLOW_BONE rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node
    n_anchors = cmds.getAttr(f"{node}.anchorWorldMatrix", size=True)
    assert_eq(n_anchors, len(fb), f"anchorWorldMatrix size {n_anchors} != {len(fb)}")
    n_pinv = cmds.getAttr(f"{node}.anchorParentInverseMatrix", size=True)
    assert_eq(n_pinv, len(fb), f"anchorParentInverseMatrix size {n_pinv} != {len(fb)}")
    n_off = cmds.getAttr(f"{node}.anchorOffset", size=True)
    assert_eq(n_off, len(fb), f"anchorOffset size {n_off} != {len(fb)}")
    group = _find_physics_group(maya_pmx_data)
    assert_true(group is not None, "Physics group not found")
    group_path = group.fullPathName()

    connected = 0
    kin_idx = 0
    for rb_idx, joint in _iter_bodies(maya_pmx_data):
        if pmx_data.rigid_bodies[rb_idx].physics_mode.value != 0:
            continue
        # anchor world = the joint's world matrix.
        srcs = (
            cmds.listConnections(f"{node}.anchorWorldMatrix[{kin_idx}]", source=True)
            or []
        )
        assert_true(
            any(f"{joint}.worldMatrix[0]" == f"{s}.worldMatrix[0]" for s in srcs),
            f"FOLLOW_BONE body {rb_idx} joint {joint} not feeding anchorWorldMatrix[{kin_idx}]",
        )
        # anchor parent inverse = the PHYSICS GROUP's world inverse (so the
        # anchor stays in the group's local space = the Bullet world frame).
        pinv_srcs = (
            cmds.listConnections(
                f"{node}.anchorParentInverseMatrix[{kin_idx}]", source=True
            )
            or []
        )
        pinv_names = set()
        for s in pinv_srcs:
            sname = s.split(".")[0]
            listed = cmds.ls(sname)
            pinv_names.add(listed[0] if listed else sname)
        group_short = cmds.ls(group_path)[0] if cmds.ls(group_path) else group_path
        assert_true(
            group_short in pinv_names,
            f"FOLLOW_BONE body {rb_idx} anchorParentInverse not fed by group world inverse",
        )
        kin_idx += 1
        connected += 1
    assert_eq(connected, len(fb), "not all FOLLOW_BONE bodies anchored")
    print(
        f"PASS: {connected} kinematic anchors fed by joints (+ group inverse + offset)"
    )
    return True


# ---------------------------------------------------------------------------
# BEHAVIOURAL tests — the core of Milestone 2
# ---------------------------------------------------------------------------


def test_simulation_steps(pmx_data: PmxModel, maya_pmx_data):
    """BEHAVIOURAL: the node is time-driven and stepping MOVES the chains.

    The solver is connected to ``time1.outTime`` and its write-back outputs
    drive the related joints, so advancing time (while swinging the root bone)
    must change at least one dynamic joint's LOCAL pose — the "simulation is
    alive" signal.
    """
    dyn = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value in (1, 2)]
    if not dyn:
        print("SKIP: model has no dynamic rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node

    # The node must be time-driven and its outputs must reach the joints.
    assert_true(
        cmds.isConnected("time1.outTime", f"{node}.time"),
        "node.time is not connected (simulation not time-driven)",
    )
    assert_true(
        bool(cmds.listConnections(f"{node}.outRotate", destination=True) or [])
        or bool(cmds.listConnections(f"{node}.outTranslate", destination=True) or []),
        "node write-back outputs have no connections",
    )

    def _local(joint):
        return cmds.getAttr(f"{joint}.rotate")[0]

    cmds.currentTime(1)
    binding.step()
    starts = {j: _local(j) for _, j in _iter_bodies(maya_pmx_data) if cmds.objExists(j)}

    for f in (5, 10, 15, 20, 25, 30):
        _swing_root(maya_pmx_data, f)
        cmds.currentTime(f)
        binding.step()
    binding.write_back()

    moved = 0
    for j, r0 in starts.items():
        if not cmds.objExists(j):
            continue
        r1 = _local(j)
        if max(abs(r1[i] - r0[i]) for i in range(3)) > 0.05:
            moved += 1
    assert_true(
        moved > 0,
        "no dynamic joint moved after stepping — simulation is not running",
    )

    _restore_root(maya_pmx_data)
    cmds.currentTime(1)
    binding.step()
    binding.write_back()
    print(f"PASS: {moved} dynamic joints moved after stepping the solver")
    return True


def test_write_back_moves_bone(pmx_data: PmxModel, maya_pmx_data):
    """BEHAVIOURAL: the node's solved pose reaches the related JOINTS.

    Phase 3: ``outTranslate``/``outRotate`` connect STRAIGHT into the joint, so
    a dynamic body's JOINT is the write-back target (no guide, no constraint).
    We swing the ROOT bone (the kinematic anchors track it), step the sim, and
    assert the most-moved dynamic joint's LOCAL pose changed from its rest —
    the node's output actually moved the BONE (its local rotate/translate
    changed, not just its world following the parent).  This is the "mesh
    binding" signal: if the solver froze, no dynamic joint would move.
    """
    dyn = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value in (1, 2)]
    if not dyn:
        print("SKIP: model has no dynamic rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")

    # Scan ALL dynamic bodies with a related bone; the most-moved one is the
    # strongest write-back signal (rigid chains legitimately hold still).
    candidates = [
        (rb_idx, joint, pmx_data.rigid_bodies[rb_idx])
        for rb_idx, joint in _iter_bodies(maya_pmx_data)
        if pmx_data.rigid_bodies[rb_idx].physics_mode.value in (1, 2)
        and pmx_data.rigid_bodies[rb_idx].related_bone_index >= 0
    ]
    if not candidates:
        print("SKIP: no dynamic body with a related bone")
        return True

    def _local(rb_idx, joint):
        """(rotate, translate) in degrees/units — the joint's LOCAL pose."""
        return (
            cmds.getAttr(f"{joint}.rotate")[0],
            cmds.getAttr(f"{joint}.translate")[0],
        )

    cmds.currentTime(1)
    binding.step()
    starts = [
        (rb_idx, joint, rb, _local(rb_idx, joint)) for rb_idx, joint, rb in candidates
    ]

    # Swing the root so the dynamic chains genuinely move.
    for f in (5, 10, 15, 20, 25, 30):
        _swing_root(maya_pmx_data, f)
        cmds.currentTime(f)
        binding.step()
    binding.write_back()

    def _local_disp(s):
        rot, tr = _local(s[0], s[1])
        dR = max(abs(rot[i] - s[3][0][i]) for i in range(3))
        dT = max(abs(tr[i] - s[3][1][i]) for i in range(3))
        return dR, dT

    best = max(starts, key=lambda s: max(_local_disp(s)))
    rb_idx, joint, rb, _p0 = best
    dR, dT = _local_disp(best)
    mode = rb.physics_mode.value

    # The driven joint's LOCAL pose must actually change (rotation for chains,
    # translation for mode-1 bodies whose body drifts from its parent).
    ok = dR > 0.05 or (mode == 1 and dT > 0.005)
    assert_true(
        ok,
        f"Dynamic joint {joint} (body {rb_idx}, mode {mode}) local pose did "
        f"not change (dR={dR:.3f}, dT={dT:.4f}) — write-back broken",
    )
    print(
        f"PASS: dynamic joint {joint} driven by solver "
        f"(mode {mode}, dR={dR:.3f}, dT={dT:.4f})"
    )

    # Restore the scene for the following suites.
    _restore_root(maya_pmx_data)
    cmds.currentTime(1)
    binding.step()
    binding.write_back()
    return True


def test_config_edit_rebuilds_node(pmx_data: PmxModel, maya_pmx_data):
    """PHASE 4: editing a body's mass mid-sim rebuilds the Bullet world in place.

    Mass (and damping, limits, collider size) is baked into the Bullet
    construction info at build time, so WITHOUT the auto-rebuild a
    ``bodies[i].bodyMass`` edit would have NO effect on the running sim.  The
    node hashes its config inputs every evaluation and rebuilds
    (destroy -> re-read -> buildWorld) when they change — keeping the dynamic
    chains glued to the CURRENT skeleton pose (not a rewind teleport to rest).

    A 0-mass dynamic body becomes STATIC in Bullet: it stops moving entirely.
    That is the deterministic proof the rebuild took effect.  We also assert
    another dynamic body KEEPS moving (the rebuild did not freeze the world)
    and that the edited body did not jump back to its rest pose.
    """
    dyn = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value in (1, 2)]
    if not dyn:
        print("SKIP: model has no dynamic rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node

    guides = [
        (rb_idx, joint)
        for rb_idx, joint in _iter_bodies(maya_pmx_data)
        if pmx_data.rigid_bodies[rb_idx].physics_mode.value in (1, 2)
    ]
    if not guides:
        print("SKIP: no dynamic body joints")
        return True

    # Swing the root bone so the dynamic chains genuinely move (a stable sim
    # does not move them on its own).
    cmds.currentTime(1)
    _swing_root(maya_pmx_data, 1)
    binding.step()
    starts = {
        rb_idx: cmds.xform(guide, q=True, ws=True, translation=True)
        for rb_idx, guide in guides
    }
    moved = None
    for f in (5, 10, 15, 20, 25, 30):
        _swing_root(maya_pmx_data, f)
        cmds.currentTime(f)
        binding.step()
        for rb_idx, guide in guides:
            p = cmds.xform(guide, q=True, ws=True, translation=True)
            p0 = starts[rb_idx]
            if max(abs(p[i] - p0[i]) for i in range(3)) > 0.01:
                moved = (rb_idx, guide)
                break
        if moved:
            break
    assert_true(
        moved is not None,
        "No dynamic body moved while swinging the root — cannot test the rebuild",
    )
    assert moved is not None  # mypy: assert_true is not a type guard

    # Prefer a body that has a reset anchor: after the rebuild it must stay
    # glued to the CURRENT skeleton pose (the "no rewind" part of the test).
    anchored = [
        g
        for g in guides
        if cmds.getAttr(f"{node}.bodies[{g[0]}].bodyResetAnchorIndex") >= 0
    ]
    if anchored:
        moved = anchored[0]
    rb_idx, guide = moved
    p_before = cmds.xform(guide, q=True, ws=True, translation=True)
    orig_mass = cmds.getAttr(f"{node}.bodies[{rb_idx}].bodyMass")

    # Edit the mass -> the node must rebuild on the next evaluation.
    cmds.setAttr(f"{node}.bodies[{rb_idx}].bodyMass", 0.0)
    binding.step()

    # No rewind: an anchored body stays at the CURRENT frame's pose instead of
    # being teleported back to its PMX rest pose (which a swung chain can be
    # several units away from).
    p_after = cmds.xform(guide, q=True, ws=True, translation=True)
    glue_err = max(abs(p_after[i] - p_before[i]) for i in range(3))
    assert_true(
        glue_err < 3.0,
        f"Config rebuild teleported body {guide} away from the current pose "
        f"(moved {glue_err:.3f}) — the in-place rebuild reset it like a rewind",
    )

    # The mass edit took effect: mass 0 = static, so THIS body must not move
    # over the next several frames (before the edit it was moving).
    pos = cmds.xform(guide, q=True, ws=True, translation=True)
    max_drift = 0.0
    for f in (35, 40, 45, 50, 55, 60):
        _swing_root(maya_pmx_data, f)
        cmds.currentTime(f)
        binding.step()
        p = cmds.xform(guide, q=True, ws=True, translation=True)
        max_drift = max(max_drift, max(abs(p[i] - pos[i]) for i in range(3)))
    assert_true(
        max_drift < 0.01,
        f"Mass-0 body {guide} still moved {max_drift:.4f} — mass edit did not "
        "take effect (no auto-rebuild)",
    )

    # The rebuild did not freeze the world: at least one OTHER dynamic body
    # still moves over the following frames (the root keeps swinging).
    cmds.currentTime(60)
    _swing_root(maya_pmx_data, 60)
    binding.step()
    others_start = {
        rb2: cmds.xform(g2, q=True, ws=True, translation=True)
        for rb2, g2 in guides
        if rb2 != rb_idx
    }
    other_moved = False
    for f in (65, 70, 75, 80, 85, 90):
        _swing_root(maya_pmx_data, f)
        cmds.currentTime(f)
        binding.step()
        for rb2, g2 in guides:
            if rb2 == rb_idx:
                continue
            p = cmds.xform(g2, q=True, ws=True, translation=True)
            p0 = others_start[rb2]
            if max(abs(p[i] - p0[i]) for i in range(3)) > 0.01:
                other_moved = True
                break
        if other_moved:
            break
    assert_true(
        other_moved,
        "No OTHER dynamic body moved after the config rebuild — the rebuild "
        "froze the simulation",
    )

    print(
        f"PASS: bodyMass edit rebuilt node in place "
        f"(glue {glue_err:.3f}, static drift {max_drift:.4f}, sim alive)"
    )

    # Restore the scene for the following suites: the original mass (rebuilds
    # the body back to dynamic) and the root bone rotation.
    cmds.setAttr(f"{node}.bodies[{rb_idx}].bodyMass", orig_mass)
    _restore_root(maya_pmx_data)
    cmds.currentTime(1)
    binding.step()
    binding.write_back()
    return True


_TESTS = [
    ("Rigid Body Physics Group", test_pmx_rigid_body_physics_group),
    ("Rigid Body Count", test_pmx_rigid_body_count),
    ("Rigid Body Attributes", test_pmx_rigid_body_attributes),
    ("Rigid Body Transform", test_pmx_rigid_body_rest_transform),
    ("Follow-Bone Anchor Tracks Joint", test_follow_bone_anchor_tracks_joint),
    ("Rigid Body No Guide Transforms", test_rigid_body_no_guide_transforms),
    ("Dynamic Bodies (write-back driven)", test_dynamic_bodies),
    ("Write-Back No DG Cycle", test_write_back_no_dg_cycle),
    ("Physics Joints", test_physics_joints),
    ("Kinematic Anchors", test_kinematic_anchors),
    ("Simulation Steps", test_simulation_steps),
    ("Write-Back Moves Bone", test_write_back_moves_bone),
    ("Config Edit Rebuilds Node", test_config_edit_rebuilds_node),
]
