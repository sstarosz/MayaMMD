"""
Integration tests for PMX rigid body physics creation in Maya.

**Milestone 2 (physics_builder rewrite — native C++ Bullet node):**

The old mayaBullet layer is GONE.  Physics is now driven by one native
``mmdPhysicsNode`` (a plain ``MPxNode`` in ``MayaMMD.mll``) that owns a
``btDiscreteDynamicsWorld``:

* ``FOLLOW_BONE`` bodies are **visible polygonal guide meshes** bound to their
  bone via ``parentConstraint`` (DG).  Their world/parent-inverse matrices feed
  the node's ``anchorWorldMatrix`` / ``anchorParentInverseMatrix`` inputs so the
  kinematic colliders track the bones every frame.
* ``PHYSICS`` / ``PHYSICS_BONE`` bodies are **dynamic**: the node writes each
  body's solved LOCAL transform to ``outTranslate[i]`` / ``outRotate[i]``,
  connected straight into the guide mesh transform; a ``parentConstraint``
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
    _RIGID_BODY_GROUP_COLORS,
    mmd_euler_to_maya_degrees,
)
from tests.integration.test_helpers import assert_eq, assert_true

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


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


def _get_binding(maya_pmx_data):
    """Return the model's physics binding (or None)."""
    return getattr(maya_pmx_data, "physics_binding", None)


def _iter_guides(maya_pmx_data):
    """Yield ``(rb_index, guide_name)`` for every created rigid-body guide."""
    binding = _get_binding(maya_pmx_data)
    if binding is None:
        return
    for rb_idx, guide in binding.bodies.items():
        if cmds.objExists(guide):
            yield rb_idx, guide


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
        _m.radians(world_r[0]), _m.radians(world_r[1]), _m.radians(world_r[2]),
        om.MEulerRotation.kXYZ,
    )
    mt.setRotation(e)
    return mt.asMatrix()


def _find_joint_by_bone_index(maya_pmx_data, bone_idx):
    """Return the Maya joint name whose pmxBoneIndex == bone_idx, or None."""
    for j in maya_pmx_data.joints:
        jn = om.MFnDependencyNode(j).name()
        if cmds.attributeQuery("pmxBoneIndex", node=jn, exists=True) and \
                cmds.getAttr(f"{jn}.pmxBoneIndex") == bone_idx:
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
    assert_true(
        cmds.isConnected("time1.outTime", f"{node}.time"),
        "mmdPhysicsNode.time not connected to time1.outTime",
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
    # One guide mesh per rigid body.
    n_guides = len(list(_iter_guides(maya_pmx_data)))
    assert_eq(
        n_guides,
        len(pmx_data.rigid_bodies),
        f"guide count {n_guides} != PMX rigid body count {len(pmx_data.rigid_bodies)}",
    )
    print(f"PASS: {n_bodies} bodies, {n_joints if pmx_data.joints else 0} joints in node")
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
        assert_eq(
            cmds.getAttr(f"{base}.bodyKinematic"),
            kin,
            f"body {rb_idx} bodyKinematic {cmds.getAttr(f'{base}.bodyKinematic')} != {kin}",
        )
        if not kin:
            assert_true(
                float(cmds.getAttr(f"{base}.bodyMass")) > 0.0,
                f"dynamic body {rb_idx} mass not > 0",
            )
        # Collision group encodes 1 << group_id.
        assert_eq(
            cmds.getAttr(f"{base}.bodyGroup"),
            1 << rb.group_id,
            f"body {rb_idx} bodyGroup != 1 << group_id",
        )
        assert_true(
            cmds.attributeQuery("bodyRestTranslate", node=node, exists=True),
            "bodyRestTranslate missing",
        )
        checked += 1
    assert_true(checked > 0, "No body attributes checked")
    print(f"PASS: {checked} node body elements carry PMX parameters")
    return True


def test_pmx_rigid_body_transform(pmx_data: PmxModel, maya_pmx_data):
    """Test FOLLOW_BONE guides: world transform fidelity + DG joint binding."""
    fb = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value == 0]
    if not fb:
        print("SKIP: model has no FOLLOW_BONE rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None, "No physics binding")

    checked = 0
    for rb_idx, guide in _iter_guides(maya_pmx_data):
        rb = pmx_data.rigid_bodies[rb_idx]
        if rb.physics_mode.value != 0:
            continue  # dynamic bodies are solver-driven — not checked here

        # FOLLOW_BONE bodies must be bound to their related joint via a
        # parentConstraint (DG), not DAG parenting.
        if rb.related_bone_index >= 0:
            jpath = _find_joint_by_bone_index(maya_pmx_data, rb.related_bone_index)
            assert_true(
                jpath is not None,
                f"FOLLOW_BONE body {guide} has no joint for bone {rb.related_bone_index}",
            )
            cons = cmds.listConnections(guide, type="parentConstraint") or []
            assert_true(
                bool(cons),
                f"FOLLOW_BONE body {guide} has no parentConstraint to its joint",
            )

        # World transform must match PMX data exactly (Z-flip + handedness).
        pos = rb.shape_position
        rot = rb.shape_rotation
        expected_t = (pos.x, pos.y, -pos.z)

        world_t = cmds.xform(guide, q=True, ws=True, translation=True)
        for got, exp, label in zip(world_t, expected_t, ("X", "Y", "Z")):
            assert_true(
                abs(got - exp) < 1e-3,
                f"Body {guide} translate{label} {got} != expected {exp}",
            )

        # Rotation: compare world MATRICES (Euler read-back is non-unique).
        got = cmds.xform(guide, q=True, ws=True, matrix=True)
        got_m = om.MMatrix([got[0:4], got[4:8], got[8:12], got[12:16]])
        exp_m = _expected_world_matrix(pos, rot)
        matrix_err = max(abs(got_m[i] - exp_m[i]) for i in range(16))
        assert_true(matrix_err < 1e-3, f"Body {guide} world matrix err {matrix_err:.3e}")

        # Handedness check: world Y basis must equal the flipped rotation image.
        world_y = (got[4], got[5], got[6])
        expected_y = _mmd_flipped_y_axis(rot.x, rot.y, rot.z)
        for gotv, exp, label in zip(world_y, expected_y, ("X", "Y", "Z")):
            assert_true(
                abs(gotv - exp) < 1e-3,
                f"Body {guide} world Y-axis {label} {gotv} != expected {exp}",
            )
        checked += 1

    assert_true(checked > 0, "No FOLLOW_BONE bodies found to check transforms")
    print(f"PASS: {checked} FOLLOW_BONE bodies at PMX world pose, DG-bound to joints")
    return True


def test_follow_bone_tracks_joint_rotation(pmx_data: PmxModel, maya_pmx_data):
    """Test that a FOLLOW_BONE body tracks its joint when the bone rotates."""
    fb = [(i, rb) for i, rb in enumerate(pmx_data.rigid_bodies)
          if rb.physics_mode.value == 0]
    if not fb:
        print("SKIP: model has no FOLLOW_BONE rigid bodies")
        return True

    import math as _m

    target = None
    for rb_idx, rb in fb:
        if rb.related_bone_index >= 0:
            target = (rb_idx, rb)
            break
    if target is None:
        print("SKIP: no FOLLOW_BONE body with a related bone")
        return True

    rb_idx, rb = target
    guide = None
    for idx, g in _iter_guides(maya_pmx_data):
        if idx == rb_idx:
            guide = g
            break
    assert_true(guide is not None, f"FOLLOW_BONE body {rb_idx} not found")

    jpath = _find_joint_by_bone_index(maya_pmx_data, rb.related_bone_index)
    assert_true(jpath is not None, f"Joint for bone {rb.related_bone_index} not found")

    t0 = cmds.xform(guide, q=True, ws=True, translation=True)
    j0 = cmds.xform(jpath, q=True, ws=True, translation=True)
    off = [t0[i] - j0[i] for i in range(3)]

    cmds.setAttr(f"{jpath}.rotateY", 30.0)
    cmds.currentTime(cmds.currentTime(query=True))

    t1 = cmds.xform(guide, q=True, ws=True, translation=True)
    j1 = cmds.xform(jpath, q=True, ws=True, translation=True)
    c, s = _m.cos(_m.radians(30.0)), _m.sin(_m.radians(30.0))
    rot_off = (off[0] * c + off[2] * s, off[1], -off[0] * s + off[2] * c)
    exp = [j1[i] + rot_off[i] for i in range(3)]
    err = max(abs(t1[i] - exp[i]) for i in range(3))
    assert_true(
        err < 1e-2,
        f"FOLLOW_BONE body {rb_idx} did not track its joint (err {err:.5f})",
    )

    # restore
    cmds.setAttr(f"{jpath}.rotateY", 0.0)
    cmds.currentTime(cmds.currentTime(query=True))
    print(f"PASS: FOLLOW_BONE body {rb_idx} tracks joint after 30 deg rotation (err {err:.2e})")
    return True


def test_pmx_rigid_body_group_colors(pmx_data: PmxModel, maya_pmx_data):
    """Test per-group color coding of the guide meshes (unique Lambert per group)."""
    fb = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value == 0]
    if not fb:
        print("SKIP: model has no FOLLOW_BONE rigid bodies")
        return True
    unique_groups = sorted({rb.group_id for rb in fb})
    colored = set()
    for rb_idx, guide in _iter_guides(maya_pmx_data):
        rb = pmx_data.rigid_bodies[rb_idx]
        gid = rb.group_id
        mesh_shapes = cmds.listRelatives(guide, shapes=True, type="mesh") or []
        assert_true(bool(mesh_shapes), f"Body {guide} has no guide mesh")
        for ms in mesh_shapes:
            # Each guide shape is shaded by one unique shader per group whose
            # color matches the group's palette entry (no draw-override tinting).
            sgs = cmds.listConnections(ms, type="shadingEngine") or []
            assert_true(bool(sgs), f"Guide {ms} has no shading group")
            shaders = cmds.listConnections(
                f"{sgs[0]}.surfaceShader", source=True, destination=False
            )
            assert_true(bool(shaders), f"Shading group {sgs[0]} has no shader")
            sh = shaders[0]
            # Maya 2024+ standard shader stores its color in baseColor;
            # Lambert (fallback on older releases) uses `color`.
            color_attr = "baseColor" if cmds.nodeType(sh) == "openPBRSurface" else "color"
            got = cmds.getAttr(f"{sh}.{color_attr}")[0]
            r, g, b = _RIGID_BODY_GROUP_COLORS[gid % len(_RIGID_BODY_GROUP_COLORS)]
            for gv, exp, label in zip(got, (r, g, b), ("R", "G", "B")):
                assert_true(
                    abs(gv - exp) < 1e-3,
                    f"Group {gid} guide {ms} color{label} {gv} != expected {exp}",
                )
        colored.add(gid)
    for gid in unique_groups:
        assert_true(gid in colored, f"Collision group {gid} has no colored body")
    print(f"PASS: {len(unique_groups)} collision groups shaded on guide meshes")
    return True


def test_dynamic_bodies(pmx_data: PmxModel, maya_pmx_data):
    """Test dynamic guides: node-output drive + DG write-back to the bone."""
    dyn = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value in (1, 2)]
    if not dyn:
        print("SKIP: model has no dynamic rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")
    node = binding.node

    checked = 0
    for rb_idx, guide in _iter_guides(maya_pmx_data):
        mode = cmds.getAttr(f"{guide}.pmxPhysicsMode")
        if mode not in (1, 2):
            continue
        rb = pmx_data.rigid_bodies[rb_idx]
        # Guide transform driven by the node's solved pose (rotate may go
        # through Maya's auto-inserted unitConversion).
        assert_true(
            _is_driven_by(f"{node}.outTranslate[{rb_idx}].outTranslateValue",
                          f"{guide}.translate"),
            f"Dynamic body {guide} translate not driven by node output",
        )
        assert_true(
            _is_driven_by(f"{node}.outRotate[{rb_idx}].outRotateValue",
                          f"{guide}.rotate"),
            f"Dynamic body {guide} rotate not driven by node output",
        )
        mesh_shapes = cmds.listRelatives(guide, shapes=True, type="mesh") or []
        assert_true(bool(mesh_shapes), f"Dynamic body {guide} has no guide mesh")
        # DG write-back to the related bone.
        if rb.related_bone_index >= 0:
            con_type = "parentConstraint" if mode == 1 else "orientConstraint"
            cons = cmds.listConnections(guide, type=con_type) or []
            assert_true(
                bool(cons),
                f"Dynamic body {guide} (mode {mode}) has no {con_type} write-back",
            )
        checked += 1
    assert_true(checked > 0, "No dynamic bodies found to check")
    print(f"PASS: {checked} dynamic bodies driven by node outputs + DG write-back")
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
    assert_eq(n, len(pmx_data.joints), f"node.joints size {n} != {len(pmx_data.joints)}")
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
    """Test that FOLLOW_BONE guides feed the node's kinematic anchors."""
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
    # Every FOLLOW_BONE guide must be connected to an anchor slot.
    connected = 0
    for rb_idx, guide in _iter_guides(maya_pmx_data):
        if pmx_data.rigid_bodies[rb_idx].physics_mode.value != 0:
            continue
        srcs = cmds.listConnections(f"{node}.anchorWorldMatrix", source=True) or []
        assert_true(
            any(f"{guide}.worldMatrix[0]" == f"{s}.worldMatrix[0]" for s in srcs),
            f"FOLLOW_BONE guide {guide} not connected to an anchorWorldMatrix",
        )
        connected += 1
    assert_eq(connected, len(fb), "not all FOLLOW_BONE guides anchored")
    print(f"PASS: {connected} kinematic anchors fed by FOLLOW_BONE guides")
    return True


# ---------------------------------------------------------------------------
# BEHAVIOURAL tests — the core of Milestone 2
# ---------------------------------------------------------------------------


def test_simulation_steps(pmx_data: PmxModel, maya_pmx_data):
    """BEHAVIOURAL: stepping time MUST move at least one dynamic body.

    A frozen solver leaves EVERY dynamic body at rest, so this detects it.
    (Some bodies are rigid chains that legitimately hold still — e.g. a rigid
    cape — so we scan a set of candidates and require at least one to move.)
    """
    dyn = [rb for rb in pmx_data.rigid_bodies if rb.physics_mode.value in (1, 2)]
    if not dyn:
        print("SKIP: model has no dynamic rigid bodies")
        return True
    binding = _get_binding(maya_pmx_data)
    assert_true(binding is not None and binding.node, "No physics binding/node")

    candidates = [
        (rb_idx, guide)
        for rb_idx, guide in _iter_guides(maya_pmx_data)
        if pmx_data.rigid_bodies[rb_idx].physics_mode.value in (1, 2)
    ]
    if not candidates:
        print("SKIP: no dynamic body guides")
        return True

    cmds.currentTime(1)
    binding.step()
    starts = [
        (rb_idx, guide, cmds.xform(guide, q=True, ws=True, translation=True))
        for rb_idx, guide in candidates
    ]

    moved = None
    for f in (5, 10, 15, 20, 25, 30):
        cmds.currentTime(f)
        binding.step()
        for rb_idx, guide, p0 in starts:
            p = cmds.xform(guide, q=True, ws=True, translation=True)
            if max(abs(p[i] - p0[i]) for i in range(3)) > 0.01:
                moved = guide
                break
        if moved:
            break

    assert_true(
        moved is not None,
        "No dynamic body moved while stepping time — simulation frozen",
    )
    print(f"PASS: dynamic body {moved} moved during playback")

    # Rewind: dt < 0 makes the node rebuild the Bullet world at rest, which
    # restores the scene for the following suites.
    cmds.currentTime(1)
    binding.step()
    return True


def _world_quat(name):
    """World-space quaternion of a transform (unambiguous vs Euler readback)."""
    m = cmds.xform(name, q=True, ws=True, matrix=True)
    mt = om.MTransformationMatrix(om.MMatrix(m))
    return mt.rotation(asQuaternion=True)


def _quat_angle_deg(a, b):
    """Rotation angle (degrees) between two unit quaternions.

    ``MQuaternion`` has no ``angleTo`` in API 2.0, so compute it from the
    4-D dot product; ``abs`` handles the q/-q double-cover of rotations.
    """
    d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w
    d = min(1.0, max(-1.0, abs(d)))
    return 2.0 * math.degrees(math.acos(d))


def test_write_back_moves_bone(pmx_data: PmxModel, maya_pmx_data):
    """BEHAVIOURAL: a dynamic body's related bone follows the solved pose.

    PHYSICS (mode 1) uses a parentConstraint — translation and/or rotation.
    PHYSICS_BONE (mode 2) uses an orientConstraint — rotation.  We step the
    sim, pick the dynamic body that moved the MOST, and assert its bone
    followed.  For mode 1 a significant ROTATION also counts (pivot-anchored
    bodies like ties/cloth swing around their pivot with little translation).
    This is the "mesh binding" the user reported as lost when the solver froze.
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
        (rb_idx, guide, pmx_data.rigid_bodies[rb_idx])
        for rb_idx, guide in _iter_guides(maya_pmx_data)
        if pmx_data.rigid_bodies[rb_idx].physics_mode.value in (1, 2)
        and pmx_data.rigid_bodies[rb_idx].related_bone_index >= 0
    ]
    if not candidates:
        print("SKIP: no dynamic body with a related bone")
        return True

    # Step the sim, then pick the body that moved the most.
    cmds.currentTime(1)
    binding.step()
    starts = [
        (rb_idx, guide, rb, cmds.xform(guide, q=True, ws=True, translation=True))
        for rb_idx, guide, rb in candidates
    ]
    for f in (5, 10, 15, 20, 25, 30):
        cmds.currentTime(f)
        binding.step()
    binding.write_back()

    def _displacement(s):
        p = cmds.xform(s[1], q=True, ws=True, translation=True)
        return max(abs(p[i] - s[3][i]) for i in range(3))

    best = max(starts, key=_displacement)
    rb_idx, guide, rb, _p0 = best
    mode = rb.physics_mode.value
    jpath = _find_joint_by_bone_index(maya_pmx_data, rb.related_bone_index)
    assert_true(jpath is not None, f"No joint for bone {rb.related_bone_index}")

    t0 = cmds.xform(jpath, q=True, ws=True, translation=True)
    q0 = _world_quat(jpath)
    for f in (10, 20, 30):
        cmds.currentTime(f)
        binding.step()
    binding.write_back()
    t1 = cmds.xform(jpath, q=True, ws=True, translation=True)
    q1 = _world_quat(jpath)

    dT = max(abs(t1[i] - t0[i]) for i in range(3))
    dR = _quat_angle_deg(q0, q1)
    ok = dT > 0.005 if mode == 1 else dR > 0.005
    if mode == 1 and not ok:
        # pivot-anchored bodies (ties, cloth) swing: rotation is the signal
        ok = dR > 0.5
    assert_true(
        ok,
        f"Bone {jpath} did not follow dynamic body {guide} "
        f"(mode {mode}, dT={dT:.4f}, dR={dR:.4f}) — write-back broken",
    )
    print(f"PASS: bone {jpath} follows dynamic body {guide} (mode {mode}, dT={dT:.3f}, dR={dR:.3f})")

    # Rewind to restore the scene.
    cmds.currentTime(1)
    binding.step()
    binding.write_back()
    return True


# ---------------------------------------------------------------------------
# Test registry and runner
# ---------------------------------------------------------------------------

_TESTS = [
    ("Rigid Body Physics Group", test_pmx_rigid_body_physics_group),
    ("Rigid Body Count", test_pmx_rigid_body_count),
    ("Rigid Body Attributes", test_pmx_rigid_body_attributes),
    ("Rigid Body Transform", test_pmx_rigid_body_transform),
    ("Follow-Bone Tracks Joint", test_follow_bone_tracks_joint_rotation),
    ("Rigid Body Group Colors", test_pmx_rigid_body_group_colors),
    ("Dynamic Bodies", test_dynamic_bodies),
    ("Physics Joints", test_physics_joints),
    ("Kinematic Anchors", test_kinematic_anchors),
    ("Simulation Steps (behavioral)", test_simulation_steps),
    ("Write-Back Moves Bone (behavioral)", test_write_back_moves_bone),
]
