"""
Integration tests for general PMX import functionality in Maya.

Tests core import components: root, geometry group, mesh, hierarchy,
materials, and skin cluster. Bone-specific tests are in test_pmx_bone_integration.py
and morph-specific tests are in test_pmx_morph_integration.py.
"""

# ── Maya standalone initialised by the test runner ───────────────────────
import math

import maya.api.OpenMaya as om
import maya.api.OpenMayaAnim as oma
from maya import cmds

from mmd.core.data_types import PmxModel
from tests.integration.test_helpers import (
    approx_equal_tuple,
    assert_eq,
    assert_true,
    step_physics,
)

_NODE_TYPE = "pmxRigidBodyNode"


# ---------------------------------------------------------------------------
# General PMX import tests (root, geometry, mesh, hierarchy, materials, skin,
# physics)
# ---------------------------------------------------------------------------


def test_pmx_root_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that PMX root node is created.

    Result-oriented: verifies a transform node with ``PMX_`` prefix
    and ``_Root`` suffix exists under the world root.
    """
    root_obj = maya_pmx_data.root_obj
    root_fn = om.MFnTransform(root_obj)

    name = root_fn.name()
    assert_true(name.endswith("_Root"), f"Root node name unexpected: {name}")
    print(f"PASS: Root node created successfully: {name}")
    return True


def test_pmx_geometry_group_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that geometry group is created under root"""
    root_obj = maya_pmx_data.root_obj
    root_fn = om.MFnTransform(root_obj)
    children = [om.MFnTransform(root_fn.child(i)) for i in range(root_fn.childCount())]
    geo_group = None
    for child in children:
        if child.name().endswith("_Geo"):
            geo_group = child
            break
    assert_true(geo_group is not None, "Geo group not found")
    parent_name = om.MFnTransform(geo_group.parent(0)).name()
    assert_eq(parent_name, root_fn.name(), "Geo has incorrect parent")
    print("PASS: Geo group created with correct hierarchy")
    return True


def test_pmx_mesh_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that mesh is created with correct properties"""
    mesh_node = maya_pmx_data.mesh_node
    mesh_fn = om.MFnMesh(mesh_node)
    vertex_count = mesh_fn.numVertices
    assert_true(vertex_count > 0, "PMX_Mesh has no vertices")
    print(f"PASS: PMX_Mesh created with {vertex_count} vertices")
    face_count = mesh_fn.numPolygons
    assert_true(face_count > 0, "PMX_Mesh has no faces")
    print(f"PASS: PMX_Mesh created with {face_count} faces")
    uv_sets = mesh_fn.getUVSetNames()
    assert_true(uv_sets and "map1" in uv_sets, "UV set 'map1' not found")
    print("PASS: UV set 'map1' exists")
    return True


def test_pmx_hierarchy(pmx_data: PmxModel, maya_pmx_data):
    """Test the complete scene hierarchy — result-oriented.

    Verifies that:
    - A root node exists with ``{model}_Root`` naming.
    - A geometry group (``{model}_Geo``) is a direct child of the root.
    - A mesh transform (``{model}_Mesh``) is a child of the geo group.
    - A mesh shape (``{model}_Mesh_Shape``) is a child of the mesh transform.
    """
    root_obj = maya_pmx_data.root_obj
    root_fn = om.MFnTransform(root_obj)

    assert_true(
        root_fn.name().endswith("_Root"), f"Root node name unexpected: {root_fn.name()}"
    )

    # Find the geo group under root
    geo_group = None
    for i in range(root_fn.childCount()):
        child = root_fn.child(i)
        child_fn = om.MFnTransform(child)
        if child_fn.name().endswith("_Geo"):
            geo_group = child_fn
            break
    assert_true(geo_group is not None, "Geo group not found")
    assert_true(
        om.MFnTransform(geo_group.parent(0)).name() == root_fn.name(),
        "Geo parent is not root",
    )

    # Find the mesh transform under geo group
    mesh_transform = None
    for i in range(geo_group.childCount()):
        child = geo_group.child(i)
        child_fn = om.MFnTransform(child)
        if child_fn.name().endswith("_Mesh"):
            mesh_transform = child_fn
            break
    assert_true(mesh_transform is not None, "Mesh not found under Geo")

    # Find mesh shape under mesh transform
    mesh_shape = None
    for i in range(mesh_transform.childCount()):
        child = mesh_transform.child(i)
        try:
            shape_fn = om.MFnMesh(child)
            if shape_fn.name().endswith("_Mesh_Shape"):
                mesh_shape = shape_fn
                break
        except Exception:
            # Child is not a mesh (e.g. a shape of a different type) — skip
            continue
    assert_true(mesh_shape is not None, "Mesh_Shape not found under Mesh")

    print("PASS: All hierarchy nodes exist and are correctly parented")
    return True


def test_pmx_material_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that PMX materials are created correctly — result-oriented.

    Verifies that the scene contains at least as many ``PMX_*_Mat`` materials
    as there are materials in the PMX data.  This check is independent of the
    exact naming algorithm — it just confirms materials were created.
    """
    scene_materials = set(cmds.ls(materials=True))
    pmx_materials = {m for m in scene_materials if m.endswith("_Mat")}
    expected_count = len(pmx_data.materials)

    assert_true(
        len(pmx_materials) >= expected_count,
        f"Found {len(pmx_materials)} PMX materials, expected at least {expected_count}",
    )
    print(f"PASS: {len(pmx_materials)} materials created (≥ {expected_count} expected)")
    return True


def test_pmx_skin_cluster_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that skin cluster is created"""
    skin_cluster_obj = maya_pmx_data.skin_cluster
    assert_true(
        skin_cluster_obj is not None and not skin_cluster_obj.isNull(),
        "Skin cluster was not created",
    )
    skin_cluster_fn = om.MFnDependencyNode(skin_cluster_obj)
    node_type = skin_cluster_fn.typeName
    assert_eq(
        node_type, "skinCluster", f"Object is not a skin cluster, got type: {node_type}"
    )
    print(f"PASS: Skin cluster created: {skin_cluster_fn.name()}")
    return True


def test_pmx_skin_weights_applied(pmx_data: PmxModel, maya_pmx_data):
    """Test that skin weights are applied to vertices"""
    skin_cluster_obj = maya_pmx_data.skin_cluster
    assert_true(
        skin_cluster_obj is not None and not skin_cluster_obj.isNull(),
        "No skin cluster to test weights",
    )

    skin_cluster_fn = oma.MFnSkinCluster(skin_cluster_obj)
    influence_paths = skin_cluster_fn.influenceObjects()
    influence_count = len(influence_paths)
    assert_true(influence_count > 0, "Skin cluster has no influences")
    print(f"PASS: Skin cluster has {influence_count} influences")

    mesh_fn = om.MFnMesh(maya_pmx_data.mesh_node)
    vertex_count = mesh_fn.numVertices
    sample_size = min(10, vertex_count)
    vertices_with_weights = 0

    for v_idx in range(sample_size):
        vertex_comp = om.MFnSingleIndexedComponent()
        vertex_obj = vertex_comp.create(om.MFn.kMeshVertComponent)
        vertex_comp.addElement(v_idx)
        weights, _ = skin_cluster_fn.getWeights(
            om.MDagPath.getAPathTo(maya_pmx_data.mesh_node), vertex_obj
        )
        if any(w > 0.0 for w in weights):
            vertices_with_weights += 1

    assert_true(vertices_with_weights > 0, "No vertices have skin weights")
    print(
        f"PASS: {vertices_with_weights}/{sample_size} sampled vertices have skin weights"
    )
    return True


def test_pmx_rigid_body_node_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that the rigid bodies group + populated pmxRigidBodyNode are created per model.

    ``build_pmx_scene`` creates a ``{model}_RigidBodies`` group under the root with
    one ``pmxRigidBodyNode`` solver (gravity -9.8), POPULATES its ``bodies``
    array through the native ``pmxRigidBody`` command (one body per PMX rigid
    body, in PMX order), and stamps the solver name on the root's
    ``pmxRigidBodyNode`` string attribute for discovery.
    """
    root_obj = maya_pmx_data.root_obj
    root_fn = om.MFnTransform(root_obj)
    root_name = root_fn.name()

    # Find the rigid bodies group under the root (mirror the _Geo lookup).
    physics_group = None
    for i in range(root_fn.childCount()):
        child = root_fn.child(i)
        child_fn = om.MFnTransform(child)
        if child_fn.name().endswith("_RigidBodies"):
            physics_group = child_fn
            break
    assert_true(physics_group is not None, "Rigid bodies group not found under root")
    assert_eq(
        om.MFnTransform(physics_group.parent(0)).name(),
        root_name,
        "Rigid bodies group parent is not root",
    )

    # Exactly one pmxRigidBodyNode solver shape under the group.
    solvers = cmds.listRelatives(physics_group.name(), children=True, type=_NODE_TYPE)
    assert_true(bool(solvers), "No pmxRigidBodyNode under the rigid bodies group")
    solver = solvers[0]
    assert_eq(cmds.nodeType(solver), _NODE_TYPE, "solver has the wrong node type")

    # Gravity default is exactly MMD's -9.8 on Y.
    grav = cmds.getAttr(f"{solver}.gravity")[0]
    assert_eq(round(grav[1], 4), -9.8, "rigid body node gravity y")

    # The solver name is stamped on the root for discovery.
    assert_true(
        cmds.attributeQuery("pmxRigidBodyNode", node=root_name, exists=True),
        "pmxRigidBodyNode root attribute missing",
    )
    assert_eq(
        cmds.getAttr(f"{root_name}.pmxRigidBodyNode"),
        solver,
        "root pmxRigidBodyNode attribute should name the solver",
    )

    # The bodies array mirrors the PMX rigid bodies (one body per rigid body,
    # in PMX order — appended by the native pmxRigidBody command).
    expected_bodies = len(pmx_data.rigid_bodies)
    assert_true(
        cmds.attributeQuery("bodies", node=solver, exists=True),
        "solver has no bodies attribute",
    )
    if expected_bodies > 0:
        assert_eq(
            int(cmds.getAttr(f"{solver}.bodies", size=True)),
            expected_bodies,
            "bodies count != PMX rigid body count",
        )
        # First body's data mirrors the first PMX rigid body.
        first = pmx_data.rigid_bodies[0]
        base = f"{solver}.bodies[0]"
        assert_eq(
            cmds.getAttr(f"{base}.bodyNameLocal"),
            first.name_local,
            "first body name_local",
        )
        assert_eq(
            int(cmds.getAttr(f"{base}.bodyPhysicsMode")),
            int(first.physics_mode.value),
            "first body physics mode",
        )
    else:
        # Models with no rigid bodies still get the node (empty bodies = no-op).
        assert_eq(
            int(cmds.getAttr(f"{solver}.bodies", size=True) or 0),
            0,
            "no-body model should have an empty bodies array",
        )

    # The joints array mirrors the PMX rigid-body constraints (one joint per
    # PMX joint, in PMX order — appended by the native pmxRigidBodyConstraint
    # command after every body exists).
    expected_joints = len(pmx_data.joints)
    assert_true(
        cmds.attributeQuery("joints", node=solver, exists=True),
        "solver has no joints attribute",
    )
    if expected_joints > 0:
        assert_eq(
            int(cmds.getAttr(f"{solver}.joints", size=True)),
            expected_joints,
            "joints count != PMX joint count",
        )
        # First joint's data mirrors the first PMX joint: body refs, type,
        # name, and the frame/limit conversion (Z-flip on position; angular
        # limits reflected through F = diag(1,1,-1): X/Y negated + swapped,
        # Z unchanged).
        first = pmx_data.joints[0]
        jbase = f"{solver}.joints[0]"
        assert_eq(
            int(cmds.getAttr(f"{jbase}.jointBodyA")),
            int(first.rigid_body_index_a),
            "first joint bodyA",
        )
        assert_eq(
            int(cmds.getAttr(f"{jbase}.jointBodyB")),
            int(first.rigid_body_index_b),
            "first joint bodyB",
        )
        assert_eq(
            int(cmds.getAttr(f"{jbase}.jointType")),
            int(first.type.value),
            "first joint type",
        )
        assert_eq(
            cmds.getAttr(f"{jbase}.jointNameLocal"),
            first.name_local,
            "first joint name_local",
        )
        assert_true(
            approx_equal_tuple(
                cmds.getAttr(f"{jbase}.jointFrameTranslate")[0],
                (first.position.x, first.position.y, -first.position.z),
            ),
            f"first joint frame translate Z-flip wrong "
            f"{cmds.getAttr(f'{jbase}.jointFrameTranslate')}",
        )
        assert_true(
            approx_equal_tuple(
                cmds.getAttr(f"{jbase}.jointAngularMin")[0],
                (
                    -first.rotation_max.x,
                    -first.rotation_max.y,
                    first.rotation_min.z,
                ),
            ),
            f"first joint angularMin reflection wrong "
            f"{cmds.getAttr(f'{jbase}.jointAngularMin')}",
        )
        assert_true(
            approx_equal_tuple(
                cmds.getAttr(f"{jbase}.jointAngularMax")[0],
                (
                    -first.rotation_min.x,
                    -first.rotation_min.y,
                    first.rotation_max.z,
                ),
            ),
            f"first joint angularMax reflection wrong "
            f"{cmds.getAttr(f'{jbase}.jointAngularMax')}",
        )
    else:
        # Models with no joints get an empty joints array.
        assert_eq(
            int(cmds.getAttr(f"{solver}.joints", size=True) or 0),
            0,
            "no-joint model should have an empty joints array",
        )
    print(
        f"PASS: rigid bodies group + {_NODE_TYPE} with {expected_bodies} bodies, "
        f"{expected_joints} joints created"
    )
    return True


# ---------------------------------------------------------------------------
# Physics simulation wiring + behavioural tests
# ---------------------------------------------------------------------------


def _find_rigid_bodies_group_and_solver(maya_pmx_data):
    """Return ``(rigid_bodies_group_name, solver_name)`` or ``(None, None)``."""
    root_obj = maya_pmx_data.root_obj
    root_fn = om.MFnTransform(root_obj)
    for i in range(root_fn.childCount()):
        child = root_fn.child(i)
        try:
            child_fn = om.MFnTransform(child)
        except Exception:
            continue
        if child_fn.name().endswith("_RigidBodies"):
            solvers = cmds.listRelatives(
                child_fn.name(), children=True, type=_NODE_TYPE
            )
            if solvers:
                return child_fn.name(), solvers[0]
    return None, None


def _joint_paths(maya_pmx_data) -> dict:
    """PMX bone index -> full joint path (skips failed/null joints)."""
    names: dict = {}
    for b_idx, j_obj in enumerate(maya_pmx_data.joints):
        if not j_obj.isNull():
            try:
                names[b_idx] = om.MFnDagNode(j_obj).fullPathName()
            except Exception:
                continue
    return names


def _root_joint(maya_pmx_data) -> str:
    """Name of the model's root bone joint (drives the whole skeleton)."""
    return om.MFnDagNode(maya_pmx_data.joints[0]).partialPathName()


def _swing_root(maya_pmx_data, frame: int, degrees: float = 20.0) -> None:
    """Rotate the root bone sinusoidally so the dynamic chains keep moving."""
    angle = degrees * math.sin(math.radians(frame * 12.0))
    root = _root_joint(maya_pmx_data)
    cmds.setAttr(f"{root}.rotateZ", angle)
    cmds.dgdirty(root)


def test_pmx_physics_wiring(pmx_data: PmxModel, maya_pmx_data):
    """SIMULATION IS ENABLED: time-driven solver + write-back wiring.

    The solver is connected to ``time1.outTime``, dynamic bodies carry their
    related joint as a MESSAGE (the node resolves the write-back parent +
    scrub-back reset anchor from it + the joint DAG), and pmxRigidBody ALWAYS
    connects ``outTranslate``/``outRotate`` STRAIGHT into the related joints
    (rotation-only for PHYSICS_BONE).  Bodies without a related joint (static
    colliders) have nothing to connect.
    """
    _group, solver = _find_rigid_bodies_group_and_solver(maya_pmx_data)
    assert_true(solver is not None, "No physics solver")
    joint_names = _joint_paths(maya_pmx_data)

    follow_bone = 0
    physics_bone = 2

    # Time-driven.
    assert_true(
        cmds.isConnected("time1.outTime", f"{solver}.time"),
        "node.time is not connected (simulation not time-driven)",
    )

    dynamic = 0
    out_translate = 0
    for rb_idx, rb in enumerate(pmx_data.rigid_bodies):
        if rb.physics_mode.value == follow_bone or rb.related_bone_index < 0:
            continue
        bone = rb.related_bone_index
        jpath = joint_names.get(bone)
        if not jpath:
            continue
        dynamic += 1
        # The body's related joint is connected as a MESSAGE by pmxRigidBody
        # (bodies[i].bodyJoint -> joint.message) — the node resolves the
        # write-back parent and the reset anchor from it + the joint DAG, so
        # no per-body wiring inputs exist.
        joint_srcs = (
            cmds.listConnections(f"{solver}.bodies[{rb_idx}].bodyJoint", source=True)
            or []
        )
        # listConnections returns SHORT node names; jpath is a full path.
        short = jpath.rpartition("|")[2]
        assert_true(
            bool(joint_srcs)
            and any(str(s).rpartition("|")[2] == short for s in joint_srcs),
            f"body {rb_idx} bodyJoint not connected to {jpath} ({joint_srcs})",
        )
        # pmxRigidBody ALWAYS wires a dynamic body on a bone: outRotate (and
        # outTranslate unless PHYSICS_BONE, which is rotation-only) connect
        # STRAIGHT into the joint.  The node's outputs are unit-typed
        # compounds (kAngle/kDistance) connected DIRECTLY — the destination
        # must be the joint itself, never a unitConversion (the bone builder's
        # own IK conversions are separate).
        rot_dests = (
            cmds.listConnections(f"{solver}.outRotate[{rb_idx}]", destination=True)
            or []
        )
        assert_true(
            bool(rot_dests) and all("unitConversion" not in str(d) for d in rot_dests),
            f"outRotate[{rb_idx}] not connected directly to the joint",
        )
        if rb.physics_mode.value != physics_bone:
            tr_dests = (
                cmds.listConnections(
                    f"{solver}.outTranslate[{rb_idx}]", destination=True
                )
                or []
            )
            assert_true(
                bool(tr_dests)
                and all("unitConversion" not in str(d) for d in tr_dests),
                f"outTranslate[{rb_idx}] not connected directly to the joint",
            )
            out_translate += 1
        else:
            tr_srcs = cmds.listConnections(f"{jpath}.translate", source=True) or []
            assert_true(
                not tr_srcs, f"PHYSICS_BONE joint {jpath} translate should be free"
            )
    assert_true(dynamic > 0, "no dynamic body with a related joint to verify")

    print(f"PASS: sim wired — {dynamic} driven joints ({out_translate} with translate)")
    return True


def test_pmx_simulation_steps(pmx_data: PmxModel, maya_pmx_data):
    """BEHAVIOURAL: the time-driven sim steps and moves the dynamic chains.

    Advancing time while swinging the root bone must change at least one
    dynamic joint's LOCAL pose — the "simulation is alive" signal.
    """
    _group, solver = _find_rigid_bodies_group_and_solver(maya_pmx_data)
    assert_true(solver is not None, "No physics solver")
    joint_names = _joint_paths(maya_pmx_data)
    dyn = {
        rb_idx: joint_names[rb.related_bone_index]
        for rb_idx, rb in enumerate(pmx_data.rigid_bodies)
        if rb.physics_mode.value in (1, 2) and rb.related_bone_index in joint_names
    }
    if not dyn:
        print("SKIP: model has no dynamic rigid bodies with a related joint")
        return True

    def _local_rot(jpath):
        return cmds.getAttr(f"{jpath}.rotate")[0]

    cmds.currentTime(1)
    step_physics(solver)
    starts = {rb: _local_rot(j) for rb, j in dyn.items()}
    for f in (5, 10, 15, 20, 25, 30):
        _swing_root(maya_pmx_data, f)
        cmds.currentTime(f)
        step_physics(solver)
    moved = sum(
        1
        for rb, j in dyn.items()
        if max(abs(cmds.getAttr(f"{j}.rotate")[0][i] - starts[rb][i]) for i in range(3))
        > 0.05
    )
    assert_true(moved > 0, "no dynamic joint moved after stepping — sim not alive")
    print(f"PASS: simulation steps — {moved}/{len(dyn)} dynamic joints moved")
    return True


def test_pmx_write_back_moves_bone(pmx_data: PmxModel, maya_pmx_data):
    """BEHAVIOURAL: the node's solved pose reaches the related JOINTS.

    The most-moved dynamic joint's LOCAL pose must change from its rest —
    the "mesh binding" signal (if the solver froze, no dynamic joint would
    move).  Rigid chains legitimately hold still, so we scan all dynamic
    bodies and use the strongest signal.
    """
    _group, solver = _find_rigid_bodies_group_and_solver(maya_pmx_data)
    assert_true(solver is not None, "No physics solver")
    joint_names = _joint_paths(maya_pmx_data)
    candidates = [
        (rb_idx, joint_names[rb.related_bone_index], rb)
        for rb_idx, rb in enumerate(pmx_data.rigid_bodies)
        if rb.physics_mode.value in (1, 2) and rb.related_bone_index in joint_names
    ]
    if not candidates:
        print("SKIP: no dynamic body with a related bone")
        return True

    def _local(jpath):
        return (
            cmds.getAttr(f"{jpath}.rotate")[0],
            cmds.getAttr(f"{jpath}.translate")[0],
        )

    cmds.currentTime(1)
    step_physics(solver)
    starts = [(rb_idx, jpath, rb, _local(jpath)) for rb_idx, jpath, rb in candidates]
    for f in (5, 10, 15, 20, 25, 30):
        _swing_root(maya_pmx_data, f)
        cmds.currentTime(f)
        step_physics(solver)

    def _disp(s):
        _rb_idx, jpath, _rb, p0 = s
        rot = cmds.getAttr(f"{jpath}.rotate")[0]
        tr = cmds.getAttr(f"{jpath}.translate")[0]
        return max(
            max(abs(rot[i] - p0[0][i]) for i in range(3)),
            max(abs(tr[i] - p0[1][i]) for i in range(3)),
        )

    best = max(starts, key=_disp)
    rb_idx, jpath, rb, p0 = best
    rot = cmds.getAttr(f"{jpath}.rotate")[0]
    tr = cmds.getAttr(f"{jpath}.translate")[0]
    dR = max(abs(rot[i] - p0[0][i]) for i in range(3))
    dT = max(abs(tr[i] - p0[1][i]) for i in range(3))
    ok = dR > 0.05 or (rb.physics_mode.value == 1 and dT > 0.005)
    assert_true(
        ok,
        f"dynamic joint {jpath} (body {rb_idx}, mode {rb.physics_mode.value}) "
        f"local pose did not change (dR={dR:.3f}, dT={dT:.4f}) — write-back broken",
    )
    print(
        f"PASS: write-back drives joint {jpath} (body {rb_idx}) dR={dR:.3f} dT={dT:.4f}"
    )
    return True


# ---------------------------------------------------------------------------
# Test registry and runner
# ---------------------------------------------------------------------------

_TESTS = [
    ("Root Creation", test_pmx_root_creation),
    ("Geometry Group Creation", test_pmx_geometry_group_creation),
    ("Mesh Creation", test_pmx_mesh_creation),
    ("Scene Hierarchy", test_pmx_hierarchy),
    ("Material Creation", test_pmx_material_creation),
    ("Skin Cluster Creation", test_pmx_skin_cluster_creation),
    ("Skin Weights Applied", test_pmx_skin_weights_applied),
    ("Rigid Body Node Creation", test_pmx_rigid_body_node_creation),
    ("Rigid Body Wiring", test_pmx_physics_wiring),
    ("Simulation Steps", test_pmx_simulation_steps),
    ("Write-back Moves Bone", test_pmx_write_back_moves_bone),
]
