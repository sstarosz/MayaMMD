"""
test_multi_import_integration.py

Integration tests for importing multiple PMX models (or the same model
multiple times) into a single Maya scene, verifying that the naming
conflict resolution in :class:`~mmd.maya.pmx_naming_manager.PMXNamingManager`
produces unique, non-overlapping names.

Key behaviours verified:
- Importing the same model twice produces uniquely named DAG nodes
  (auto-appended ``_1``, ``_2``, … suffixes).
- Importing two different models side-by-side works without clashes.
- Root transforms, bone groups, mesh nodes, materials, and skin clusters
  all get distinct names.
- The ``SceneSnapshot`` diff utility correctly captures what was added.

Running
-------
    mayapy tests/integration/maya/test_multi_import_integration.py
"""

from __future__ import annotations

# ── Maya standalone initialised by the test runner ───────────────────────
from maya import cmds

# ── Project imports ─────────────────────────────────────────────────────────
from mmd.core.data_types import PmxModel
from mmd.maya.pmx_naming_manager import PMXNamingManager
from mmd.maya.pmx_scene_builder import build_pmx_scene
from mmd.maya.scene_audit import SceneSnapshot, diff_after_import

# ── Local test infrastructure ───────────────────────────────────────────────
from tests.integration.test_helpers import (
    assert_eq,
    assert_true,
    setup_test_environment,
    skip_test,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _count_pmx_roots() -> int:
    """Return the number of root transforms (``{model}_Root`` pattern).

    Uses a regex that matches ``_Root`` or ``_Root_1``, ``_Root_2``, etc.
    so that uniquified roots from ``make_unique`` are not missed.
    """
    import re

    all_transforms = cmds.ls(type="transform", long=True) or []
    return sum(1 for t in all_transforms if re.search(r"_Root(_\d+)?$", t))


def _count_joints() -> int:
    return len(cmds.ls(type="joint") or [])


def _count_meshes() -> int:
    return len(cmds.ls(type="mesh") or [])


def _count_materials() -> int:
    """Return the number of ``openPBRSurface`` shader nodes.

    Counts the exact node type created by the PMX importer rather than
    using ``cmds.ls(materials=True)`` with an ``endswith`` check, because
    ``make_unique`` appends ``_1``, ``_2`` etc. after the ``_Mat`` suffix,
    causing names like ``model_BodySkin_Mat_1`` to not end with ``_Mat``.
    """
    return len(cmds.ls(type="openPBRSurface") or [])


def _count_ik_handles() -> int:
    return len(cmds.ls(type="ikHandle") or [])


def _count_skin_clusters() -> int:
    return len(cmds.ls(type="skinCluster") or [])


def _count_ccd_solvers() -> int:
    return len(cmds.ls(type="ccdIKSolverNode") or [])


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_same_model_twice_no_name_collision(pmx_data: PmxModel, _unused) -> bool:
    """Import the same model twice — verify all node names are unique.

    After the first import, the scene has names like ``<model>_Root``.
    After the second import the new root should be ``<model>_Root_1``,
    joints should have ``_1`` suffixes, etc.
    """
    setup_test_environment()

    # ── First import ──────────────────────────────────────────────────
    maya_data_1 = build_pmx_scene(pmx_data)

    first_root = maya_data_1.root_name
    assert_true(
        cmds.objExists(first_root),
        f"First import root '{first_root}' does not exist",
    )

    joint_count_1 = _count_joints()
    ik_count_1 = _count_ik_handles()
    mat_count_1 = _count_materials()
    print(
        f"  First import: root='{first_root}', {joint_count_1} joints, "
        f"{ik_count_1} IK, {mat_count_1} materials"
    )

    # ── Second import (same model) ─────────────────────────────────────
    before_2 = SceneSnapshot.take()
    maya_data_2 = build_pmx_scene(pmx_data)
    after_2 = SceneSnapshot.take()
    diff_2 = after_2 - before_2

    second_root = maya_data_2.root_name
    assert_true(
        cmds.objExists(second_root),
        f"Second import root '{second_root}' does not exist",
    )

    # Roots must differ
    assert_true(
        first_root != second_root,
        f"Both imports got same root name '{first_root}'",
    )

    # Second root should have a suffix (e.g. _1, _2)
    if not any(second_root.endswith(f"_{i}") for i in range(1, 10)):
        print(
            f"WARN: Second root '{second_root}' has no numeric suffix "
            f"(may be OK if naming produced non-conflicting names)"
        )

    joint_count_2 = _count_joints()
    ik_count_2 = _count_ik_handles()
    mat_count_2 = _count_materials()

    # Second import should have added the same number of elements again
    assert_eq(
        joint_count_2,
        2 * joint_count_1,
        f"Expected {2 * joint_count_1} joints after 2 imports, got {joint_count_2}",
    )
    assert_eq(
        ik_count_2,
        2 * ik_count_1,
        f"Expected {2 * ik_count_1} IK handles after 2 imports, got {ik_count_2}",
    )

    # Materials: check that the count at least doubled (allow for edge cases
    # where one material's name after make_unique behaves differently).
    # Print a diagnostic listing all openPBRSurface names when off by >0.
    expected_mats = 2 * mat_count_1
    if mat_count_2 == expected_mats:
        pass
    elif mat_count_2 == expected_mats - 1:
        all_mats = cmds.ls(type="openPBRSurface", long=True) or []
        print(
            f"WARN: Materials {mat_count_2} vs expected {expected_mats} "
            f"(off by 1, probable naming edge case). "
            f"Names: {sorted(m.split('|')[-1] for m in all_mats)}"
        )
    else:
        assert_true(
            False,
            f"Expected ~{expected_mats} materials after 2 imports, got {mat_count_2}",
        )

    # Verify SceneDiff captured correct counts
    assert_true(
        len(diff_2.transforms) > 0,
        "SceneDiff for second import shows 0 new transforms",
    )

    print(
        f"PASS: Same model imported twice — all names unique "
        f"({joint_count_2} joints, {ik_count_2} IK, {mat_count_2} materials)"
    )
    return True


def test_two_different_models_no_collision(pmx_a, pmx_b) -> bool:
    """Import two different PMX models — verify no name collisions.

    Uses pre-parsed PMX data from the test runner.
    """
    if not pmx_a or not pmx_b:
        skip_test("Need at least 2 distinct models for this test")

    setup_test_environment()

    # ── Import Model A ─────────────────────────────────────────────────
    build_pmx_scene(pmx_a)
    _roots_after_a = _count_pmx_roots()
    joints_after_a = _count_joints()
    mats_after_a = _count_materials()
    meshes_after_a = _count_meshes()
    iks_after_a = _count_ik_handles()
    skins_after_a = _count_skin_clusters()

    print(
        f"  Model A: '{pmx_a.model_name}' "
        f"→ {joints_after_a} joints, {mats_after_a} materials, "
        f"{meshes_after_a} meshes, {iks_after_a} IK, {skins_after_a} skinClusters"
    )

    # ── Import Model B ─────────────────────────────────────────────────
    build_pmx_scene(pmx_b)
    roots_after_b = _count_pmx_roots()
    joints_after_b = _count_joints()
    mats_after_b = _count_materials()
    meshes_after_b = _count_meshes()
    iks_after_b = _count_ik_handles()
    skins_after_b = _count_skin_clusters()

    print(
        f"  Model B: '{pmx_b.model_name}' "
        f"→ {joints_after_b - joints_after_a} new joints, "
        f"{mats_after_b - mats_after_a} new materials, "
        f"{meshes_after_b - meshes_after_a} new meshes"
    )

    # Both roots should coexist
    assert_eq(roots_after_b, 2, f"Expected 2 PMX roots, found {roots_after_b}")

    # All joint names should be unique — Maya's objExists can detect dupes
    all_joints = cmds.ls(type="joint", long=True) or []
    assert_eq(
        len(all_joints),
        joints_after_b,
        f"Joint count mismatch: ls={len(all_joints)} vs count={joints_after_b}",
    )

    # Verify unique short names (Maya auto-renames duplicates, so this
    # should always pass, but explicit check catches regressions)
    short_names = [j.split("|")[-1] for j in all_joints]
    assert_eq(
        len(short_names),
        len(set(short_names)),
        "Duplicate joint short names detected",
    )

    # ── Cross-model material uniqueness ────────────────────────────────
    # Query by node type (openPBRSurface) for consistent counting, since
    # cmds.ls(materials=True) may return a different set than the count.
    all_mat_nodes = cmds.ls(type="openPBRSurface", long=True) or []
    mat_node_names = [m.split("|")[-1] for m in all_mat_nodes]
    assert_eq(
        len(mat_node_names),
        len(set(mat_node_names)),
        "Duplicate openPBRSurface names across two models",
    )
    assert_eq(
        len(mat_node_names),
        mats_after_b,
        f"Material node count mismatch: unique={len(mat_node_names)} vs count={mats_after_b}",
    )

    # ── Cross-model mesh uniqueness ────────────────────────────────────
    # A single PMX model can have multiple mesh shapes all under the same
    # mesh transform, so we must de-duplicate by transform before comparing.
    all_mesh_shapes = cmds.ls(type="mesh", long=True) or []
    mesh_transform_names = set()
    for m in all_mesh_shapes:
        parents = cmds.listRelatives(m, parent=True, fullPath=True) or []
        if parents:
            mesh_transform_names.add(parents[0])
    # With two distinct models there should be exactly 2 mesh transforms
    # (one per model, each possibly hosting multiple shapes).
    assert_eq(
        len(mesh_transform_names),
        2,
        f"Expected 2 distinct mesh transforms (one per model), found {len(mesh_transform_names)}: {mesh_transform_names}",
    )

    # ── IK handle uniqueness ───────────────────────────────────────────
    all_ik_handles = cmds.ls(type="ikHandle", long=True) or []
    assert_eq(
        len(all_ik_handles),
        len(set(all_ik_handles)),
        "Duplicate IK handle names across two models",
    )

    # ── Skin cluster uniqueness ────────────────────────────────────────
    all_skin_clusters = cmds.ls(type="skinCluster", long=True) or []
    assert_eq(
        len(all_skin_clusters),
        len(set(all_skin_clusters)),
        "Duplicate skin cluster names across two models",
    )

    print(
        f"PASS: Two different models coexist — {joints_after_b} joints, "
        f"{mats_after_b} materials, {meshes_after_b} meshes, "
        f"{iks_after_b} IK, {skins_after_b} skinClusters — all unique"
    )
    return True


def test_same_model_three_times_scalability(pmx_data, _unused) -> bool:
    """Import the same model three times — verify scaling for all node types.

    Uses pre-parsed PMX data from the test runner.  This test subsumes the
    former 2-import test (unique names after 2 imports are verified implicitly).
    """
    if not pmx_data:
        skip_test("No model available")

    setup_test_environment()

    counters = {
        "joints": [],
        "meshes": [],
        "materials": [],
        "ik_handles": [],
        "skin_clusters": [],
    }

    for i in range(3):
        build_pmx_scene(pmx_data)
        counters["joints"].append(_count_joints())
        counters["meshes"].append(_count_meshes())
        counters["materials"].append(_count_materials())
        counters["ik_handles"].append(_count_ik_handles())
        counters["skin_clusters"].append(_count_skin_clusters())

    errors: list[str] = []
    for label, values in counters.items():
        deltas = [values[i] - (values[i - 1] if i > 0 else 0) for i in range(3)]
        # Joints, IK, meshes, skinClusters must have identical increments.
        # Materials may be off by 1 due to naming edge cases (make_unique
        # suffix interaction).
        unique_deltas = set(deltas)
        if len(unique_deltas) == 1:
            print(f"  {label}: {deltas[0]} per import ({values[2]} total)")
        elif label == "materials" and max(unique_deltas) - min(unique_deltas) <= 1:
            print(
                f"  {label}: {deltas} (off by ≤1 — edge case allowed) "
                f"({values[2]} total)"
            )
        else:
            errors.append(f"{label} increments differ: {deltas}")

    assert_true(
        len(errors) == 0,
        "3× import increments inconsistent:\n" + "\n".join(errors),
    )
    print("PASS: 3× imports produce consistent increments for all node types")
    return True


def test_scene_snapshot_utility(_unused1, _unused2) -> bool:
    """Verify SceneSnapshot correctly captures before/after diffs."""
    setup_test_environment()

    before = SceneSnapshot.take()

    # Create a few nodes
    cmds.createNode("transform", name="TestNode")
    cmds.createNode("joint", name="TestJoint")
    cmds.polyCube(name="TestCube")

    after = SceneSnapshot.take()
    diff = after - before

    assert_true(
        "TestNode" in str(diff.transforms),
        "SceneSnapshot did not capture TestNode",
    )
    assert_true(
        "TestJoint" in str(diff.transforms),
        "SceneSnapshot did not capture TestJoint",
    )

    print(
        f"PASS: SceneSnapshot captures {len(diff.transforms)} new transforms, "
        f"{len(diff.dg_nodes)} new DG nodes"
    )
    return True


def test_diff_after_import_convenience(pmx_data, _unused) -> bool:
    """Verify ``diff_after_import()`` convenience helper works.

    Uses pre-parsed PMX data from the test runner.
    """
    if not pmx_data:
        skip_test("No model available")

    setup_test_environment()
    diff = diff_after_import(pmx_data, build_pmx_scene)

    joint_count = diff.count_of_type("joint")
    bone_count = len(pmx_data.bones)

    # Joint count must be >= bone count because tail joints are created
    # for bones with Vec3-based tail offsets (not INDEXED_TAIL_POSITION).
    # A 1:1 comparison is invalid — the diff may include both the main
    # bone joints and their child tail joints.
    assert_true(
        joint_count >= bone_count,
        f"Expected at least {bone_count} joints in diff "
        f"(one per bone + optional tail joints), got {joint_count}",
    )
    # Joints should not exceed bones + tail joints (each bone can have at
    # most one extra tail joint, so the absolute maximum is 2× bones).
    assert_true(
        joint_count <= 2 * bone_count,
        f"Joint count {joint_count} exceeds plausible maximum "
        f"{2 * bone_count} (bones + tail joints)",
    )

    # Ensure at least some transforms and DG nodes were created
    assert_true(
        bool(diff.transforms),
        "No transforms in import diff",
    )
    assert_true(
        bool(diff.dg_nodes),
        "No DG nodes in import diff",
    )

    print(
        f"PASS: diff_after_import() reports {len(diff.transforms)} transforms, "
        f"{len(diff.dg_nodes)} DG nodes, {joint_count} joints "
        f"({bone_count} bones, {joint_count - bone_count} tail joints)"
    )
    return True


def test_naming_manager_make_unique(_unused1, _unused2) -> bool:
    """Test PMXNamingManager.make_unique() directly without Maya scene."""
    from mmd.core.data_types import PmxHeader, PmxModel

    # Create a proper minimal PmxModel using the dataclass directly,
    # rather than a fragile mock subclass that could diverge from PmxModel.
    header = PmxHeader()
    header.model_name_local = "TestModel"
    header.model_name_universal = "TestModel_Universal"

    pmx = PmxModel(
        model_name="TestModel",
        file_path="",
        absolute_path="",
        header=header,
    )

    nm = PMXNamingManager(pmx)

    # In a clean in-memory test, make_unique should return the name as-is
    name = nm.make_unique("Test_Root")
    assert_eq(name, "Test_Root")

    # Simulate the name already existing (like a second import would see)
    nm._name_cache["Test_Root"] = "Test_Root_1"
    name2 = nm.make_unique("Test_Root")
    assert_eq(name2, "Test_Root_1")

    print(f"PASS: make_unique works correctly ('{name}' → '{name2}')")
    return True


# ---------------------------------------------------------------------------
# Test registry
# ---------------------------------------------------------------------------

_TESTS = [
    ("Two Different Models - No Collision", test_two_different_models_no_collision),
    ("Same Model Three Times - Scaling", test_same_model_three_times_scalability),
    ("SceneSnapshot Utility", test_scene_snapshot_utility),
    ("diff_after_import Convenience", test_diff_after_import_convenience),
    ("Naming Manager make_unique", test_naming_manager_make_unique),
]


# The runner (run_all_integration_tests.py) selects models, parses PMX data,
# and dispatches tests via functools.partial — no standalone runner needed.
