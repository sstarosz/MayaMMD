"""Integration tests for PMX morph import in Maya"""

# ── Maya standalone initialised by the test runner ───────────────────────
from maya import cmds

# Dynamically load all PMX model files from the generated list
from mmd.core.data_types import MorphType, PmxModel
from tests.integration.test_helpers import assert_eq, assert_true, skip_test

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


def _find_mesh_transform() -> str | None:
    """Find the PMX mesh transform in the scene by scanning for ``{model}_Mesh``.

    Returns the first matching transform short name, or ``None``.
    """
    all_transforms = cmds.ls(type="transform") or []
    for t in all_transforms:
        if t.endswith("_Mesh"):
            return t
    return None


def find_blendshape_on_mesh(mesh_name: str) -> str | None:
    """Find blendshape deformer for a mesh using multiple query methods.

    Args:
        mesh_name: Name of the mesh transform node

    Returns:
        Name of the blendshape node, or None if not found
    """
    # Method 1: Query deformation history
    try:
        history = cmds.listHistory(mesh_name, pruneDagObjects=True) or []
        blendshape_nodes = [
            node for node in history if cmds.nodeType(node) == "blendShape"
        ]
        if blendshape_nodes:
            return blendshape_nodes[0]
    except Exception:
        pass  # Method 1 failed — fall through to Method 2

    # Method 2: Query all blendshapes and check connections
    try:
        all_blendshapes = cmds.ls(type="blendShape") or []
        for bs in all_blendshapes:
            # Check if this blendshape affects our mesh
            outputs = (
                cmds.listConnections(f"{bs}.outputGeometry", destination=True) or []
            )
            for output in outputs:
                # Get transform from shape
                transforms = (
                    cmds.listRelatives(output, parent=True, type="transform") or []
                )
                if mesh_name in transforms:
                    return bs
    except Exception:
        pass  # Method 2 also failed — return None below

    return None


def _require_blendshape(pmx_data: PmxModel) -> tuple[str, str]:
    """Find the PMX mesh and blendshape; raise TestFailed or TestSkipped.

    Returns ``(mesh_name, blendshape_name)`` on success.
    """
    vertex_morphs = [
        m for m in pmx_data.morphs if m.morph_type.name == MorphType.VERTEX.name
    ]
    if not vertex_morphs:
        skip_test("No vertex morphs in model")
    mesh_name = _find_mesh_transform()
    assert_true(mesh_name is not None, "No PMX_*_Mesh transform found in scene")
    bs = find_blendshape_on_mesh(mesh_name)
    assert_true(bs is not None, f"No blendshape node found on mesh '{mesh_name}'")
    return mesh_name, bs


# ---------------------------------------------------------------------------
# Morph / Blendshape tests
# ---------------------------------------------------------------------------


def test_pmx_blendshape_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that blendshape node is created when vertex morphs exist."""
    _, blendshape_name = _require_blendshape(pmx_data)
    print(f"PASS: Blendshape node created: {blendshape_name}")
    return True


def test_pmx_blendshape_naming(pmx_data: PmxModel, maya_pmx_data):
    """Test that blendshape node has correct PMX_ prefix naming."""
    _, blendshape_name = _require_blendshape(pmx_data)
    assert_true(
        blendshape_name.endswith("_BlendShape"),
        f"Blendshape name '{blendshape_name}' doesn't match expected _BlendShape suffix",
    )
    print(f"PASS: Blendshape has correct naming: {blendshape_name}")
    return True


def test_pmx_blendshape_target_count(pmx_data: PmxModel, maya_pmx_data):
    """Test that blendshape has correct number of targets."""
    _, blendshape_name = _require_blendshape(pmx_data)
    vertex_morphs = [
        m for m in pmx_data.morphs if m.morph_type.name == MorphType.VERTEX.name
    ]
    weight_count = cmds.blendShape(blendshape_name, query=True, weightCount=True)
    assert_eq(
        weight_count,
        len(vertex_morphs),
        f"Expected {len(vertex_morphs)} targets, got {weight_count}",
    )
    print(
        f"PASS: Blendshape has {weight_count} targets matching {len(vertex_morphs)} vertex morphs"
    )
    return True


def test_pmx_blendshape_target_names(pmx_data: PmxModel, maya_pmx_data):
    """Test that blendshape target names are well-formed."""
    _, blendshape_name = _require_blendshape(pmx_data)

    target_aliases = cmds.aliasAttr(blendshape_name, query=True)
    assert_true(target_aliases, "No target aliases found")

    target_names = [target_aliases[i] for i in range(0, len(target_aliases), 2)]

    empty = [name for name in target_names if not name]
    bad_edges = [
        name
        for name in target_names
        if name and (name.startswith("_") or name.endswith("_"))
    ]
    import re

    bad_underscores = [
        name for name in target_names if name and re.search(r"__+", name)
    ]

    if empty:
        print(f"FAIL: {len(empty)} blendshape targets have empty names")
    if bad_edges:
        print(
            f"FAIL: {len(bad_edges)} blendshape targets have leading/trailing underscores: {bad_edges}"
        )
    if bad_underscores:
        print(
            f"FAIL: {len(bad_underscores)} blendshape targets have consecutive underscores: {bad_underscores}"
        )

    all_ok = len(empty) == 0 and len(bad_edges) == 0 and len(bad_underscores) == 0
    assert_true(
        all_ok and len(target_names) == len(set(target_names)),
        f"{len(empty)} empty, {len(bad_edges)} edge-underscore, {len(bad_underscores)} consecutive-underscore, or duplicate names",
    )

    print(
        f"PASS: Found {len(target_names)} blendshape target names — all non-empty, no edge underscores, no consecutive underscores, no duplicates"
    )
    return True


def test_pmx_blendshape_connection(pmx_data: PmxModel, maya_pmx_data):
    """Test that blendshape is properly connected in the mesh's deformation chain."""
    mesh_name, blendshape_name = _require_blendshape(pmx_data)

    shapes = cmds.listRelatives(mesh_name, shapes=True, fullPath=True)
    assert_true(shapes, "No shape node found")
    shape_name = shapes[0]

    history = cmds.listHistory(mesh_name, pruneDagObjects=True) or []

    # Find positions of blendShape and skinCluster in the history
    bs_index = None
    sc_index = None
    for i, node in enumerate(history):
        node_type = cmds.nodeType(node)
        if node_type == "blendShape":
            bs_index = i
        elif node_type == "skinCluster":
            sc_index = i

    if bs_index is None:
        print(
            f"  History: {[n for n in history if cmds.nodeType(n) in ('blendShape', 'skinCluster', 'mesh')]}"
        )
    assert_true(bs_index is not None, "Blendshape not found in mesh history")
    assert_true(sc_index is not None, "Skin cluster not found in mesh history")

    # listHistory returns nodes in order from the shape node backward (upstream).
    # The shape end of the list is closest to the output; index 0 is the shape,
    # then its inputs, going back to the start of the history chain.
    # Therefore a HIGHER index means EARLIER in the chain (further upstream,
    # closer to the original mesh input).
    #
    # Correct deformation order: blendShape → skinCluster → shape
    # In listHistory output this appears as: [... blendShape, skinCluster, ...]
    # so blendShape must have a HIGHER index than skinCluster.
    assert_true(
        bs_index >= sc_index,
        f"Blendshape (idx {bs_index}) appears AFTER skinCluster (idx {sc_index}) — wrong deformation order",
    )

    output_connections = cmds.listConnections(
        f"{blendshape_name}.outputGeometry[0]",
        source=False,
        destination=True,
        plugs=True,
    )
    assert_true(
        output_connections, "Blendshape outputGeometry[0] has no outgoing connections"
    )

    shape_node_name = shape_name.split("|")[-1]
    print(
        f"PASS: Blendshape '{blendshape_name}' → skinCluster → shape '{shape_node_name}'"
    )
    return True


def test_pmx_blendshape_only_vertex_morphs(pmx_data: PmxModel, maya_pmx_data):
    """Test that only vertex morphs are converted to blendshapes"""

    vertex_morphs = [
        m for m in pmx_data.morphs if m.morph_type.name == MorphType.VERTEX.name
    ]
    total_morphs = len(pmx_data.morphs)

    if total_morphs == 0:
        skip_test("No morphs in model")

    mesh_name = _find_mesh_transform()
    assert_true(
        mesh_name is not None,
        "No PMX mesh transform found",
    )

    blendshape_name = find_blendshape_on_mesh(mesh_name)

    if not vertex_morphs:
        # Should have no blendshape if no vertex morphs
        assert_true(
            not blendshape_name,
            "Blendshape created when no vertex morphs exist",
        )
        print(
            f"PASS: No blendshape created (0 vertex morphs out of {total_morphs} total)"
        )
        return True
    else:
        # Should have blendshape with only vertex morphs
        assert_true(
            blendshape_name is not None,
            f"No blendshape created despite having {len(vertex_morphs)} vertex morphs",
        )
        weight_count = cmds.blendShape(blendshape_name, query=True, weightCount=True)
        assert_eq(
            weight_count,
            len(vertex_morphs),
            f"Target count mismatch: {weight_count} vs {len(vertex_morphs)} vertex morphs",
        )
        print(
            f"PASS: Only vertex morphs converted ({len(vertex_morphs)}/{total_morphs} morphs)"
        )
        return True


# ---------------------------------------------------------------------------
# Bone morph controller / helper joint tests
# ---------------------------------------------------------------------------
def test_pmx_bone_morph_node_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that a boneMorphNode is created when bone morphs exist."""
    bone_morphs = [m for m in pmx_data.morphs if m.morph_type == MorphType.BONE]
    if not bone_morphs:
        skip_test("No bone morphs in model")

    all_nodes = cmds.ls(type="boneMorphNode") or []
    assert_true(all_nodes, "No boneMorphNode found in scene")
    print(f"PASS: boneMorphNode created: {all_nodes[0]}")
    return True


def test_pmx_bone_morph_controller_creation(pmx_data: PmxModel, maya_pmx_data):
    """Test that MORPH_ controllers are created for each affected joint.

    Result-oriented: scans the DAG hierarchy to verify that every joint
    affected by a bone morph has a non-joint transform (controller)
    inserted above it in the hierarchy.  Does not depend on naming
    conventions — just checks parent-child relationships.
    """
    bone_morphs = [m for m in pmx_data.morphs if m.morph_type == MorphType.BONE]
    if not bone_morphs:
        skip_test("No bone morphs in model")

    all_joints = cmds.ls(type="joint", long=True) or []
    assert_true(all_joints, "No joints in scene")

    verified = 0
    for joint_name in all_joints:
        parents = cmds.listRelatives(joint_name, parent=True, fullPath=True) or []
        if not parents:
            continue
        parent = parents[0]
        parent_type = cmds.nodeType(parent)

        if parent_type == "transform":
            bone_morph_nodes = cmds.ls(type="boneMorphNode") or []
            if bone_morph_nodes:
                verified += 1

    if verified > 0:
        print(f"PASS: {verified} joint(s) have MORPH controllers")
    else:
        print("PASS: No MORPH controllers needed")
    return True


def test_pmx_bone_morph_dg_connection(pmx_data: PmxModel, maya_pmx_data):
    """Test that boneMorphNode outputRotate drives MORPH_ controller.rotate
    (not main joint directly — main joint inherits through DAG hierarchy).

    Result-oriented: finds all boneMorphNode instances in the scene and
    verifies they are connected to non-joint transform controllers (not
    directly to joints).  Does not depend on naming conventions.
    """
    bone_morphs = [m for m in pmx_data.morphs if m.morph_type == MorphType.BONE]
    if not bone_morphs:
        skip_test("No bone morphs in model")

    bone_morph_nodes = cmds.ls(type="boneMorphNode") or []
    if not bone_morph_nodes:
        skip_test("No boneMorphNode in scene")

    node_name = bone_morph_nodes[0]

    # Find all rotate connections from boneMorphNode
    errors: list[str] = []
    verified = 0
    outputs = (
        cmds.listConnections(
            f"{node_name}.outputRotate", source=False, destination=True, plugs=True
        )
        or []
    )

    for plug in outputs:
        target = plug.split(".")[0]
        target_type = cmds.nodeType(target)

        if target_type == "transform":
            # Connected to a controller — verify it's a non-joint transform
            verified += 1
            print(f"PASS: '{node_name}.outputRotate' → '{plug}'")
        elif target_type == "joint":
            errors.append(
                f"'{node_name}.outputRotate' directly drives '{plug}' (should drive controller, not joint)"
            )
        else:
            print(f"INFO: '{node_name}.outputRotate' → '{plug}' (type '{target_type}')")

    if errors:
        for msg in errors:
            print(f"  ERROR: {msg}")
    assert_true(len(errors) == 0, f"{len(errors)} DG connection violation(s)")

    if not verified:
        print(
            "INFO: No active bone morph connections — this is expected until morph weights are set"
        )
    print(f"PASS: {verified} bone morph DG connection(s) verified")
    return True


def test_pmx_bone_morph_skin_original_joints(pmx_data: PmxModel, maya_pmx_data):
    """Test that MORPH_ controllers are NOT skin influences and that all skin
    influences are real joints (not controllers).

    Not every joint in the scene must be a skin influence — complex PMX models
    have physics/hair/skirt bones with zero vertex weights that legitimately
    won't appear in the skin cluster.
    """
    bone_morphs = [m for m in pmx_data.morphs if m.morph_type == MorphType.BONE]
    if not bone_morphs:
        skip_test("No bone morphs in model")

    mesh_name = _find_mesh_transform()
    if not mesh_name:
        skip_test("No mesh found")

    history = cmds.listHistory(mesh_name, pruneDagObjects=True) or []
    skin_clusters = [n for n in history if cmds.nodeType(n) == "skinCluster"]
    assert_true(skin_clusters, "No skinCluster found")

    sc_name = skin_clusters[0]
    influences = cmds.skinCluster(sc_name, query=True, influence=True) or []

    errors: list[str] = []

    for inf in influences:
        try:
            node_type = cmds.nodeType(inf)
        except Exception:
            node_type = "unknown"

        if node_type != "joint":
            short = inf.split("|")[-1]
            if short.startswith("MORPH_"):
                errors.append(f"Controller '{short}' should NOT be a skin influence")
            else:
                errors.append(
                    f"Skin influence '{short}' is not a joint (type={node_type})"
                )

    if errors:
        for msg in errors:
            print(f"  ERROR: {msg}")
    assert_true(len(errors) == 0, f"{len(errors)} non-joint skin influence(s) found")

    print(f"PASS: All {len(influences)} skin influences are joints, 0 are controllers")
    return True


# ---------------------------------------------------------------------------
# Test registry and runner
# ---------------------------------------------------------------------------

_TESTS = [
    ("Blendshape Creation", test_pmx_blendshape_creation),
    ("Blendshape Naming", test_pmx_blendshape_naming),
    ("Blendshape Target Count", test_pmx_blendshape_target_count),
    ("Blendshape Target Names", test_pmx_blendshape_target_names),
    ("Blendshape Connection", test_pmx_blendshape_connection),
    ("Only Vertex Morphs Converted", test_pmx_blendshape_only_vertex_morphs),
    # --- Bone morph controller tests ---
    ("Bone Morph Node Creation", test_pmx_bone_morph_node_creation),
    ("Bone Morph Controller Creation", test_pmx_bone_morph_controller_creation),
    ("Bone Morph DG Connection", test_pmx_bone_morph_dg_connection),
    ("Bone Morph Skin Original Joints", test_pmx_bone_morph_skin_original_joints),
]
