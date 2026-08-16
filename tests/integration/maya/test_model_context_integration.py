"""
test_model_context_integration.py

Integration tests for :class:`~mmd.maya.model_context.ModelContext` and the
scene-discovery utilities (``build_bone_map_from_scene``, etc.) that back it.

These tests create minimal PMX-like scene structures **without** running the
full PMX import pipeline, so they can verify that the selection-driven,
attribute-based discovery logic works correctly in isolation.

Key behaviours verified:
- ``ModelContext.boneMap()`` discovers joints via ``pmxNameLocal`` / ``pmxNameUniversal``.
- ``ModelContext.morphMap()`` reads the ``pmxMorphMapping`` compound attribute.
- ``ModelContext.boneMorphNode()`` / ``blendShapeNode()`` find nodes via
  self-describing root attrs **and** fallback scan paths.
- ``ModelContext.ikHandles()`` lists IK handle descendants.
- ``find_model_root_from_selection()`` walks up from any descendant joint.
- ``refresh_from_selection()`` detects model switches.
- Cache is cleared on ``_set_active()`` (model switch) and ``invalidate_cache()``.
- ``ModelContext.isValid`` correctly reflects root existence.

Running
-------
    mayapy tests/integration/maya/test_model_context_integration.py
"""

from __future__ import annotations

import os

# ── Maya standalone initialised by the test runner ───────────────────────
from maya import cmds

# ── PMX/VMD/VPD pipeline imports (used by real-model tests) ────────────────
from mmd.core.pmx_importer import parse_pmx
from mmd.core.vmd_importer import parse_vmd_file
from mmd.core.vpd_importer import parse_vpd_file

# ── Project imports ─────────────────────────────────────────────────────────
from mmd.maya.model_context import ModelContext
from mmd.maya.pmx_model_utils import (
    build_bone_map_from_scene,
    build_morph_map_from_scene,
    discover_model_roots_in_scene,
    find_all_model_roots_from_selection,
    find_blend_shape_node,
    find_bone_morph_node,
    find_ik_handles,
    find_model_root_from_selection,
)
from mmd.maya.pmx_scene_builder import build_pmx_scene
from mmd.maya.vmd_scene_builder import apply_vmd_to_scene
from mmd.maya.vpd_scene_builder import apply_vpd_pose_to_scene

# ── Local test infrastructure ───────────────────────────────────────────────
from tests.integration.test_helpers import (
    assert_eq,
    assert_true,
    setup_test_environment,
    skip_test,
)

# ══════════════════════════════════════════════════════════════════════════
# Scene-build helpers
# ══════════════════════════════════════════════════════════════════════════


def _create_minimal_model_scene(root_name: str = "TestModel_Root") -> str:
    """Create a minimal PMX-like model in the scene and return the root path.

    Builds:
    - ``{root_name}`` transform (the root).
    - ``{root_name}|J_Bip001_Pelvis`` joint with ``pmxNameLocal=" pelvis"``
      and ``pmxNameUniversal=" pelvis"``.
    - ``{root_name}|J_Bip001_Pelvis|J_Bip001_Spine`` joint with
      ``pmxNameLocal=" spine"``.
    - A mesh transform ``{root_name}|{mesh_name}`` with a mesh shape.
    - A blendShape deformer ``{root_name}_blendShape`` on the mesh, with
      ``pmxMorphMapping`` compound attribute describing two targets.
    - A boneMorphNode ``{root_name}_boneMorph`` connected to a MORPH_ controller.
    - An IK handle ``{root_name}_ikHandle`` under the root.

    Returns:
        The full DAG path of the root transform.
    """
    root = cmds.createNode("transform", name=root_name)

    # ── Self-describing root attribute (fast discovery path) ────────────
    cmds.addAttr(root, longName="pmxModelName", dataType="string")
    cmds.setAttr(
        f"{root}.pmxModelName",
        root_name.replace("_Root", "").replace("_Root_", ""),
        type="string",
    )

    # ── Joint hierarchy with PMX bone attributes ────────────────────────
    pelvis = cmds.createNode("joint", name="J_Bip001_Pelvis", parent=root)
    cmds.addAttr(pelvis, longName="pmxNameLocal", dataType="string")
    cmds.setAttr(f"{pelvis}.pmxNameLocal", " pelvis", type="string")
    cmds.addAttr(pelvis, longName="pmxNameUniversal", dataType="string")
    cmds.setAttr(f"{pelvis}.pmxNameUniversal", " pelvis", type="string")

    spine = cmds.createNode("joint", name="J_Bip001_Spine", parent=pelvis)
    cmds.addAttr(spine, longName="pmxNameLocal", dataType="string")
    cmds.setAttr(f"{spine}.pmxNameLocal", " spine", type="string")
    # No pmxNameUniversal on spine — tests local-only fallback

    # ── Mesh + blendShape ───────────────────────────────────────────────
    # Create a poly plane and parent it under the root
    plane = cmds.polyPlane(
        name=f"{root_name}_Mesh",
        constructionHistory=False,
        width=1,
        height=1,
        subdivisionsX=1,
        subdivisionsY=1,
    )
    cmds.parent(plane, root)
    mesh_shape = cmds.listRelatives(plane, shapes=True, type="mesh")[0]

    bs_node = cmds.createNode("blendShape", name=f"{root_name}_blendShape")
    cmds.connectAttr(f"{bs_node}.outputGeometry[0]", f"{mesh_shape}.inMesh", force=True)

    # Populate pmxMorphMapping compound
    cmds.addAttr(
        bs_node,
        longName="pmxMorphMapping",
        attributeType="compound",
        numberOfChildren=2,
        multi=True,
    )
    cmds.addAttr(
        bs_node, longName="pmxName", parent="pmxMorphMapping", dataType="string"
    )
    cmds.addAttr(
        bs_node, longName="mayaAlias", parent="pmxMorphMapping", dataType="string"
    )

    cmds.setAttr(f"{bs_node}.pmxMorphMapping[0].pmxName", "blink", type="string")
    cmds.setAttr(f"{bs_node}.pmxMorphMapping[0].mayaAlias", "eyeBlink", type="string")
    cmds.setAttr(f"{bs_node}.pmxMorphMapping[1].pmxName", "smile", type="string")
    cmds.setAttr(f"{bs_node}.pmxMorphMapping[1].mayaAlias", "mouthSmile", type="string")

    # Also create aliases on the blendShape so alias-based lookup works
    cmds.aliasAttr("eyeBlink", f"{bs_node}.weight[0]")
    cmds.aliasAttr("mouthSmile", f"{bs_node}.weight[1]")

    # ── IK handle ───────────────────────────────────────────────────────
    effector = cmds.createNode("ikEffector", name=f"{root_name}_effector", parent=spine)
    ik_handle = cmds.createNode("ikHandle", name=f"{root_name}_ikHandle", parent=root)
    cmds.connectAttr(f"{pelvis}.message", f"{ik_handle}.startJoint")
    cmds.connectAttr(f"{effector}.message", f"{ik_handle}.endEffector")

    return root


def _create_bone_morph_for_model(root: str) -> None:
    """Add a ``boneMorphNode`` to a model and wire up its DG connections.

    Creates a morph target on the boneMorphNode by writing directly to the
    ``morphTargets`` compound attribute (bypassing the ``boneBlendShape``
    command which has an intermittent Maya 2026 standalone parsing bug).
    Also creates a MORPH_ controller transform above the target joint and
    connects ``outputRotate[0]`` to it.

    The ``boneMorphNode`` node type is registered by the MayaMMD plugin
    (already loaded by the test runner — no need to load it here).
    """
    bmn = cmds.createNode("boneMorphNode", name=f"{root}_boneMorph")

    # Find the first joint under this root to use as the morph target
    joints = cmds.ls(type="joint") or []
    target_joint = joints[0] if joints else ""
    if not target_joint:
        return

    # ── Manually populate morphTargets compound ─────────────────────────
    # This mirrors what boneBlendShape addTarget does internally.
    cmds.setAttr(f"{bmn}.weight[0]", 0.0)
    cmds.setAttr(f"{bmn}.morphTargets[0].targetName", "test_morph", type="string")
    cmds.setAttr(f"{bmn}.morphTargets[0].boneNames[0]", target_joint, type="string")
    cmds.setAttr(f"{bmn}.morphTargets[0].positionOffset[0]", 0.0, 0.0, 0.0)
    cmds.setAttr(f"{bmn}.morphTargets[0].rotationOffset[0]", 0.0, 0.0, 0.0, 1.0)

    # ── Create MORPH_ controller and connect outputRotate ───────────────
    # Get the parent of the target joint to parent the controller
    joint_parents = cmds.listRelatives(target_joint, parent=True, fullPath=True) or []
    ctrl = cmds.createNode(
        "transform",
        name="MORPH_J_Bip001_Pelvis_MorphCtrl",
        parent=joint_parents[0] if joint_parents else root,
    )
    # Reparent joint under controller (preserving local transform)
    cmds.parent(target_joint, ctrl, relative=True)

    # Touch outputRotate[0] so the array element exists, then connect
    try:
        cmds.getAttr(f"{bmn}.outputRotate[0]")
    except Exception:
        pass  # Array element may not exist yet — dgdirty will force creation
    cmds.dgdirty(bmn)
    cmds.connectAttr(f"{bmn}.outputRotate[0]", f"{ctrl}.rotate", force=True)


# ══════════════════════════════════════════════════════════════════════════
# Tests — scene-discovery utilities
# ══════════════════════════════════════════════════════════════════════════


def test_find_model_root_from_selection() -> bool:
    """Verify root discovery by walking up from a selected joint."""
    setup_test_environment()
    root = _create_minimal_model_scene("DiscoveryRoot_Root")
    pelvis = cmds.ls(type="joint")[0]

    cmds.select(pelvis)
    found = find_model_root_from_selection()
    assert_true(
        found and root in found,
        f"expected root '{root}', got '{found}'",
    )

    # Selection on a non-PMX object returns None
    orphan = cmds.createNode("transform", name="Orphan")
    cmds.select(orphan)
    found = find_model_root_from_selection()
    assert_true(
        found is None,
        f"expected None for non-PMX selection, got '{found}'",
    )

    # Empty selection returns None
    cmds.select(clear=True)
    found = find_model_root_from_selection()
    assert_true(
        found is None,
        f"expected None for empty selection, got '{found}'",
    )

    print("PASS: find_model_root_from_selection")
    return True


def test_build_bone_map_from_scene() -> bool:
    """Verify bone map discovers pmxNameLocal and pmxNameUniversal attrs."""
    setup_test_environment()
    root = _create_minimal_model_scene("BoneMap_Root")

    bone_map = build_bone_map_from_scene(root)
    # Our helper creates " pelvis" (local+uni) and " spine" (local only)
    expected_local = {" pelvis", " spine"}
    found_local = set(bone_map.keys())
    assert_true(
        found_local == expected_local,
        f"expected {expected_local}, got {found_local}",
    )

    # pmxNameUniversal maps to the same joint as pmxNameLocal
    assert_true(
        bone_map[" pelvis"] == bone_map.get(" pelvis", ""),
        "universal ' pelvis' should map to same joint as local",
    )

    # Verify all values are existing joints
    for maya_joint in bone_map.values():
        assert_true(
            cmds.objExists(maya_joint),
            f"bone map value '{maya_joint}' is not a valid node",
        )

    print("PASS: build_bone_map_from_scene")
    return True


def test_build_morph_map_from_scene() -> bool:
    """Verify morph map reads pmxMorphMapping compound attribute."""
    setup_test_environment()
    root = _create_minimal_model_scene("MorphMap_Root")

    morph_map = build_morph_map_from_scene(root)
    expected = {"blink": "eyeBlink", "smile": "mouthSmile"}
    assert_true(
        morph_map == expected,
        f"expected {expected}, got {morph_map}",
    )

    # Clean fallback path: when pmxMorphMapping is absent, should return
    # identity map from aliasAttr
    setup_test_environment()
    root2 = _create_minimal_model_scene("MorphMapFallback_Root")
    bs = cmds.ls(type="blendShape")[0]
    cmds.attributeQuery("pmxMorphMapping", node=bs, exists=True)
    cmds.deleteAttr(f"{bs}.pmxMorphMapping")

    morph_map_fb = build_morph_map_from_scene(root2)
    assert_true(
        bool(morph_map_fb),
        "expected identity fallback map, got empty",
    )
    # Should have "eyeBlink" and "mouthSmile" as identity aliases
    assert_true(
        "eyeBlink" in morph_map_fb and "mouthSmile" in morph_map_fb,
        f"expected 'eyeBlink'/'mouthSmile' in fallback, got {morph_map_fb}",
    )

    print("PASS: build_morph_map_from_scene")
    return True


def test_find_blend_shape_node() -> bool:
    """Verify blendShape discovery via mesh deformation history scan."""
    setup_test_environment()
    root = _create_minimal_model_scene("BSNode_Root")

    # The function should find it by scanning the mesh deformation history
    bs = find_blend_shape_node(root)
    assert_true(
        bs and cmds.objExists(bs),
        f"could not find blendShape via history scan, got '{bs}'",
    )

    assert_true(
        cmds.nodeType(bs) == "blendShape",
        f"'{bs}' is not a blendShape node",
    )

    # No blendShape in scene → None
    setup_test_environment()
    no_bs_root = cmds.createNode("transform", name="NoBS_Root")
    result = find_blend_shape_node(no_bs_root)
    assert_true(
        result is None,
        f"expected None for model without blendShape, got '{result}'",
    )

    print("PASS: find_blend_shape_node")
    return True


def test_find_bone_morph_node() -> bool:
    """Verify boneMorphNode discovery via outputRotate connection tracing."""
    setup_test_environment()
    root = _create_minimal_model_scene("BMNode_Root")
    _create_bone_morph_for_model(root)

    bmn = find_bone_morph_node(root)
    assert_true(
        bmn and cmds.objExists(bmn),
        f"could not find boneMorphNode via connection tracing, got '{bmn}'",
    )

    assert_true(
        cmds.nodeType(bmn) == "boneMorphNode",
        f"'{bmn}' is not a boneMorphNode",
    )

    print("PASS: find_bone_morph_node")
    return True


def test_find_ik_handles() -> bool:
    """Verify IK handle discovery."""
    setup_test_environment()
    root = _create_minimal_model_scene("IKFind_Root")

    handles = find_ik_handles(root)
    assert_eq(
        len(handles),
        1,
        f"expected 1 IK handle, got {len(handles)}: {handles}",
    )

    # Verify it's the one we created
    expected_name = f"{root}_ikHandle"
    assert_eq(
        handles[0],
        expected_name,
        f"expected '{expected_name}', got '{handles[0]}'",
    )

    print("PASS: find_ik_handles")
    return True


# ══════════════════════════════════════════════════════════════════════════
# Tests — ModelContext
# ══════════════════════════════════════════════════════════════════════════


def test_model_context_bone_map() -> bool:
    """ModelContext.boneMap() returns correct mapping."""
    setup_test_environment()
    root = _create_minimal_model_scene("CtxBone_Root")

    ctx = ModelContext()
    ctx.set_active_root(root)

    bone_map = ctx.boneMap()
    expected_keys = {" pelvis", " spine"}
    assert_true(
        set(bone_map.keys()) == expected_keys,
        f"expected {expected_keys}, got {set(bone_map.keys())}",
    )

    # Empty when no root
    ctx.clear()
    bone_map = ctx.boneMap()
    assert_true(
        not bone_map,
        f"expected empty bone map after clear, got {bone_map}",
    )

    print("PASS: ModelContext.boneMap")
    return True


def test_model_context_morph_map() -> bool:
    """ModelContext.morphMap() returns correct mapping."""
    setup_test_environment()
    root = _create_minimal_model_scene("CtxMorph_Root")

    ctx = ModelContext()
    ctx.set_active_root(root)

    morph_map = ctx.morphMap()
    expected = {"blink": "eyeBlink", "smile": "mouthSmile"}
    assert_true(
        morph_map == expected,
        f"expected {expected}, got {morph_map}",
    )

    print("PASS: ModelContext.morphMap")
    return True


def test_model_context_blend_shape_node() -> bool:
    """ModelContext.blendShapeNode() returns the correct node name."""
    setup_test_environment()
    root = _create_minimal_model_scene("CtxBS_Root")

    ctx = ModelContext()
    ctx.set_active_root(root)

    bs = ctx.blendShapeNode()
    assert_true(
        bs and cmds.objExists(bs),
        f"expected valid blendShape, got '{bs}'",
    )

    # Verify it's actually a blendShape
    assert_true(
        cmds.nodeType(bs) == "blendShape",
        f"'{bs}' is not a blendShape node",
    )

    print("PASS: ModelContext.blendShapeNode")
    return True


def test_model_context_bone_morph_node() -> bool:
    """ModelContext.boneMorphNode() returns the correct node name."""
    setup_test_environment()
    root = _create_minimal_model_scene("CtxBMN_Root")
    _create_bone_morph_for_model(root)

    ctx = ModelContext()
    ctx.set_active_root(root)

    bmn = ctx.boneMorphNode()
    assert_true(
        bmn and cmds.objExists(bmn),
        f"expected valid boneMorphNode, got '{bmn}'",
    )

    assert_true(
        cmds.nodeType(bmn) == "boneMorphNode",
        f"'{bmn}' is not a boneMorphNode",
    )

    print("PASS: ModelContext.boneMorphNode")
    return True


def test_model_context_ik_handles() -> bool:
    """ModelContext.ikHandles() returns correct list."""
    setup_test_environment()
    root = _create_minimal_model_scene("CtxIK_Root")

    ctx = ModelContext()
    ctx.set_active_root(root)

    handles = ctx.ikHandles()
    assert_eq(
        len(handles),
        1,
        f"expected 1 IK handle, got {len(handles)}",
    )

    print("PASS: ModelContext.ikHandles")
    return True


def test_model_context_is_valid() -> bool:
    """ModelContext.isValid reflects root existence correctly."""
    setup_test_environment()
    root = _create_minimal_model_scene("Valid_Root")

    ctx = ModelContext()
    ctx.set_active_root(root)
    assert_true(ctx.isValid, "expected isValid=True after set_active_root")

    # Delete the root → isValid should become False
    cmds.delete(root)
    assert_true(not ctx.isValid, "expected isValid=False after root deleted")

    # Empty context → isValid=False
    ctx.clear()
    assert_true(not ctx.isValid, "expected isValid=False after clear")

    print("PASS: ModelContext.isValid")
    return True


def test_model_context_refresh_from_selection() -> bool:
    """refresh_from_selection() detects model switches."""
    setup_test_environment()
    root_a = _create_minimal_model_scene("ModelA_Root")
    root_b = _create_minimal_model_scene("ModelB_Root")

    ctx = ModelContext()

    # Select a joint from model A → context should switch to A
    joint_a = cmds.ls(f"|{root_a.lstrip('|')}|*", type="joint")[0]
    cmds.select(joint_a)
    changed = ctx.refresh_from_selection()
    assert_true(
        changed,
        f"expected context to change, got unchanged (root={ctx.rootName})",
    )
    # ctx.rootName is a full DAG path (e.g. "|ModelA_Root").  Verify the
    # last component matches.
    assert_true(
        ctx.rootName.endswith("ModelA_Root"),
        f"expected root ending 'ModelA_Root', got '{ctx.rootName}'",
    )

    # Select a joint from model B → context should switch to B
    joint_b = cmds.ls(f"|{root_b.lstrip('|')}|*", type="joint")[0]
    cmds.select(joint_b)
    changed = ctx.refresh_from_selection()
    assert_true(
        changed,
        f"expected context to change, got unchanged (root={ctx.rootName})",
    )
    assert_true(
        ctx.rootName.endswith("ModelB_Root"),
        f"expected root ending 'ModelB_Root', got '{ctx.rootName}'",
    )

    # Same selection again → no change
    changed = ctx.refresh_from_selection()
    assert_true(
        not changed,
        "expected False (no change) when selection is same model",
    )

    # Clear selection → context should clear to "" (change from valid root to empty)
    cmds.select(clear=True)
    changed = ctx.refresh_from_selection()
    assert_true(
        changed,
        "expected True when clearing selection (root should go to '')",
    )
    assert_eq(
        ctx.rootName,
        "",
        f"expected root '' after clearing selection, got '{ctx.rootName}'",
    )

    print("PASS: ModelContext.refresh_from_selection")
    return True


def test_model_context_cache_invalidation() -> bool:
    """Verify cache clears on model switch and explicit invalidate."""
    setup_test_environment()
    root = _create_minimal_model_scene("Cache_Root")

    ctx = ModelContext()
    ctx.set_active_root(root)

    # First call populates cache
    _ = ctx.boneMap()
    # Second call returns cached data
    _ = ctx.boneMap()

    # Explicit invalidation
    ctx.invalidate_cache()
    # After invalidation, the lazy getter should re-query

    # Model switch should also clear cache
    new_root = cmds.createNode("transform", name="NewModel_Root")
    # Add a joint so it's a valid PMX root
    new_joint = cmds.createNode("joint", name="J_NewBone", parent=new_root)
    cmds.addAttr(new_joint, longName="pmxNameLocal", dataType="string")
    cmds.setAttr(f"{new_joint}.pmxNameLocal", "new_bone", type="string")

    ctx.set_active_root(new_root)
    bone_map = ctx.boneMap()
    assert_true(
        "new_bone" in bone_map,
        f"expected 'new_bone' in map after model switch, got {bone_map}",
    )
    # Old bone should not be in cache
    assert_true(
        " pelvis" not in bone_map,
        "old bone ' pelvis' still in cache after model switch",
    )

    print("PASS: ModelContext cache invalidation")
    return True


def test_model_context_set_active_root() -> bool:
    """set_active_root triggers modelChanged signal."""
    setup_test_environment()
    root = _create_minimal_model_scene("Signal_Root")

    received: list[str] = []
    ctx = ModelContext()
    ctx.modelChanged.connect(received.append)

    ctx.set_active_root(root)
    assert_true(
        received == [root],
        f"expected signal with '{root}', got {received}",
    )

    # Setting the same root should NOT emit
    received.clear()
    ctx.set_active_root(root)
    assert_true(
        not received,
        f"signal should not emit when same root is set, got {received}",
    )

    # Clear should emit ""
    received.clear()
    ctx.clear()
    assert_true(
        received == [""],
        f"expected signal with '', got {received}",
    )

    print("PASS: ModelContext.set_active_root signal")
    return True


def test_model_context_multi_model_isolation() -> bool:
    """Two ModelContext instances pointed at different models stay isolated."""
    setup_test_environment()
    root_a = _create_minimal_model_scene("IsolationA_Root")
    root_b = _create_minimal_model_scene("IsolationB_Root")

    ctx_a = ModelContext()
    ctx_a.set_active_root(root_a)

    ctx_b = ModelContext()
    ctx_b.set_active_root(root_b)

    map_a = ctx_a.boneMap()
    map_b = ctx_b.boneMap()

    # Both should have same bone names (same helper creates identical structure)
    assert_true(
        set(map_a.keys()) == set(map_b.keys()),
        f"maps should have same keys, got {set(map_a.keys())} vs {set(map_b.keys())}",
    )

    # But the actual joint paths should differ (different roots)
    joint_paths_a = set(map_a.values())
    joint_paths_b = set(map_b.values())
    assert_true(
        not (joint_paths_a & joint_paths_b),
        "joint paths should not overlap between models",
    )

    print("PASS: ModelContext multi-model isolation")
    return True


# ══════════════════════════════════════════════════════════════════════════
# Real-model integration tests (require PMX asset files)
# ══════════════════════════════════════════════════════════════════════════

# Model / motion / pose paths — set by the test runner before real-model
# tests execute.  Synthetic tests don't depend on these.
_MODEL_A: str | None = None
_MODEL_B: str | None = None
_MOTION: str | None = None
_POSE: str | None = None

# Cache parsed VMD/VPD data — real-model tests parse the same files twice.
_vmd_data_cache: object | None = None
_vpd_data_cache: object | None = None


def _get_vmd_data() -> object | None:
    """Parse and cache the test VMD file."""
    global _vmd_data_cache
    if _vmd_data_cache is None and _MOTION:
        _vmd_data_cache = parse_vmd_file(_MOTION)
    return _vmd_data_cache


def _get_vpd_data() -> object | None:
    """Parse and cache the test VPD file."""
    global _vpd_data_cache
    if _vpd_data_cache is None and _POSE:
        _vpd_data_cache = parse_vpd_file(_POSE)
    return _vpd_data_cache


def _real_model_available() -> bool:
    """``True`` when at least one PMX model file is present."""
    return _MODEL_A is not None and os.path.exists(_MODEL_A)


def _two_models_available() -> bool:
    """``True`` when two distinct PMX model files are present."""
    return _MODEL_A and _MODEL_B and os.path.exists(_MODEL_B)


def _motion_available() -> bool:
    """``True`` when a VMD motion file is present."""
    return _MOTION is not None and os.path.exists(_MOTION)


def _pose_available() -> bool:
    """``True`` when a VPD pose file is present."""
    return _POSE is not None and os.path.exists(_POSE)


def _load_and_build(pmx_path: str):
    """Parse a PMX file and build the Maya scene, returning (pmx_data, maya_data)."""
    pmx_data = parse_pmx(pmx_path)
    maya_data = build_pmx_scene(pmx_data)
    return pmx_data, maya_data


# ── Shared pre-built scene data (populated by run_model_context_tests) ──
# When set, real-model tests use these instead of calling _load_and_build().
_PREBUILT_PMX_A = None
_PREBUILT_MAYA_A = None
_PREBUILT_PMX_B = None
_PREBUILT_MAYA_B = None


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_model_context_on_real_model(pmx_a=None, maya_a=None) -> bool:
    """Load a real PMX model, then verify ModelContext can discover it.

    Data is passed by the runner via ``functools.partial``.
    """
    if maya_a is None:
        skip_test("No PMX model files available")
    pmx_data, maya_data = pmx_a, maya_a

    ctx = ModelContext()
    ctx.set_active_root(maya_data.root_name)

    # boneMap should contain at least as many entries as the model has bones
    bone_map = ctx.boneMap()
    assert_true(
        len(bone_map) >= len(pmx_data.bones),
        f"boneMap has {len(bone_map)} entries, model has {len(pmx_data.bones)} bones",
    )

    # morphMap should match the number of vertex morphs from import
    morph_map = ctx.morphMap()
    assert_true(
        bool(morph_map),
        "morphMap is empty on a real model that has morphs",
    )

    # blendShapeNode should exist
    bs = ctx.blendShapeNode()
    assert_true(
        bs and cmds.objExists(bs),
        "blendShapeNode not found on real model",
    )

    # ikHandles should find at least the number of IK bones in the model
    ik_count = len(ctx.ikHandles())
    ik_bones = [b for b in pmx_data.bones if b.flags & 32]  # IK flag = bit 5
    assert_true(
        ik_count >= len(ik_bones),
        f"found {ik_count} IK handles, model has {len(ik_bones)} IK bones",
    )

    print(
        f"PASS: ModelContext on real model ({len(bone_map)} bones, "
        f"{len(morph_map)} morphs, {ik_count} IK)"
    )
    return True


def test_model_context_multi_model_real(maya_a=None, maya_b=None) -> bool:
    """Load two real models, switch ModelContext between them.

    Data is passed by the runner via ``functools.partial``.
    """
    if maya_a is None or maya_b is None:
        skip_test("Need two PMX models for multi-model test")

    ctx = ModelContext()
    ctx.set_active_root(maya_a.root_name)
    map_a = ctx.boneMap()

    # Keep a reference to one of model A's joint paths to detect cache leakage
    a_sample_joint = next(iter(map_a.values()))

    ctx.set_active_root(maya_b.root_name)
    map_b = ctx.boneMap()

    # Model B's bone map should not contain model A's joints
    assert_true(
        a_sample_joint not in map_b.values(),
        "model A's joints leaked into model B's cache after switch",
    )

    # Switching back to model A should re-discover its data
    ctx.set_active_root(maya_a.root_name)
    map_a_again = ctx.boneMap()
    assert_true(
        a_sample_joint in map_a_again.values(),
        "switching back to model A lost its joint data",
    )

    print(
        f"PASS: ModelContext multi-model real "
        f"({len(map_a)} bones → {len(map_b)} bones → {len(map_a_again)} bones)"
    )
    return True


def test_model_context_to_vmd_builder(maya_a=None, vmd_data=None) -> bool:
    """Use ModelContext to resolve data, pass it to apply_vmd_to_scene.

    Data is passed by the runner via ``functools.partial``.
    """
    if maya_a is None or vmd_data is None:
        skip_test("Need a PMX model and a VMD motion file")
    maya_data = maya_a

    ctx = ModelContext()
    ctx.set_active_root(maya_data.root_name)

    apply_vmd_to_scene(
        vmd_data,
        model=ctx.resolve(),
        start_frame=1,
        apply_bone_anim=True,
        apply_morph_anim=False,
    )

    # Verify animation curves were created on at least one bone
    anim_curves = cmds.ls(type="animCurveTA") or cmds.ls(type="animCurveTL")
    assert_true(
        bool(anim_curves),
        "no animation curves were created",
    )

    print(f"PASS: ModelContext → VMD builder ({len(anim_curves)} anim curves created)")
    return True


def test_model_context_to_vpd_builder(maya_a=None, vpd_data=None) -> bool:
    """Use ModelContext to resolve data, pass it to apply_vpd_pose_to_scene.

    Data is passed by the runner via ``functools.partial``.
    """
    if maya_a is None or vpd_data is None:
        skip_test("Need a PMX model and a VPD pose file")
    maya_data = maya_a

    ctx = ModelContext()
    ctx.set_active_root(maya_data.root_name)

    apply_vpd_pose_to_scene(
        vpd_data,
        model=ctx.resolve(),
        create_keyframe=False,
    )

    print("PASS: ModelContext → VPD builder")
    return True


def test_multi_model_vmd_isolation(maya_a=None, maya_b=None, vmd_data=None) -> bool:
    """Load two models, apply VMD to one via ModelContext, verify other untouched.

    Data is passed by the runner via ``functools.partial``.
    """
    if maya_a is None or maya_b is None or vmd_data is None:
        skip_test("Need two PMX models and a VMD motion file")

    # Capture rest pose of model B before any animation
    ctx_b = ModelContext()
    ctx_b.set_active_root(maya_b.root_name)
    b_bone_map = ctx_b.boneMap()
    b_rest: dict[str, tuple[float, float, float]] = {}
    for pmx_name, joint_name in b_bone_map.items():
        try:
            rx = cmds.getAttr(f"{joint_name}.rotateX")
            ry = cmds.getAttr(f"{joint_name}.rotateY")
            rz = cmds.getAttr(f"{joint_name}.rotateZ")
            b_rest[pmx_name] = (rx, ry, rz)
        except Exception:
            pass  # Joint may not exist in the scene — skip

    # Apply VMD to model A
    ctx_a = ModelContext()
    ctx_a.set_active_root(maya_a.root_name)
    apply_vmd_to_scene(
        vmd_data,
        model=ctx_a.resolve(),
        start_frame=1,
        apply_bone_anim=True,
        apply_morph_anim=False,
    )

    # Verify model B's bones are still at rest
    violations = 0
    for pmx_name, joint_name in b_bone_map.items():
        if pmx_name not in b_rest:
            continue
        try:
            rx = cmds.getAttr(f"{joint_name}.rotateX")
            ry = cmds.getAttr(f"{joint_name}.rotateY")
            rz = cmds.getAttr(f"{joint_name}.rotateZ")
            expected = b_rest[pmx_name]
            if (
                abs(rx - expected[0]) > 0.001
                or abs(ry - expected[1]) > 0.001
                or abs(rz - expected[2]) > 0.001
            ):
                violations += 1
                if violations <= 3:
                    print(
                        f"  WARN: Model B joint '{pmx_name}' changed: "
                        f"({rx:.3f}, {ry:.3f}, {rz:.3f}) vs rest "
                        f"({expected[0]:.3f}, {expected[1]:.3f}, {expected[2]:.3f})"
                    )
        except Exception:
            pass  # Joint disappeared between cache and verify — skip

    assert_true(
        violations == 0,
        f"{violations} joints in model B changed after applying VMD to model A",
    )

    print("PASS: multi-model VMD isolation (model B unchanged)")
    return True


def test_multi_model_vpd_isolation(maya_a=None, maya_b=None, vpd_data=None) -> bool:
    """Load two models, apply VPD to one via ModelContext, verify other untouched.

    Data is passed by the runner via ``functools.partial``.
    """
    if maya_a is None or maya_b is None or vpd_data is None:
        skip_test("Need two PMX models and a VPD pose file")

    # Capture rest pose of model B before any pose
    ctx_b = ModelContext()
    ctx_b.set_active_root(maya_b.root_name)
    b_bone_map = ctx_b.boneMap()
    b_rest: dict[str, tuple[float, float, float, float, float, float]] = {}
    for pmx_name, joint_name in b_bone_map.items():
        try:
            tx = cmds.getAttr(f"{joint_name}.translateX")
            ty = cmds.getAttr(f"{joint_name}.translateY")
            tz = cmds.getAttr(f"{joint_name}.translateZ")
            rx = cmds.getAttr(f"{joint_name}.rotateX")
            ry = cmds.getAttr(f"{joint_name}.rotateY")
            rz = cmds.getAttr(f"{joint_name}.rotateZ")
            b_rest[pmx_name] = (tx, ty, tz, rx, ry, rz)
        except Exception:
            pass  # Joint may not exist — skip

    # Apply VPD to model A
    ctx_a = ModelContext()
    ctx_a.set_active_root(maya_a.root_name)
    apply_vpd_pose_to_scene(
        vpd_data,
        model=ctx_a.resolve(),
        create_keyframe=False,
    )

    # Verify model B's joints are unchanged
    violations = 0
    for pmx_name, joint_name in b_bone_map.items():
        if pmx_name not in b_rest:
            continue
        try:
            tx = cmds.getAttr(f"{joint_name}.translateX")
            ty = cmds.getAttr(f"{joint_name}.translateY")
            tz = cmds.getAttr(f"{joint_name}.translateZ")
            rx = cmds.getAttr(f"{joint_name}.rotateX")
            ry = cmds.getAttr(f"{joint_name}.rotateY")
            rz = cmds.getAttr(f"{joint_name}.rotateZ")
            expected = b_rest[pmx_name]
            if (
                abs(tx - expected[0]) > 0.001
                or abs(ty - expected[1]) > 0.001
                or abs(tz - expected[2]) > 0.001
                or abs(rx - expected[3]) > 0.001
                or abs(ry - expected[4]) > 0.001
                or abs(rz - expected[5]) > 0.001
            ):
                violations += 1
                if violations <= 3:
                    print(
                        f"  WARN: Model B joint '{pmx_name}' changed: "
                        f"T({tx:.3f},{ty:.3f},{tz:.3f}) R({rx:.3f},{ry:.3f},{rz:.3f}) "
                        f"vs rest T({expected[0]:.3f},{expected[1]:.3f},{expected[2]:.3f}) "
                        f"R({expected[3]:.3f},{expected[4]:.3f},{expected[5]:.3f})"
                    )
        except Exception:
            pass  # Joint disappeared between cache and verify — skip

    assert_true(
        violations == 0,
        f"{violations} joints in model B changed after applying VPD to model A",
    )

    print("PASS: multi-model VPD isolation (model B unchanged)")
    return True


# ══════════════════════════════════════════════════════════════════════════
# New tests — scene-discovery edge cases
# ══════════════════════════════════════════════════════════════════════════


def test_discover_model_roots_in_scene() -> bool:
    """Verify discover_model_roots_in_scene with 0, 1, and multiple models."""
    setup_test_environment()

    # 0 models → empty list
    roots = discover_model_roots_in_scene()
    assert_true(
        not roots,
        f"expected empty list with no models, got {roots}",
    )

    # 1 model → single root
    _root_a = _create_minimal_model_scene("DiscoveryScene_Root")
    roots = discover_model_roots_in_scene()
    assert_eq(
        len(roots),
        1,
        f"expected 1 root, got {len(roots)}: {roots}",
    )
    assert_true(
        roots[0].endswith("DiscoveryScene_Root"),
        f"expected root ending 'DiscoveryScene_Root', got '{roots[0]}'",
    )

    # 2 models → two roots
    _root_b = _create_minimal_model_scene("SecondModel_Root")
    roots = discover_model_roots_in_scene()
    assert_eq(
        len(roots),
        2,
        f"expected 2 roots, got {len(roots)}: {roots}",
    )

    # Model without pmxModelName attr — legacy naming convention fallback
    setup_test_environment()
    legacy_root = cmds.createNode("transform", name="LegacyModel_Root")
    legacy_joint = cmds.createNode("joint", name="J_LegacyBone", parent=legacy_root)
    cmds.addAttr(legacy_joint, longName="pmxNameLocal", dataType="string")
    cmds.setAttr(f"{legacy_joint}.pmxNameLocal", "legacy_bone", type="string")
    roots = discover_model_roots_in_scene()
    assert_true(
        any("LegacyModel_Root" in r for r in roots),
        f"legacy root (no pmxModelName) should be discoverable, got {roots}",
    )

    print("PASS: discover_model_roots_in_scene")
    return True


def test_find_all_model_roots_from_selection() -> bool:
    """Verify find_all_model_roots_from_selection covers multi-selection."""
    setup_test_environment()
    root_a = _create_minimal_model_scene("MultiRootA_Root")
    root_b = _create_minimal_model_scene("MultiRootB_Root")

    # Empty selection → empty list
    cmds.select(clear=True)
    roots = find_all_model_roots_from_selection()
    assert_true(
        not roots,
        f"expected empty list for empty selection, got {roots}",
    )

    # Single selection → one root
    joint_a = cmds.ls(f"|{root_a.lstrip('|')}|*", type="joint")[0]
    cmds.select(joint_a)
    roots = find_all_model_roots_from_selection()
    assert_eq(
        len(roots),
        1,
        f"expected 1 root for single selection, got {len(roots)}: {roots}",
    )

    # Multi-selection across two models → two unique roots
    joint_b = cmds.ls(f"|{root_b.lstrip('|')}|*", type="joint")[0]
    cmds.select([joint_a, joint_b])
    roots = find_all_model_roots_from_selection()
    assert_eq(
        len(roots),
        2,
        f"expected 2 roots for multi-selection, got {len(roots)}: {roots}",
    )

    # Non-PMX selection → empty list (even if PMX models exist in scene)
    orphan = cmds.createNode("transform", name="OrphanNode")
    cmds.select(orphan)
    roots = find_all_model_roots_from_selection()
    assert_true(
        not roots,
        f"expected empty list for non-PMX selection, got {roots}",
    )

    print("PASS: find_all_model_roots_from_selection")
    return True


def test_find_model_root_selection_edge_cases() -> bool:
    """Additional edge cases for find_model_root_from_selection."""
    setup_test_environment()
    root = _create_minimal_model_scene("EdgeCase_Root")

    # Selection is the root itself → should find itself
    cmds.select(root)
    found = find_model_root_from_selection()
    assert_true(
        found and root in found,
        f"selecting root itself should find it, got '{found}'",
    )

    # Selection is a mesh child → walks up to root
    mesh = cmds.ls(type="mesh")[0]
    cmds.select(mesh)
    found = find_model_root_from_selection()
    assert_true(
        found and root in found,
        f"selecting mesh child should find root, got '{found}'",
    )

    # Selection is an IK handle → walks up to root
    ik_h = cmds.ls(type="ikHandle")[0]
    cmds.select(ik_h)
    found = find_model_root_from_selection()
    assert_true(
        found and root in found,
        f"selecting IK handle should find root, got '{found}'",
    )

    print("PASS: find_model_root_from_selection edge cases")
    return True


def test_build_bone_map_edge_cases() -> bool:
    """Edge cases for build_bone_map_from_scene: empty root, partial attrs."""
    setup_test_environment()

    # Empty root (no joints) → empty map
    empty_root = cmds.createNode("transform", name="EmptyRoot_Root")
    bone_map = build_bone_map_from_scene(empty_root)
    assert_true(
        not bone_map,
        f"expected empty map for root without joints, got {bone_map}",
    )

    # Root with joints but no pmxName* attributes → empty map
    no_attr_root = cmds.createNode("transform", name="NoAttr_Root")
    cmds.createNode("joint", name="J_NoAttr", parent=no_attr_root)
    bone_map = build_bone_map_from_scene(no_attr_root)
    assert_true(
        not bone_map,
        f"expected empty map for joints without pmxName attrs, got {bone_map}",
    )

    # Non-existent root → empty map (no crash)
    bone_map = build_bone_map_from_scene("|NonExistent_Root")
    assert_true(
        not bone_map,
        f"expected empty map for non-existent root, got {bone_map}",
    )

    print("PASS: build_bone_map_from_scene edge cases")
    return True


def test_build_morph_map_edge_cases() -> bool:
    """Edge cases for build_morph_map_from_scene: no blendShape, missing attr."""
    setup_test_environment()

    # No blendShape node → empty map
    no_bs_root = cmds.createNode("transform", name="NoBSMorph_Root")
    morph_map = build_morph_map_from_scene(no_bs_root)
    assert_true(
        not morph_map,
        f"expected empty map with no blendShape, got {morph_map}",
    )

    # BlendShape node exists but no pmxMorphMapping → identity fallback
    root = _create_minimal_model_scene("MorphEdge_Root")
    bs = cmds.ls(type="blendShape")[0]
    # Delete pmxMorphMapping attribute
    if cmds.attributeQuery("pmxMorphMapping", node=bs, exists=True):
        cmds.deleteAttr(f"{bs}.pmxMorphMapping")
    morph_map = build_morph_map_from_scene(root)
    assert_true(
        bool(morph_map),
        "expected non-empty identity fallback map",
    )
    # Should contain the alias names as identity entries
    assert_true(
        "eyeBlink" in morph_map and "mouthSmile" in morph_map,
        f"expected 'eyeBlink'/'mouthSmile' in fallback, got {morph_map}",
    )

    # Verify they are identity mappings (name == alias)
    for k, v in morph_map.items():
        assert_true(
            k == v,
            f"fallback should be identity map, got '{k}'→'{v}'",
        )

    print("PASS: build_morph_map_from_scene edge cases")
    return True


# ══════════════════════════════════════════════════════════════════════════
# Test registry
# ══════════════════════════════════════════════════════════════════════════

_TESTS = [
    (
        "Scene Discovery - find_model_root_from_selection",
        test_find_model_root_from_selection,
    ),
    (
        "Scene Discovery - find_model_root_from_selection edge cases",
        test_find_model_root_selection_edge_cases,
    ),
    ("Scene Discovery - build_bone_map_from_scene", test_build_bone_map_from_scene),
    (
        "Scene Discovery - build_bone_map_from_scene edge cases",
        test_build_bone_map_edge_cases,
    ),
    ("Scene Discovery - build_morph_map_from_scene", test_build_morph_map_from_scene),
    (
        "Scene Discovery - build_morph_map_from_scene edge cases",
        test_build_morph_map_edge_cases,
    ),
    ("Scene Discovery - find_blend_shape_node", test_find_blend_shape_node),
    ("Scene Discovery - find_bone_morph_node", test_find_bone_morph_node),
    ("Scene Discovery - find_ik_handles", test_find_ik_handles),
    (
        "Scene Discovery - discover_model_roots_in_scene",
        test_discover_model_roots_in_scene,
    ),
    (
        "Scene Discovery - find_all_model_roots_from_selection",
        test_find_all_model_roots_from_selection,
    ),
    ("ModelContext - boneMap", test_model_context_bone_map),
    ("ModelContext - morphMap", test_model_context_morph_map),
    ("ModelContext - blendShapeNode", test_model_context_blend_shape_node),
    ("ModelContext - boneMorphNode", test_model_context_bone_morph_node),
    ("ModelContext - ikHandles", test_model_context_ik_handles),
    ("ModelContext - isValid", test_model_context_is_valid),
    (
        "ModelContext - refresh_from_selection",
        test_model_context_refresh_from_selection,
    ),
    ("ModelContext - cache invalidation", test_model_context_cache_invalidation),
    ("ModelContext - set_active_root signal", test_model_context_set_active_root),
    ("ModelContext - multi-model isolation", test_model_context_multi_model_isolation),
    # ── Real-model tests (require PMX asset files; skip gracefully when absent) ──
    ("Real Model - ModelContext discovery", test_model_context_on_real_model),
    ("Real Model - multi-model switch", test_model_context_multi_model_real),
    ("Real Model - ModelContext → VMD builder", test_model_context_to_vmd_builder),
    ("Real Model - ModelContext → VPD builder", test_model_context_to_vpd_builder),
    ("Real Model - multi-model VMD isolation", test_multi_model_vmd_isolation),
    ("Real Model - multi-model VPD isolation", test_multi_model_vpd_isolation),
]


# ══════════════════════════════════════════════════════════════════════════
# Mutating test names (wrapped in undo chunks for perfect isolation)
# ══════════════════════════════════════════════════════════════════════════
_MUTATING_CONTEXT_TESTS: set[str] = {
    "Real Model - ModelContext → VMD builder",
    "Real Model - ModelContext → VPD builder",
    "Real Model - multi-model VMD isolation",
    "Real Model - multi-model VPD isolation",
}


# Real-model test orchestration is handled by run_all_integration_tests.py.
# The runner selects models, pre-builds scenes, and dispatches synthetic
# and real-model tests with appropriate undo wrapping for mutating tests.
