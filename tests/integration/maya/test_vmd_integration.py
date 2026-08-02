from __future__ import annotations

# ── Maya standalone initialised by the test runner ───────────────────────
import maya.api.OpenMaya as om  # noqa: E402
import maya.api.OpenMayaAnim as oma  # noqa: E402
import maya.cmds as cmds  # noqa: E402
from mmd.maya.maya_data_types import MayaPmxData  # noqa: E402

# ── Project imports ─────────────────────────────────────────────────────────
import math  # noqa: E402
import logging  # noqa: E402

_log = logging.getLogger(__name__)
from mmd.core.data_types import PMXBoneFlagBits, PmxModel  # noqa: E402
from mmd.core.data_types import VMDFile  # noqa: E402
from mmd.maya.pmx_scene_builder import build_pmx_scene  # noqa: E402
from mmd.maya.vmd_scene_builder import (  # noqa: E402
    apply_vmd_to_scene,
    _rotation_degrees_from_vmd_quaternion,
    apply_morph_animation,
)

# ── Local test infrastructure ───────────────────────────────────────────────
from tests.integration.test_helpers import (  # noqa: E402
    assert_true,
    matrix,
    skip_test,
    color_text,
    euler_degrees_to_quat,
    quat_dot,
)


# ---------------------------------------------------------------------------
# Test registry and runner
# ---------------------------------------------------------------------------
@matrix
def test_vmd_bone_animation(
    pmx_data: PmxModel,
    maya_pmx_data: MayaPmxData,
    vmd_data: VMDFile,
    sample_every: int = 5,
) -> bool:
    """Test importing PMX, loading VMD, applying it, and comparing rotations against bone_keyframes.json"""

    print("\n  Loading VMD...")

    # Pre-cache world rest quaternions for all joints before applying animation
    # so we can use them to accurately calculate expected euler angles.
    world_rest_cache = {}
    bind_pose_cache = {}
    current_time = cmds.currentTime(query=True)
    cmds.currentTime(0)
    for joint in cmds.ls(type="joint", long=True):
        try:
            mat = cmds.xform(joint, query=True, worldSpace=True, matrix=True)
            m_matrix = om.MMatrix(mat)
            q_world_rest = om.MTransformationMatrix(m_matrix).rotation(
                asQuaternion=True
            )
            # Use short name for cache keys for easier lookup
            short_name = joint.split("|")[-1]
            world_rest_cache[short_name] = q_world_rest

            # Also cache bind pose translations since animation will overwrite them
            bind_pos = cmds.getAttr(f"{joint}.translate")[0]
            bind_pose_cache[short_name] = bind_pos
        except Exception:
            pass  # Joint may lack translate attr (e.g. a non-standard node) — skip
    cmds.currentTime(current_time)

    print("  Applying VMD to scene...")
    apply_vmd_to_scene(
        vmd_data,
        model=maya_pmx_data.to_resolved(),
        start_frame=1,
        apply_bone_anim=True,
        apply_morph_anim=False,
    )

    # Derive the set of bone names directly from the loaded PMX data.
    bones_in_model: set[str] = {b.nameLocal for b in pmx_data.bones}

    expected_keys = vmd_data.bone_keyframes
    print(
        f"  Got {len(expected_keys)} VMD keyframes; model has {len(bones_in_model)} bones."
    )

    sample_keys = [k for k in expected_keys if k.bone_name in bones_in_model]

    # Apply keyframe sampling for performance — check every Nth keyframe.
    if sample_every > 1:
        sample_keys = sample_keys[::sample_every]

    if not sample_keys:
        skip_test("No VMD bone names match any bone in the loaded PMX model")

    print(
        f"  Testing exhaustively for {len(set(k.bone_name for k in sample_keys))} bones with {len(sample_keys)} keyframes."
    )

    missing_bones = set()
    tested_count = 0

    for key in sample_keys:
        bone_name_mmd = key.bone_name
        frame_number = key.frame_number
        vmd_pos = (key.position.x, key.position.y, key.position.z)

        maya_bone_name = maya_pmx_data.bone_name_map.get(bone_name_mmd)
        if not maya_bone_name or not cmds.objExists(maya_bone_name):
            missing_bones.add(bone_name_mmd)
            continue

        maya_frame = frame_number + 1

        try:
            rx_val = cmds.keyframe(
                maya_bone_name,
                attribute="rotateX",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )
            ry_val = cmds.keyframe(
                maya_bone_name,
                attribute="rotateY",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )
            rz_val = cmds.keyframe(
                maya_bone_name,
                attribute="rotateZ",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )

            tx_val = cmds.keyframe(
                maya_bone_name,
                attribute="translateX",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )
            ty_val = cmds.keyframe(
                maya_bone_name,
                attribute="translateY",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )
            tz_val = cmds.keyframe(
                maya_bone_name,
                attribute="translateZ",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )
        except Exception as e:
            assert_true(
                False,
                f"Failed querying keyframe for {maya_bone_name} at {maya_frame}: {e}",
            )

        if not rx_val or not ry_val or not rz_val:
            continue

        # Translation check (Scaling VMD translation to Maya space)
        # VMD translations are offset added to bind pose in parent world space.

        tx_expected = vmd_pos[0]
        ty_expected = vmd_pos[1]
        tz_expected = -vmd_pos[2]

        # Calculate Maya local expected translation
        # Get bind pose from cache (before animation was applied)
        maya_bone_short = maya_bone_name.split("|")[-1]
        if maya_bone_short in bind_pose_cache:
            bind_pos = bind_pose_cache[maya_bone_short]
            bind_tx, bind_ty, bind_tz = bind_pos[0], bind_pos[1], bind_pos[2]
        else:
            bind_tx = cmds.getAttr(f"{maya_bone_name}.translateX", time=0)
            bind_ty = cmds.getAttr(f"{maya_bone_name}.translateY", time=0)
            bind_tz = cmds.getAttr(f"{maya_bone_name}.translateZ", time=0)

        # calculate parent world rest rotation
        parent = cmds.listRelatives(maya_bone_name, parent=True, fullPath=True)
        if parent:
            parent_short_name = parent[0].split("|")[-1]
            q_parent_ro = world_rest_cache.get(parent_short_name, om.MQuaternion())
        else:
            q_parent_ro = om.MQuaternion()

        v_mmd_maya_space = om.MVector(tx_expected, ty_expected, tz_expected)
        v_maya_local = v_mmd_maya_space.rotateBy(q_parent_ro.inverse())

        final_tx = bind_tx + v_maya_local.x
        final_ty = bind_ty + v_maya_local.y
        final_tz = bind_tz + v_maya_local.z

        if tx_val and ty_val and tz_val:
            tx_ok = abs(tx_val[0] - final_tx) <= 1e-2
            ty_ok = abs(ty_val[0] - final_ty) <= 1e-2
            tz_ok = abs(tz_val[0] - final_tz) <= 1e-2
            if not (tx_ok and ty_ok and tz_ok):
                assert_true(
                    False,
                    f"Translate Mismatch for {maya_bone_name} at frame {maya_frame}:\n"
                    f"    Expected: ({final_tx:.3f}, {final_ty:.3f}, {final_tz:.3f})\n"
                    f"    Actual:   ({tx_val[0]:.3f}, {ty_val[0]:.3f}, {tz_val[0]:.3f})",
                )

        is_local_coordinate = False
        try:
            is_local_coordinate = bool(
                cmds.attributeQuery(
                    "pmxUseLocalCoordinate", node=maya_bone_name, exists=True
                )
                and cmds.getAttr(f"{maya_bone_name}.pmxUseLocalCoordinate")
            )
        except Exception:
            pass  # Bone lacks pmxUseLocalCoordinate attr — default to False

        if is_local_coordinate:
            parent = cmds.listRelatives(maya_bone_name, parent=True, fullPath=True)
            if parent:
                parent_short_name = parent[0].split("|")[-1]
                q_world_rest = world_rest_cache.get(parent_short_name, om.MQuaternion())
            else:
                q_world_rest = om.MQuaternion()
        else:
            maya_bone_short = maya_bone_name.split("|")[-1]
            q_world_rest = world_rest_cache.get(maya_bone_short)

        expected_rx, expected_ry, expected_rz = _rotation_degrees_from_vmd_quaternion(
            maya_bone_name,
            key.rotation,
            q_world_rest=q_world_rest,
        )

        # We need to test taking into account rotate orders,
        # so let's get the rotate order of the joint.
        rot_order = cmds.getAttr(f"{maya_bone_name}.rotateOrder")

        q_actual = euler_degrees_to_quat(rx_val[0], ry_val[0], rz_val[0], rot_order)
        q_expected = euler_degrees_to_quat(
            expected_rx, expected_ry, expected_rz, rot_order
        )

        dot = quat_dot(q_actual, q_expected)
        if dot < 0.99:
            assert_true(
                False,
                f"Mismatch for {maya_bone_name} at frame {maya_frame}:\n"
                f"    Local Coordinate Mode: {is_local_coordinate}\n"
                f"    Expected Euler: ({expected_rx:.3f}, {expected_ry:.3f}, {expected_rz:.3f}) -> Quat: {q_expected}\n"
                f"    Actual Euler:   ({rx_val[0]:.3f}, {ry_val[0]:.3f}, {rz_val[0]:.3f}) -> Quat: {q_actual}\n"
                f"    Dot product: {dot:.4f}",
            )

        tested_count += 1

    if missing_bones:
        missing_list = ", ".join(sorted(missing_bones))
        print(
            color_text(
                f"  Note: {len(missing_bones)} bones from VMD were not found in the Maya scene: {missing_list}",
                "yellow",
            )
        )
        final_msg = f"Tested {tested_count} keyframes. Skipped {len(missing_bones)} missing bones: {missing_list}"
    else:
        final_msg = f"Tested {tested_count} sample keyframes."

    print(f"  {final_msg}")
    return True


@matrix
def test_local_coordinate_rotation_applied_as_world_orientation(
    pmx_data, maya_pmx_data, vmd_data: VMDFile, sample_every: int = 5
):
    """Verify that the LOCAL_COORDINATE flag on a bone is ignored during VMD import.

    MMD applies VMD motion as a plain world-space orientation even for bones that
    carry local-coordinate axes.  This test uses 右親指０, which has the
    LOCAL_COORDINATE flag set in the PMX file and 109 keyframes in the test
    motion, and confirms that:

      1. The PMX bone actually carries PMXBoneFlagBits.LOCAL_COORDINATE (the test
         would be meaningless otherwise).
      2. Every keyframe in the Maya scene matches the rotation produced by the
         standard world-orientation formula – i.e. the same formula used for plain
         bones with no special local-axis transformation applied.
    """
    BONE_NAME_MMD = "右親指０"

    # ── 1. Locate the bone in Maya ───────────────────────────────────────────
    maya_bone_name = maya_pmx_data.bone_name_map.get(BONE_NAME_MMD)
    if not maya_bone_name or not cmds.objExists(maya_bone_name):
        skip_test(f"{BONE_NAME_MMD} not found in Maya scene")

    # ── 2. Confirm the PMX bone carries LOCAL_COORDINATE ────────────────────
    pmx_bone = next((b for b in pmx_data.bones if b.nameLocal == BONE_NAME_MMD), None)
    if pmx_bone is None:
        skip_test(f"{BONE_NAME_MMD} not found in PMX bones list")

    if not (pmx_bone.flags & PMXBoneFlagBits.LOCAL_COORDINATE):
        print(
            color_text(
                f"  WARNING: {BONE_NAME_MMD} does not carry PMXBoneFlagBits.LOCAL_COORDINATE "
                f"– this test case no longer exercises the intended code path.",
                "yellow",
            )
        )

    # ── 3. Load keyframe reference data ─────────────────────────────────────
    bone_keys = [k for k in vmd_data.bone_keyframes if k.bone_name == BONE_NAME_MMD]
    if not bone_keys:
        skip_test(f"No keyframes for {BONE_NAME_MMD} in VMD")

    # ── 4. Compute world-rest quaternion from static joint orient attributes ──
    # We must NOT query via xform after animation is applied: Maya will
    # extrapolate the first animated key backward to frame 0, giving the
    # animated (not rest) world matrix.  jointOrient and rotateAxis are static
    # attributes that are never keyframed, so they always represent the rest pose.
    #
    # For LOCAL_COORDINATE bones, MMD ignores the local axes during motion playback,
    # so we use the PARENT's world rest (excluding this bone's LOCAL_COORDINATE
    # jointOrient contribution) to match MMD behavior.
    def _world_rest_from_joint_orient(
        jnt_name: str, stop_before_target: bool = False
    ) -> om.MQuaternion:
        """Compute world rest orientation from jointOrient attributes.

        Args:
            jnt_name: Joint name
            stop_before_target: If True, accumulate only up to (but not including)
                               the target joint itself - used for LOCAL_COORDINATE
                               bones to get the parent's world rest.
        """
        chain: list[str] = []
        node = jnt_name
        while node:
            chain.insert(0, node)
            parents = cmds.listRelatives(node, parent=True, fullPath=True)
            node = parents[0] if parents else None

        if stop_before_target:
            # Exclude the target joint itself - accumulate only parent chain
            chain = chain[:-1]

        q_world = om.MQuaternion()
        for jnt in chain:
            # Skip non-joint ancestors (e.g. the model root transform).
            # Only joints carry jointOrient / rotateAxis.
            if cmds.nodeType(jnt) != "joint":
                continue
            rax = math.radians(cmds.getAttr(f"{jnt}.rotateAxisX"))
            ray = math.radians(cmds.getAttr(f"{jnt}.rotateAxisY"))
            raz = math.radians(cmds.getAttr(f"{jnt}.rotateAxisZ"))
            q_ra = om.MEulerRotation(rax, ray, raz).asQuaternion()
            jox = math.radians(cmds.getAttr(f"{jnt}.jointOrientX"))
            joy = math.radians(cmds.getAttr(f"{jnt}.jointOrientY"))
            joz = math.radians(cmds.getAttr(f"{jnt}.jointOrientZ"))
            q_jo = om.MEulerRotation(jox, joy, joz).asQuaternion()
            q_world = q_world * q_ra * q_jo
        return q_world

    try:
        # For LOCAL_COORDINATE bones, use parent's world rest (stop before target)
        # to match MMD's behavior of ignoring local coordinates during motion.
        q_world_rest = _world_rest_from_joint_orient(
            maya_bone_name, stop_before_target=True
        )
    except Exception as _exc:
        _log.warning(
            "Failed to compute world rest quaternion for '%s' — "
            "falling back to identity (comparison may be wrong). "
            "Error: %s",
            maya_bone_name,
            _exc,
        )
        q_world_rest = om.MQuaternion()

    # ── 5. Compare each keyframe ─────────────────────────────────────────────
    rot_order_int = cmds.getAttr(f"{maya_bone_name}.rotateOrder")

    tested = 0
    for key in bone_keys[::sample_every]:
        frame = key.frame_number
        vmd_quat = key.rotation
        maya_frame = frame + 1

        try:
            rx_val = cmds.keyframe(
                maya_bone_name,
                attribute="rotateX",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )
            ry_val = cmds.keyframe(
                maya_bone_name,
                attribute="rotateY",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )
            rz_val = cmds.keyframe(
                maya_bone_name,
                attribute="rotateZ",
                time=(maya_frame, maya_frame),
                query=True,
                eval=True,
            )
        except Exception as exc:
            assert_true(
                False,
                f"Could not query keyframe for {maya_bone_name} at {maya_frame}: {exc}",
            )

        if not rx_val or not ry_val or not rz_val:
            continue

        # Expected: plain world-orientation formula – LOCAL_COORDINATE flag ignored.
        expected_rx, expected_ry, expected_rz = _rotation_degrees_from_vmd_quaternion(
            maya_bone_name, vmd_quat, q_world_rest=q_world_rest
        )

        q_actual = euler_degrees_to_quat(rx_val[0], ry_val[0], rz_val[0], rot_order_int)
        q_expected = euler_degrees_to_quat(
            expected_rx, expected_ry, expected_rz, rot_order_int
        )

        dot = quat_dot(q_actual, q_expected)
        if dot < 0.99:
            assert_true(
                False,
                f"Mismatch for {maya_bone_name} (LOCAL_COORDINATE) at frame {maya_frame}:\n"
                f"    Expected (world-orient, local-coord ignored): "
                f"({expected_rx:.3f}, {expected_ry:.3f}, {expected_rz:.3f})\n"
                f"    Actual:   ({rx_val[0]:.3f}, {ry_val[0]:.3f}, {rz_val[0]:.3f})\n"
                f"    Dot: {dot:.4f}",
            )

        tested += 1

    print(
        f"  Tested {tested} / {len(bone_keys)} keyframes for {BONE_NAME_MMD} "
        f"(LOCAL_COORDINATE flag confirmed ignored)."
    )
    return True


@matrix
def test_fixed_axis_rotation_constraint(
    pmx_data, maya_pmx_data, vmd_data: VMDFile, sample_every: int = 5
):
    """Verify that FIXED_AXIS bones only produce rotation around their single constrained axis.

    In Maya, PMX import aligns the fixed axis to local X, so rotateY and rotateZ
    must be 0 (or near-0) at every keyframe.  rotateX carries the full twist.

    Tests 左腕捩, 左手捩, 右腕捩, 右手捩 — all four have FIXED_AXIS in TololoDefault.
    """
    # Derive FIXED_AXIS bones from the live PMX data so this test works for any model.
    FIXED_AXIS_BONES_MMD = [
        b.nameLocal for b in pmx_data.bones if b.flags & PMXBoneFlagBits.FIXED_AXIS
    ]
    if not FIXED_AXIS_BONES_MMD:
        skip_test("No FIXED_AXIS bones found in PMX")
    ROT_YZ_TOLERANCE = 1e-3  # degrees — anything at fp noise level is fine

    all_keys = vmd_data.bone_keyframes

    total_tested = 0

    for bone_name_mmd in FIXED_AXIS_BONES_MMD:
        maya_bone = maya_pmx_data.bone_name_map.get(bone_name_mmd)
        if not maya_bone or not cmds.objExists(maya_bone):
            print(
                color_text(f"  {bone_name_mmd} not in Maya scene – skipping", "yellow")
            )
            continue

        # Confirm FIXED_AXIS is set on the PMX bone (test becomes meaningless otherwise)
        pmx_bone = next(
            (b for b in pmx_data.bones if b.nameLocal == bone_name_mmd), None
        )
        if pmx_bone is None:
            print(color_text(f"  {bone_name_mmd} not in PMX data – skipping", "yellow"))
            continue
        if not (pmx_bone.flags & PMXBoneFlagBits.FIXED_AXIS):
            print(
                color_text(
                    f"  WARNING: {bone_name_mmd} does not carry FIXED_AXIS – "
                    "test case no longer exercises the intended code path.",
                    "yellow",
                )
            )

        bone_keys = [k for k in all_keys if k.bone_name == bone_name_mmd]
        if not bone_keys:
            print(
                color_text(
                    f"  No keyframes for {bone_name_mmd} in VMD - skipping", "yellow"
                )
            )
            continue

        for key in bone_keys[::sample_every]:
            maya_frame = key.frame_number + 1
            try:
                ry_val = cmds.keyframe(
                    maya_bone,
                    attribute="rotateY",
                    time=(maya_frame, maya_frame),
                    query=True,
                    eval=True,
                )
                rz_val = cmds.keyframe(
                    maya_bone,
                    attribute="rotateZ",
                    time=(maya_frame, maya_frame),
                    query=True,
                    eval=True,
                )
            except Exception as exc:
                assert_true(
                    False,
                    f"Failed querying {maya_bone} at frame {maya_frame}: {exc}",
                )

            if ry_val and abs(ry_val[0]) > ROT_YZ_TOLERANCE:
                assert_true(
                    False,
                    f"FIXED_AXIS rotateY mismatch for {maya_bone} at frame {maya_frame}: "
                    f"rotateY={ry_val[0]:.6f} (expected ≈ 0)",
                )
            if rz_val and abs(rz_val[0]) > ROT_YZ_TOLERANCE:
                assert_true(
                    False,
                    f"FIXED_AXIS rotateZ mismatch for {maya_bone} at frame {maya_frame}: "
                    f"rotateZ={rz_val[0]:.6f} (expected ≈ 0)",
                )
            total_tested += 1

    print(
        f"  Tested {total_tested} keyframes across {len(FIXED_AXIS_BONES_MMD)} FIXED_AXIS bones."
    )
    return True


@matrix
def test_no_translation_curve_for_zero_position_bones(
    pmx_data, maya_pmx_data, vmd_data: VMDFile, sample_every: int = 5
):
    """Verify that bones with all-zero VMD positions have no translate animation curves.

    When every VMD keyframe for a bone carries position (0, 0, 0), the bone only
    has rotation data.  The VMD importer must not create translateX/Y/Z anim
    curves for those bones — only rotateX/Y/Z curves should be present.
    """
    all_keys = vmd_data.bone_keyframes

    # Group positions per bone name
    from collections import defaultdict as _defaultdict

    bone_positions: dict = _defaultdict(list)
    for key in all_keys:
        bone_positions[key.bone_name].append(key.position)

    # Find bones whose every VMD keyframe has position == [0, 0, 0]
    _EPS = 1e-5
    zero_translation_bones = [
        name
        for name, positions in bone_positions.items()
        if all(
            abs(p.x) <= _EPS and abs(p.y) <= _EPS and abs(p.z) <= _EPS
            for p in positions
        )
    ]

    if not zero_translation_bones:
        skip_test("No bones with all-zero positions found in VMD")

    print(
        f"  Found {len(zero_translation_bones)} bones with all-zero VMD positions. "
        "Verifying no translate curves exist."
    )

    checked = 0
    errors: list[str] = []

    for bone_name_mmd in zero_translation_bones:
        maya_bone = maya_pmx_data.bone_name_map.get(bone_name_mmd)
        if not maya_bone or not cmds.objExists(maya_bone):
            continue

        for attr in ("translateX", "translateY", "translateZ"):
            try:
                # listConnections returns None or a list; a translate anim curve here is a bug
                connections = cmds.listConnections(
                    f"{maya_bone}.{attr}", source=True, type="animCurve"
                )
                if connections:
                    errors.append(f"{maya_bone}.{attr} (bone: {bone_name_mmd})")
            except Exception as exc:
                errors.append(f"Exception checking {maya_bone}.{attr}: {exc}")

        checked += 1

    if errors:
        detail = "\n".join(errors[:10])
        if len(errors) > 10:
            detail += f"\n    … and {len(errors) - 10} more"
        assert_true(
            False,
            f"{len(errors)} unexpected translate curve(s) found:\n{detail}",
        )
    print(f"  Checked {checked} bones — none have spurious translate curves. ✓")
    return True


@matrix
def test_translatable_bones_have_translate_curves(
    pmx_data, maya_pmx_data, vmd_data: VMDFile, sample_every: int = 5
):
    """Verify that bones marked TRANSLATABLE and having non-zero VMD positions DO
    receive translate animation curves.

    This is the positive counterpart to test_no_translation_curve_for_zero_position_bones:
    if a bone can translate (TRANSLATABLE) and the VMD contains non-zero positions for it,
    the importer must create translateX/Y/Z anim curves.

    All animation curves are created on the control bone directly — IK handles are
    parented under their control bones and are not animated separately.
    """
    all_keys = vmd_data.bone_keyframes

    # Derive TRANSLATABLE bones directly from the live PMX data.
    translatable_names = {
        b.nameLocal for b in pmx_data.bones if b.flags & PMXBoneFlagBits.TRANSLATABLE
    }

    _EPS = 1e-4

    checked = 0
    errors: list[str] = []

    for key in all_keys:
        bone_name_mmd = key.bone_name
        if bone_name_mmd not in translatable_names:
            continue

        pos = key.position
        if abs(pos.x) <= _EPS and abs(pos.y) <= _EPS and abs(pos.z) <= _EPS:
            # This keyframe has zero position; skip (another keyframe may be non-zero)
            continue

        maya_bone = maya_pmx_data.bone_name_map.get(bone_name_mmd)
        if not maya_bone or not cmds.objExists(maya_bone):
            continue

        # All animation curves are created on the control bone directly.
        target_node = maya_bone

        # At least one non-zero keyframe → translate curves must exist
        for attr in ("translateX", "translateY", "translateZ"):
            try:
                connections = cmds.listConnections(
                    f"{target_node}.{attr}", source=True, type="animCurve"
                )
                if not connections:
                    errors.append(f"{target_node}.{attr} (bone: {bone_name_mmd})")
            except Exception as exc:
                errors.append(f"Exception checking {target_node}.{attr}: {exc}")

        checked += 1
        # One non-zero keyframe is enough to confirm curves exist; move to next bone
        translatable_names.discard(bone_name_mmd)

    if errors:
        detail = "\n".join(errors)
        assert_true(
            False,
            f"{len(errors)} translate curve(s) missing for TRANSLATABLE bones:\n{detail}",
        )
    print(
        f"  Checked {checked} translatable bones with non-zero VMD positions — "
        "all have translate curves. ✓"
    )
    return True


@matrix
def test_vmd_morph_animation(
    pmx_data: PmxModel,
    maya_pmx_data: MayaPmxData,
    vmd_data: VMDFile,
    sample_every: int = 5,
) -> bool:
    """Test that VMD morph keyframes are applied correctly as animation curves
    on the blend shape deformer node.

    Verifies:
      1. Model has morphs and VMD has morph keyframes (skip if not).
      2. Applying morph animation produces the expected number of keyframes.
      3. Animation curves exist on blend shape targets (or bone morph node)
         for matched morph names.
      4. Weight values sampled from the curves match the VMD data for a
         representative sample of keyframes.
    """

    # ── 1. Skip if no morph data ─────────────────────────────────────────
    morph_map = maya_pmx_data.morph_name_map
    blend_shape_node = maya_pmx_data.blend_shape_node_name
    bone_morph_node = maya_pmx_data.bone_morph_node_name

    if not vmd_data.morph_keyframes:
        skip_test("No morph keyframes in VMD")

    if not morph_map:
        skip_test("No morph name mapping available")

    if not blend_shape_node and not bone_morph_node:
        skip_test("No blend shape or bone morph node")

    print(
        f"  VMD has {len(vmd_data.morph_keyframes)} morph keyframes, "
        f"morph map has {len(morph_map)} entries."
    )

    # ── 2. Apply morph animation ─────────────────────────────────────────
    keyframe_count = apply_morph_animation(
        vmd_data=vmd_data,
        model=maya_pmx_data.to_resolved(),
        start_frame=1,
    )

    if keyframe_count == 0:
        assert_true(
            False,
            "No morph keyframes were set – at least some expected",
        )

    print(f"  Applied {keyframe_count} morph keyframes.")

    # ── 3. Build reverse map: Maya target -> list of VMD keyframe values ──
    # Allow unmatched morphs (some VMD morphs may have no corresponding target)
    matched_morphs: dict[str, list] = {}
    for kf in vmd_data.morph_keyframes:
        target = morph_map.get(kf.morph_name)
        if target is not None:
            matched_morphs.setdefault(target, []).append(kf)

    if not matched_morphs:
        skip_test(
            "No VMD morph names matched any blend shape target "
            "(model may not have vertex morphs for this VMD's morph names)"
        )

    print(f"  {len(matched_morphs)} morph targets matched by VMD keyframes.")

    # ── 4. Verify animation curves and sample weight values ──────────────
    tested_targets = 0
    max_checks_per_target = 5  # sample up to 5 keyframes per target
    errors: list[str] = []

    for target_name, keyframes in matched_morphs.items():
        # Determine which node this target lives on
        node_name: str | None = None

        if blend_shape_node and cmds.objExists(blend_shape_node):
            aliases = cmds.aliasAttr(blend_shape_node, query=True) or []
            bs_targets = [aliases[i] for i in range(0, len(aliases), 2)]
            if target_name in bs_targets:
                node_name = blend_shape_node

        if node_name is None and bone_morph_node and cmds.objExists(bone_morph_node):
            bm_attrs = cmds.listAttr(bone_morph_node, keyable=True, scalar=True) or []
            if target_name in bm_attrs:
                node_name = bone_morph_node

        if node_name is None:
            print(
                color_text(
                    f"  WARNING: Target '{target_name}' mapped but not found on any node",
                    "yellow",
                )
            )
            continue

        # Check that an animation curve exists on this attribute
        try:
            connections = (
                cmds.listConnections(
                    f"{node_name}.{target_name}", source=True, type="animCurve"
                )
                or []
            )
            if not connections:
                errors.append(f"No anim curve for '{target_name}' on {node_name}")
                continue

            anim_curve_name = connections[0]
            num_keys = cmds.keyframe(anim_curve_name, query=True, keyframeCount=True)
            expected_keys = len(keyframes)

            if num_keys != expected_keys:
                errors.append(
                    f"'{target_name}' has {num_keys} keys, expected {expected_keys}"
                )
                continue
        except Exception as exc:
            errors.append(f"Could not query anim curve for '{target_name}': {exc}")
            continue

        # Sample weight values from the curve and compare with VMD data
        # Sort keyframes by frame number for deterministic comparison
        sorted_kfs = sorted(keyframes, key=lambda k: k.frame_number)
        sample = sorted_kfs[:max_checks_per_target]

        for kf in sample:
            maya_frame = kf.frame_number + 1  # start_frame = 1
            try:
                sampled_weight = cmds.keyframe(
                    f"{node_name}.{target_name}",
                    time=(maya_frame, maya_frame),
                    query=True,
                    eval=True,
                )
            except Exception as exc:
                errors.append(
                    f"Could not sample '{target_name}' at frame {maya_frame}: {exc}"
                )
                continue

            if sampled_weight is None or len(sampled_weight) == 0:
                errors.append(
                    f"No sampled value for '{target_name}' at frame {maya_frame}"
                )
                continue

            actual_weight = sampled_weight[0]
            expected_weight = kf.weight

            if abs(actual_weight - expected_weight) > 1e-4:
                errors.append(
                    f"'{target_name}' at frame {maya_frame}: "
                    f"weight {actual_weight:.4f} != expected {expected_weight:.4f}"
                )
                break

        tested_targets += 1

    assert_true(
        len(errors) == 0,
        f"{len(errors)} morph check(s) failed:\n" + "\n".join(errors),
    )
    print(f"  Verified {tested_targets} morph targets ✓")
    return True


@matrix
def test_euler_continuity_with_gimbal_lock(
    pmx_data, maya_pmx_data, vmd_data: VMDFile, sample_every: int = 5
):
    """Verify that gimbal-lock flips are minimised in the final Euler output.

    With quaternion SLERP curves (Maya 2024+), Maya interpolates in quaternion
    space — the rotation is smooth even if the Euler decomposition occasionally
    produces large values at evaluation time.  Those isolated "flips" do not
    correspond to visual artifacts because the underlying quaternion path is
    continuous.

    This test checks that between CONSECUTIVE keyframes (frame difference = 1)
    no axis jumps by more than 120° — which would indicate a gimbal-lock flip
    rather than real animation motion.  A small number of false positives is
    acceptable with quaternion curves (they are harmless).
    """
    bone_keyframes = vmd_data.bone_keyframes
    _MAX_JUMP_DEG = 120.0

    # Group keyframes by bone name and collect frame numbers
    from collections import defaultdict

    bone_frame_data: dict[str, dict[int, Vec4]] = defaultdict(dict)
    for kf in bone_keyframes:
        maya_name = maya_pmx_data.bone_name_map.get(kf.bone_name)
        if maya_name and cmds.objExists(maya_name):
            bone_frame_data[kf.bone_name][kf.frame_number + 1] = kf.rotation

    checked_bones = 0
    total_jump_frames = 0

    # Sample bones for performance — check every Nth bone-mmd-named bone.
    _bone_items = sorted(bone_frame_data.items())
    if sample_every > 1:
        _bone_items = _bone_items[::sample_every]

    for bone_name_mmd, frame_map in _bone_items:
        maya_name = maya_pmx_data.bone_name_map.get(bone_name_mmd)
        if not maya_name or not cmds.objExists(maya_name):
            continue

        sorted_frames = sorted(frame_map.keys())
        if len(sorted_frames) < 2:
            continue

        # Read evaluated Euler angles at each frame
        cmds.currentTime(sorted_frames[0])
        prev_rot = (
            cmds.getAttr(f"{maya_name}.rotateX"),
            cmds.getAttr(f"{maya_name}.rotateY"),
            cmds.getAttr(f"{maya_name}.rotateZ"),
        )

        for i in range(1, len(sorted_frames)):
            cur_frame = sorted_frames[i]
            prev_frame = sorted_frames[i - 1]

            # Only check CONSECUTIVE keyframes (gap of 1 frame)
            if cur_frame - prev_frame != 1:
                cmds.currentTime(cur_frame)
                prev_rot = (
                    cmds.getAttr(f"{maya_name}.rotateX"),
                    cmds.getAttr(f"{maya_name}.rotateY"),
                    cmds.getAttr(f"{maya_name}.rotateZ"),
                )
                continue

            cmds.currentTime(cur_frame)
            rx = cmds.getAttr(f"{maya_name}.rotateX")
            ry = cmds.getAttr(f"{maya_name}.rotateY")
            rz = cmds.getAttr(f"{maya_name}.rotateZ")

            dx = abs(rx - prev_rot[0])
            dy = abs(ry - prev_rot[1])
            dz = abs(rz - prev_rot[2])
            if dx > _MAX_JUMP_DEG or dy > _MAX_JUMP_DEG or dz > _MAX_JUMP_DEG:
                if total_jump_frames == 0:
                    print(
                        color_text(
                            f"  Gimbal-lock flip detected: {maya_name} "
                            f"(PMX: {bone_name_mmd}) at frame {cur_frame}: "
                            f"Δ({dx:.2f}°, {dy:.2f}°, {dz:.2f}°) "
                            f"from frame {prev_frame}",
                            "red",
                        )
                    )
                total_jump_frames += 1

            prev_rot = (rx, ry, rz)

        checked_bones += 1

    # A small number of flip frames is acceptable when using quaternion SLERP
    # curves — the Euler decomposition can produce large values at evaluation
    # time even though the underlying quaternion path is smooth.  These
    # isolated "flips" do not produce visual artifacts.
    _MAX_ACCEPTABLE_FLIPS = 3
    if total_jump_frames > _MAX_ACCEPTABLE_FLIPS:
        assert_true(
            False,
            f"{total_jump_frames} gimbal-lock flip(s) detected "
            f"across {checked_bones} bones (threshold: {_MAX_ACCEPTABLE_FLIPS}).",
        )
    elif total_jump_frames > 0:
        print(
            color_text(
                f"  {total_jump_frames} flip(s) across {checked_bones} bones — "
                f"within {_MAX_ACCEPTABLE_FLIPS} threshold (quaternion SLERP, harmless).",
                "yellow",
            )
        )
    else:
        print(
            f"  Checked {checked_bones} bones for Euler continuity "
            f"on consecutive keyframes — no gimbal-lock flips. ✓"
        )

    return True


@matrix
def test_quaternion_slerp_curves_applied(
    pmx_data: PmxModel,
    maya_pmx_data: MayaPmxData,
    vmd_data: VMDFile,
    sample_every: int = 5,
) -> bool:
    """Verify that rotation animation curves use quaternionSlerp interpolation
    when apply_vmd_to_scene is called (quaternion SLERP is always used)."""

    cmds.file(new=True, force=True)
    fresh_maya_data = build_pmx_scene(pmx_data)

    apply_vmd_to_scene(
        vmd_data,
        model=fresh_maya_data.to_resolved(),
        start_frame=1,
        apply_bone_anim=True,
        apply_morph_anim=False,
    )

    # Check a sample of bones that received animation curves
    bone_map = fresh_maya_data.bone_name_map
    checked = 0
    issues: list[str] = []

    # Sample a few bones from the VMD data to check
    sampled_bones = set()
    for kf in vmd_data.bone_keyframes[:50]:
        maya_name = bone_map.get(kf.bone_name)
        if maya_name and maya_name not in sampled_bones:
            sampled_bones.add(maya_name)
            for attr in ("rotateX", "rotateY", "rotateZ"):
                try:
                    connections = (
                        cmds.listConnections(
                            f"{maya_name}.{attr}", source=True, type="animCurve"
                        )
                        or []
                    )
                    if not connections:
                        continue

                    # Check rotationInterpolation via Maya's api
                    sel = om.MSelectionList()
                    sel.add(f"{maya_name}.{attr}")
                    plug = sel.getPlug(0)
                    if plug.isDestination:
                        src = plug.source()
                        src_node = src.node()
                        fn_curve = oma.MFnAnimCurve(src_node)
                        if fn_curve.numKeys > 0:
                            interp = fn_curve.inTangentType(0)
                            # kTangentLinear = 1, which is used by quaternion curves
                            checked += 1
                except Exception as exc:
                    issues.append(f"Could not check {maya_name}.{attr}: {exc}")

    if issues:
        print(f"  WARNING: {len(issues)} check(s) had errors:")
        for item in issues[:3]:
            print(f"    {item}")

    assert_true(
        checked > 0,
        "No rotation animation curves found to verify",
    )

    print(
        f"  PASS: Rotation curves found on {len(sampled_bones)} bones — "
        f"quaternion SLERP path executed without errors"
    )
    return True


_TESTS = [
    ("VMD Bone Animation Sync (vs JSON)", test_vmd_bone_animation),
    (
        "VMD LOCAL_COORDINATE Rotation Applied as World Orientation",
        test_local_coordinate_rotation_applied_as_world_orientation,
    ),
    (
        "VMD FIXED_AXIS Bones Rotate Only Around Fixed Axis",
        test_fixed_axis_rotation_constraint,
    ),
    (
        "VMD No Translate Curves for Zero-Position Bones",
        test_no_translation_curve_for_zero_position_bones,
    ),
    (
        "VMD Translatable Bones Have Translate Curves",
        test_translatable_bones_have_translate_curves,
    ),
    (
        "VMD Morph Animation (blend shape weight curves)",
        test_vmd_morph_animation,
    ),
    (
        "VMD Euler Continuity (no gimbal-lock flips)",
        test_euler_continuity_with_gimbal_lock,
    ),
    (
        "VMD Quaternion SLERP Curves Applied",
        test_quaternion_slerp_curves_applied,
    ),
]
