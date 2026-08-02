import os

import pytest

# Use robust, location-independent path resolution
from assets.assets_utils import get_all_vmd_paths, is_lfs_pointer_file
from mmd.core.vmd_importer import parse_vmd_file

TEST_VMD_FILES = get_all_vmd_paths()


@pytest.mark.parametrize("vmd_path", TEST_VMD_FILES)
def test_parse_vmd_loads_motion(vmd_path, caplog):
    """Test that each VMD motion file can be successfully parsed."""
    assert os.path.exists(vmd_path), f"VMD file does not exist: {vmd_path}"

    if is_lfs_pointer_file(vmd_path):
        pytest.skip(f"VMD file is a Git LFS pointer (run 'git lfs pull'): {vmd_path}")

    with caplog.at_level("WARNING"):
        try:
            vmd_file = parse_vmd_file(vmd_path)
        except Exception as e:
            pytest.fail(f"parse_vmd_file failed for {vmd_path}: {e}")

        warnings = [
            record for record in caplog.records if record.levelname >= "WARNING"
        ]
        if warnings:
            print(f"Warnings while parsing {vmd_path}:")
            for warning in warnings:
                print(f"  - {warning.message}")

        assert vmd_file is not None, f"parse_vmd_file returned None for {vmd_path}"

        # Check basic properties
        assert hasattr(vmd_file, "header"), "VMD file missing header property"
        assert hasattr(vmd_file, "version"), "VMD file missing version property"
        assert hasattr(vmd_file, "model_name"), "VMD file missing model_name property"
        assert hasattr(vmd_file, "bone_keyframes"), (
            "VMD file missing bone_keyframes property"
        )
        assert hasattr(vmd_file, "morph_keyframes"), (
            "VMD file missing morph_keyframes property"
        )
        assert hasattr(vmd_file, "camera_keyframes"), (
            "VMD file missing camera_keyframes property"
        )
        assert hasattr(vmd_file, "light_keyframes"), (
            "VMD file missing light_keyframes property"
        )
        assert hasattr(vmd_file, "shadow_keyframes"), (
            "VMD file missing shadow_keyframes property"
        )
        assert hasattr(vmd_file, "property_keyframes"), (
            "VMD file missing property_keyframes property"
        )

        # Verify bone keyframes have required properties
        for bone in vmd_file.bone_keyframes:
            assert hasattr(bone, "bone_name"), (
                "Bone keyframe missing bone_name property"
            )
            assert hasattr(bone, "frame_number"), (
                "Bone keyframe missing frame_number property"
            )
            assert hasattr(bone, "position"), "Bone keyframe missing position property"
            assert hasattr(bone, "rotation"), "Bone keyframe missing rotation property"

        # Verify morph keyframes have required properties
        for morph in vmd_file.morph_keyframes:
            assert hasattr(morph, "morph_name"), (
                "Morph keyframe missing morph_name property"
            )
            assert hasattr(morph, "frame_number"), (
                "Morph keyframe missing frame_number property"
            )
            assert hasattr(morph, "weight"), "Morph keyframe missing weight property"

        # Verify camera keyframes have required properties
        for camera in vmd_file.camera_keyframes:
            assert hasattr(camera, "frame_number"), (
                "Camera keyframe missing frame_number property"
            )
            assert hasattr(camera, "distance"), (
                "Camera keyframe missing distance property"
            )
            assert hasattr(camera, "position"), (
                "Camera keyframe missing position property"
            )
            assert hasattr(camera, "rotation"), (
                "Camera keyframe missing rotation property"
            )

        # Verify light keyframes have required properties
        for light in vmd_file.light_keyframes:
            assert hasattr(light, "frame_number"), (
                "Light keyframe missing frame_number property"
            )
            assert hasattr(light, "color"), "Light keyframe missing color property"
            assert hasattr(light, "direction"), (
                "Light keyframe missing direction property"
            )

        # Verify shadow keyframes have required properties
        for shadow in vmd_file.shadow_keyframes:
            assert hasattr(shadow, "frame_number"), (
                "Shadow keyframe missing frame_number property"
            )
            assert hasattr(shadow, "mode"), "Shadow keyframe missing mode property"
            assert hasattr(shadow, "distance"), (
                "Shadow keyframe missing distance property"
            )

        # Verify property/IK keyframes have required properties
        for prop in vmd_file.property_keyframes:
            assert hasattr(prop, "frame_number"), (
                "Property keyframe missing frame_number property"
            )
            assert hasattr(prop, "visible"), (
                "Property keyframe missing visible property"
            )
            assert hasattr(prop, "ik_states"), (
                "Property keyframe missing ik_states property"
            )
