import os

import pytest

# Use robust, location-independent path resolution
from assets.assets_utils import get_all_vpd_paths, is_lfs_pointer_file
from mmd.core.vpd_importer import parse_vpd_file

TEST_VPD_FILES = get_all_vpd_paths()


@pytest.mark.parametrize("vpd_path", TEST_VPD_FILES)
def test_parse_vpd_loads_pose(vpd_path, caplog):
    """Test that each VPD pose file can be successfully parsed."""
    assert os.path.exists(vpd_path), f"VPD file does not exist: {vpd_path}"

    if is_lfs_pointer_file(vpd_path):
        pytest.skip(f"VPD file is a Git LFS pointer (run 'git lfs pull'): {vpd_path}")

    with caplog.at_level("WARNING"):
        try:
            vpd_file = parse_vpd_file(vpd_path)
        except Exception as e:
            pytest.fail(f"parse_vpd_file failed for {vpd_path}: {e}")

        warnings = [
            record for record in caplog.records if record.levelname >= "WARNING"
        ]
        if warnings:
            print(f"Warnings while parsing {vpd_path}:")
            for warning in warnings:
                print(f"  - {warning.message}")

        assert vpd_file is not None, f"parse_vpd_file returned None for {vpd_path}"

        # Check basic properties
        assert hasattr(vpd_file, "header"), "VPD file missing header property"
        assert hasattr(vpd_file, "model_name"), "VPD file missing model_name property"
        assert hasattr(vpd_file, "bone_count"), "VPD file missing bone_count property"
        assert hasattr(vpd_file, "bones"), "VPD file missing bones property"

        # Verify bone count matches parsed bones
        assert len(vpd_file.bones) == vpd_file.bone_count, (
            f"Expected {vpd_file.bone_count} bones, but parsed {len(vpd_file.bones)}"
        )

        # Verify each bone has required properties
        for bone in vpd_file.bones:
            assert hasattr(bone, "bone_idx"), "Bone missing bone_idx property"
            assert hasattr(bone, "bone_name"), "Bone missing bone_name property"
            assert hasattr(bone, "position"), "Bone missing position property"
            assert hasattr(bone, "rotation"), "Bone missing rotation property"
