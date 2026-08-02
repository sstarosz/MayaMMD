import os

import pytest

# Use robust, location-independent path resolution
from assets.assets_utils import get_all_pmx_model_paths, is_lfs_pointer_file
from mmd.core.pmx_importer import parse_pmx

TEST_MODEL_FILES = get_all_pmx_model_paths()


@pytest.mark.parametrize("model_path", TEST_MODEL_FILES)
def test_parse_pmx_loads_model(model_path, caplog):
    assert os.path.exists(model_path), f"Model file does not exist: {model_path}"

    if is_lfs_pointer_file(model_path):
        pytest.skip(f"PMX file is a Git LFS pointer (run 'git lfs pull'): {model_path}")

    with caplog.at_level("WARNING"):
        try:
            model = parse_pmx(model_path)
        except Exception as e:
            pytest.fail(f"parse_pmx failed for {model_path}: {e}")

        warnings = [
            record for record in caplog.records if record.levelname >= "WARNING"
        ]
        if warnings:
            print(f"Warnings while parsing {model_path}:")
            for warning in warnings:
                print(f"  - {warning.message}")

        assert model is not None, f"parse_pmx returned None for {model_path}"
        # Optionally, check some basic properties
        assert hasattr(model, "header"), "Model missing header property"
        assert hasattr(model, "vertices"), "Model missing vertices property"
