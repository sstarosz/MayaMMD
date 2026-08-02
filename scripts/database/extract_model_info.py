"""
This script loads model from asset folder and extracts information about it in json format.
This is used to create a database of models that can be used in the future for testing, statistics, etc.

Mainly used for testing purposes, but can be useful for other things as well.
For testing to test:
- If the model can be loaded in Maya without errors.
- If new features are working on old models.
- If new features do not break old models.
"""

import os
import sys

# This script is inside scripts/database folder, so we need to go two levels up to access the assets folder and the mmd package
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT_DIR)

from assets.assets_utils import get_all_pmx_model_paths
from mmd.core.pmx_importer import dump_pmx_to_json, parse_pmx

ASSETS_DIR = os.path.join(ROOT_DIR, "assets")
MODELS_DIR = os.path.join(ASSETS_DIR, "models")

#
EXPORTS_DIR = os.path.join(ASSETS_DIR, "models_database")
os.makedirs(EXPORTS_DIR, exist_ok=True)


if __name__ == "__main__":
    # Get all model paths from assets_utils
    MODELS_PATHS = get_all_pmx_model_paths()

    for model_path in MODELS_PATHS:
        # Get relative path from MODELS_DIR
        rel_path = os.path.relpath(model_path, MODELS_DIR)

        # Get directory structure (e.g., "GirlsFrontline/TololoDefault")
        rel_dir = os.path.dirname(rel_path)
        model_name = os.path.splitext(os.path.basename(model_path))[0]

        # Create export directory structure - this is the final export path
        export_path = os.path.join(EXPORTS_DIR, rel_dir)
        os.makedirs(export_path, exist_ok=True)

        # Export model data to json
        print(f"Processing model: {model_name} -> {export_path}")
        model_data = parse_pmx(model_path)
        dump_pmx_to_json(model_data, export_path)
