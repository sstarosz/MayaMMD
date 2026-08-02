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

from assets.assets_utils import get_all_vmd_paths
from mmd.core.vmd_importer import dump_vmd_to_json, parse_vmd_file

ASSETS_DIR = os.path.join(ROOT_DIR, "assets")
MOTION_DIR = os.path.join(ASSETS_DIR, "motions")

#
EXPORTS_DIR = os.path.join(ASSETS_DIR, "motions_database")
os.makedirs(EXPORTS_DIR, exist_ok=True)


if __name__ == "__main__":
    # Get all motion paths from assets_utils
    MOTION_PATHS = get_all_vmd_paths()

    for motion_path in MOTION_PATHS:
        # Get relative path from MOTION_DIR
        rel_path = os.path.relpath(motion_path, MOTION_DIR)

        # Get directory structure (e.g., subfolder if any)
        rel_dir = os.path.dirname(rel_path)
        motion_name = os.path.splitext(os.path.basename(motion_path))[0]

        # Create export directory structure - include motion name as subfolder
        if rel_dir:
            export_path = os.path.join(EXPORTS_DIR, rel_dir, motion_name)
        else:
            export_path = os.path.join(EXPORTS_DIR, motion_name)
        os.makedirs(export_path, exist_ok=True)

        # Export motion data to json
        print(f"Processing motion: {motion_name} -> {export_path}")
        motion_data = parse_vmd_file(motion_path)
        dump_vmd_to_json(motion_data, export_path)
