"""
This script loads VPD pose files from the assets folder and extracts
information about them in JSON format.  This is used to create a database
of poses that can be used in the future for testing, statistics, etc.

The JSON dumps are written to assets/poses_database/ mirroring the same
sub-folder structure as the source poses under assets/poses/.
"""

import os
import sys

# This script is inside scripts/database folder, so we need to go two levels up
# to access the assets folder and the mmd package
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT_DIR)

from assets.assets_utils import get_all_vpd_paths
from mmd.core.vpd_importer import dump_vpd_to_json, parse_vpd_file

ASSETS_DIR = os.path.join(ROOT_DIR, "assets")
POSES_DIR = os.path.join(ASSETS_DIR, "poses")

EXPORTS_DIR = os.path.join(ASSETS_DIR, "poses_database")
os.makedirs(EXPORTS_DIR, exist_ok=True)


if __name__ == "__main__":
    POSE_PATHS = get_all_vpd_paths()

    for pose_path in POSE_PATHS:
        # Get relative path from POSES_DIR
        rel_path = os.path.relpath(pose_path, POSES_DIR)

        # Get directory structure (e.g., "Pose Pack #15")
        rel_dir = os.path.dirname(rel_path)
        pose_name = os.path.splitext(os.path.basename(pose_path))[0]

        # Create export directory structure — include pose name as subfolder
        if rel_dir:
            export_path = os.path.join(EXPORTS_DIR, rel_dir, pose_name)
        else:
            export_path = os.path.join(EXPORTS_DIR, pose_name)
        os.makedirs(export_path, exist_ok=True)

        # Export pose data to JSON
        print(f"Processing pose: {pose_name} -> {export_path}")
        pose_data = parse_vpd_file(pose_path)
        dump_vpd_to_json(pose_data, export_path)
