"""
Script to scan all .pmx files in the assets/models directory and generate a Python file
(assets/pmx_model_files.py) containing a list of all model paths (relative to assets/models).

Run this script whenever you add or remove models to keep the list up to date.
The generated Python file can be imported and used directly in tests or other scripts.
"""

import os

# This script is inside scripts/database folder, so we need to go two levels up to ROOT_DIR
_SCRIPT_DIR = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)

ASSETS_MODELS_DIR = os.path.join(_SCRIPT_DIR, "assets", "models")

MOTION_DIR = os.path.join(_SCRIPT_DIR, "assets", "motions")

POSES_DIR = os.path.join(_SCRIPT_DIR, "assets", "poses")

OUTPUT_PMX_PATH = os.path.join(
    _SCRIPT_DIR,
    "assets",
    "pmx_model_files.py",
)

OUTPUT_VMD_PATH = os.path.join(
    _SCRIPT_DIR,
    "assets",
    "vmd_motion_files.py",
)

OUTPUT_VPD_PATH = os.path.join(
    _SCRIPT_DIR,
    "assets",
    "vpd_pose_files.py",
)


def scan_directory_for_files(root_dir, extension, base_dir=None) -> list[str]:
    """
    Scan a directory for files with a specific extension.

    Args:
        root_dir: Directory to scan
        extension: File extension to filter (e.g., '.pmx', '.vmd', '.vpd')
        base_dir: Base directory for relative paths (defaults to root_dir)

    Returns:
        List of relative file paths
    """
    if base_dir is None:
        base_dir = root_dir

    files_list = []
    for root, _, files in os.walk(root_dir):
        for file in files:
            if file.lower().endswith(extension):
                full_path = os.path.join(root, file)
                files_list.append(os.path.relpath(full_path, base_dir))

    return files_list


def save_list_to_python_file(file_list, output_path, variable_name):
    # Write the Python file
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("# This file is auto-generated. Do not edit by hand.\n")
        f.write(
            "# Contains a list of all PMX model files (relative to assets/models)\n\n"
        )
        f.write("ALL_{}S = [\n".format(variable_name.upper()))
        for path in file_list:
            # Replace backslashes with forward slashes to avoid escape sequence issues
            normalized_path = path.replace("\\", "/")
            f.write(f'    "{normalized_path}",\n')
        f.write("]\n")
        f.write("\n")
        f.write("\n# This file is auto-generated. Do not edit by hand.\n")


pmx_files = scan_directory_for_files(ASSETS_MODELS_DIR, ".pmx", ASSETS_MODELS_DIR)
vmd_files = scan_directory_for_files(MOTION_DIR, ".vmd", MOTION_DIR)
vpd_files = scan_directory_for_files(POSES_DIR, ".vpd", POSES_DIR)

save_list_to_python_file(pmx_files, OUTPUT_PMX_PATH, "PMX")
save_list_to_python_file(vmd_files, OUTPUT_VMD_PATH, "VMD")
save_list_to_python_file(vpd_files, OUTPUT_VPD_PATH, "VPD")
