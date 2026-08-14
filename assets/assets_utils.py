import logging
import os
from typing import List

# The generated file lists (pmx_model_files.py / vmd_motion_files.py /
# vpd_pose_files.py) are gitignored — regenerate them locally with
# `scripts/database/generate_file_list.py`.  Fall back to empty lists so the
# helpers keep working (and tests skip) on a fresh checkout.
try:
    from .pmx_model_files import ALL_PMXS
except ImportError:
    ALL_PMXS: list[str] = []

try:
    from .vmd_motion_files import ALL_VMDS
except ImportError:
    ALL_VMDS: list[str] = []

try:
    from .vpd_pose_files import ALL_VPDS
except ImportError:
    ALL_VPDS: list[str] = []

_log = logging.getLogger(__name__)

# Git LFS pointer files start with this header
_LFS_POINTER_HEADER = b"version https://git-lfs.github.com"


def is_lfs_pointer_file(file_path: str) -> bool:
    """
    Returns True if the file is a Git LFS pointer (not the actual binary content).
    LFS pointer files are small text files starting with 'version https://git-lfs.github.com'.
    """
    try:
        with open(file_path, "rb") as f:
            header = f.read(len(_LFS_POINTER_HEADER))
        return header == _LFS_POINTER_HEADER
    except OSError:
        return False


def _filter_lfs_pointers(paths: List[str]) -> List[str]:
    """Filter out files that are Git LFS pointers, returning only real binary files."""
    real_files = []
    for p in paths:
        if os.path.exists(p) and not is_lfs_pointer_file(p):
            real_files.append(p)
    return real_files


# This function returns absolute paths to all PMX models, regardless of where the project is located.
def get_all_pmx_model_paths() -> List[str]:
    """
    Returns a list of absolute paths to all PMX model files.

    Returns an empty list if the models directory is not present.
    See assets/models/README.md for setup instructions.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(here, "models")

    if not os.path.isdir(models_dir):
        _log.debug("Models directory not found — see assets/models/README.md.")
        return []

    return [os.path.join(models_dir, rel_path) for rel_path in ALL_PMXS]


def get_all_vmd_paths() -> List[str]:
    """
    Returns a list of absolute paths to all VMD motion files.

    Returns an empty list if the motions directory is not present.
    See assets/motions/README.md for setup instructions.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    motions_dir = os.path.join(here, "motions")

    if not os.path.isdir(motions_dir):
        _log.debug("Motions directory not found — see assets/motions/README.md.")
        return []

    return [os.path.join(motions_dir, rel_path) for rel_path in ALL_VMDS]


def get_all_vpd_paths() -> List[str]:
    """
    Returns a list of absolute paths to all VPD pose files.

    Returns an empty list if the poses directory is not present.
    See assets/poses/README.md for setup instructions.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    poses_dir = os.path.join(here, "poses")

    if not os.path.isdir(poses_dir):
        _log.debug("Poses directory not found — see assets/poses/README.md.")
        return []

    return [os.path.join(poses_dir, rel_path) for rel_path in ALL_VPDS]
