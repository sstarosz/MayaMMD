# Poses

Place your `.vpd` pose files here.

## Supported format

- VPD (Vocaloid Pose Data) — MikuMikuDance pose format

## File discovery

After adding files, regenerate the file list:

```bash
python scripts/database/generate_file_list.py
```

This updates `assets/vpd_pose_files.py` so tests and benchmarks can discover your poses.
