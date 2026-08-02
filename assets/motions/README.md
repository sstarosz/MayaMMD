# Motions

Place your `.vmd` motion files here.

## Supported format

- VMD (Vocaloid Motion Data) — MikuMikuDance animation format

## File discovery

After adding files, regenerate the file list:

```bash
python scripts/database/generate_file_list.py
```

This updates `assets/vmd_motion_files.py` so tests and benchmarks can discover your motions.
