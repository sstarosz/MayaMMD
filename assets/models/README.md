# Models

Place your `.pmx` model files here.

## Supported format

- PMX (Polygon Model eXtended) — MikuMikuDance model format

## How to get models

- Export from MikuMikuDance (MMD)
- Download from model sharing communities (e.g., BowlRoll, NicoNico)
- Convert from other formats using tools like PMX Editor

## File discovery

After adding files, regenerate the file list:

```bash
python scripts/database/generate_file_list.py
```

This updates `assets/pmx_model_files.py` so tests and benchmarks can discover your models.
