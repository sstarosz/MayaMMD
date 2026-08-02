"""
This script formats all JSON files in the models_database directory with proper indentation and UTF-8 encoding.
It ensures that the JSON files are human-readable and consistently formatted.
Note: Json files are not formatted to reduce file size, this script reformats them for better readability.
"""

import json
import os

# This script is inside scripts/database folder, so we need to go two levels up to ROOT_DIR
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODELS_DATABASE_DIR = os.path.join(ROOT_DIR, "assets", "models_database")

for dirpath, _, filenames in os.walk(MODELS_DATABASE_DIR):
    for filename in filenames:
        if filename.endswith(".json"):
            file_path = os.path.join(dirpath, filename)
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                with open(file_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=4, ensure_ascii=False)
                print(f"Formatted: {file_path}")
            except Exception as e:
                print(f"Error formatting {file_path}: {e}")
