"""
This script traverses the "assets/models_database" directory and minifies all JSON files by removing unnecessary whitespace and ensuring UTF-8 encoding.
This helps reduce file size while maintaining the integrity of the JSON data.
Note: This script is intended for use before publishing new versions of the models database
"""

import json
import os

# This script is inside scripts/database folder, so we need to go two levels up to ROOT_DIR
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODELS_DATABASE_DIR = os.path.join(ROOT_DIR, "assets", "models_database")
MOTIONS_DATABASE_DIR = os.path.join(ROOT_DIR, "assets", "motions_database")
POSE_DATABASE_DIR = os.path.join(ROOT_DIR, "assets", "poses_database")


# Minify models database
for dirpath, _, filenames in os.walk(MODELS_DATABASE_DIR):
    for filename in filenames:
        if filename.endswith(".json"):
            file_path = os.path.join(dirpath, filename)
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                with open(file_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, separators=(",", ":"), ensure_ascii=False)
                print(f"Minified: {file_path}")
            except Exception as e:
                print(f"Error minifying {file_path}: {e}")

# Minify motions database
for dirpath, _, filenames in os.walk(MOTIONS_DATABASE_DIR):
    for filename in filenames:
        if filename.endswith(".json"):
            file_path = os.path.join(dirpath, filename)
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                with open(file_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, separators=(",", ":"), ensure_ascii=False)
                print(f"Minified: {file_path}")
            except Exception as e:
                print(f"Error minifying {file_path}: {e}")

# Minify poses database
for dirpath, _, filenames in os.walk(POSE_DATABASE_DIR):
    for filename in filenames:
        if filename.endswith(".json"):
            file_path = os.path.join(dirpath, filename)
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                with open(file_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, separators=(",", ":"), ensure_ascii=False)
                print(f"Minified: {file_path}")
            except Exception as e:
                print(f"Error minifying {file_path}: {e}")
