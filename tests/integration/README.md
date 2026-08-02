# PMX Integration Tests

Integration tests for PMX → Maya import functionality. Tests run against real PMX model files using Maya's Python API in standalone mode.

## Structure

```
tests/integration/
├── test_helpers.py                    # Core test infrastructure
└── maya/
    ├── run_all_integration_tests.py  # Main test runner (run all suites)
    ├── test_model_context_integration.py # ModelContext + scene-discovery tests (no PMX required)
    ├── test_multi_import_integration.py # Multi-model import / naming-collision tests
    ├── test_pmx_import_integration.py   # General import tests (root, geometry, mesh, materials, skin)
    ├── test_pmx_bone_integration.py     # Bone-specific tests (hierarchy, IK, constraints)
    ├── test_pmx_morph_integration.py    # Morph-specific tests (blendshapes)
    ├── test_vmd_integration.py          # VMD animation integration tests
    ├── test_vpd_integration.py          # VPD pose integration tests
    ├── nodes/
    │   ├── test_bone_morph_node_integration.py   # Tests for boneMorphNode (no PMX required)
    │   └── test_ccd_ik_solver_node_integration.py # Tests for CCD IK solver (no PMX required)
    └── cmds/
        └── test_bone_blend_shape_cmd_integration.py  # Tests for boneBlendShape command (no PMX required)
```

## Requirements

- **Autodesk Maya 2026** (or compatible version)
- Maya Python interpreter (`mayapy.exe`)
- PMX model files in `assets/models/` directory

## Running Tests

### Run All Tests

```bash
"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe" tests/integration/maya/run_all_integration_tests.py

# Or use the convenience script:
python tests/integration/run_integration_tests.py all
```

### Run Individual Test Suites

```bash
# General import tests only
"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe" tests/integration/maya/test_pmx_import_integration.py
# Or: python tests/integration/run_integration_tests.py import

# Bone tests only
"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe" tests/integration/maya/test_pmx_bone_integration.py
# Or: python tests/integration/run_integration_tests.py bone

# Morph tests only
"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe" tests/integration/maya/test_pmx_morph_integration.py
# Or: python tests/integration/run_integration_tests.py morph

# Multi-model import tests
# Or: python tests/integration/run_integration_tests.py multi

# VMD animation tests
python tests/integration/run_integration_tests.py vmd

# VPD pose tests
python tests/integration/run_integration_tests.py vpd

# ModelContext + scene-discovery tests (no PMX required)
python tests/integration/run_integration_tests.py context

# BoneMorphNode only (no PMX required)
"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe" tests/integration/maya/nodes/test_bone_morph_node_integration.py
# Or: python tests/integration/run_integration_tests.py node

# CCD IK Solver Node only (no PMX required)
"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe" tests/integration/maya/nodes/test_ccd_ik_solver_node_integration.py
# Or: python tests/integration/run_integration_tests.py ccd

# BoneBlendShapeCmd only (no PMX required)
"C:\Program Files\Autodesk\Maya2026\bin\mayapy.exe" tests/integration/maya/cmds/test_bone_blend_shape_cmd_integration.py
# Or: python tests/integration/run_integration_tests.py cmd
```

### BoneMorphNode Tests (No PMX Required)

The `node` test suite tests the `boneMorphNode` custom Maya dependency graph node **without requiring PMX models**. These tests:

- Use a minimal plugin loader (only BoneMorphNode, no full MayaMMD dependencies)
- Test the node directly via Maya API and direct attribute manipulation
- Don't depend on BoneBlendShapeCmd - fully independent testing
- Verify quaternion SLERP rotation blending
- Verify linear translation blending
- Verify multi-target additive blending behavior

This suite runs independently and is much faster than PMX-based tests since it doesn't parse any model files.

### BoneBlendShapeCmd Tests (No PMX Required)

The `cmd` test suite tests the `boneBlendShape` Maya command **without requiring PMX models**. These tests:

- Load both BoneMorphNode and BoneBlendShapeCmd plugins
- Test command syntax and argument parsing
- Test query operations (listTargets, targetData)
- Test edit operations (addTarget with automatic connection setup)
- Verify weight alias creation
- Verify plusMinusAverage node creation and connection
- Test undo/redo functionality
- Test multiple targets and multi-joint scenarios

This suite validates the high-level command interface that simplifies bone morph setup and is used by the PMX importer.

```

## Configuration

All test configuration is done **in code** (no command-line arguments needed).

### Filtering Models

Edit `TEST_CONFIG` in `run_all_integration_tests.py`:

```python
TEST_CONFIG = {
    "enabled_suites": ["import", "bone", "morph", "node", "cmd"],  # Which test suites to run
    "model_filter": None,  # None = test all models
}
```

#### Example: Test specific models by folder name

```python
from tests.integration.test_helpers import create_model_folder_filter

TEST_CONFIG = {
    "enabled_suites": ["import", "bone", "morph"],
    "model_filter": create_model_folder_filter("Acacia", "Fanny"),
}
```

#### Example: Test only Snowbreak models

```python
from tests.integration.test_helpers import create_model_path_pattern_filter

TEST_CONFIG = {
    "enabled_suites": ["import", "bone", "morph"],
    "model_filter": create_model_path_pattern_filter("SnowbreakContainmentZone"),
}
```

#### Example: Combine multiple filters

```python
from tests.integration.test_helpers import (
    create_model_folder_filter,
    create_model_path_pattern_filter,
    combine_filters,
)

# Test Snowbreak models, but only Acacia and Fanny
snowbreak_filter = create_model_path_pattern_filter("SnowbreakContainmentZone")
folder_filter = create_model_folder_filter("Acacia", "Fanny")

TEST_CONFIG = {
    "enabled_suites": ["bone"],  # Only bone tests
    "model_filter": combine_filters(snowbreak_filter, folder_filter),
}
```

### Selecting Test Suites

Control which test suites run by editing `enabled_suites`:

```python
# Run all test suites
TEST_CONFIG = {
    "enabled_suites": ["import", "bone", "morph", "node"],
    # ...
}

# Run only bone tests
TEST_CONFIG = {
    "enabled_suites": ["bone"],
    # ...
}

# Run import and morph tests (skip bones)
TEST_CONFIG = {
    "enabled_suites": ["import", "morph"],
    # ...
}
```

## Available Filter Functions

All filter functions are in `tests/integration/test_helpers.py`:

| Function                                    | Description                     | Example                                               |
| ------------------------------------------- | ------------------------------- | ----------------------------------------------------- |
| `create_model_name_filter(*names)`          | Match by filename               | `create_model_name_filter("Acacia.pmx", "Fanny.pmx")` |
| `create_model_folder_filter(*folders)`      | Match by parent folder          | `create_model_folder_filter("Acacia", "Fanny")`       |
| `create_model_path_pattern_filter(pattern)` | Match by substring in full path | `create_model_path_pattern_filter("Snowbreak")`       |
| `combine_filters(*filters)`                 | Combine filters with AND logic  | `combine_filters(filter1, filter2)`                   |

## Writing New Tests

### 1. Add test function

Test functions must have this signature:

```python
def test_my_feature(
    pmx_data: PmxModel, maya_pmx_data
) -> bool:  # test_my_feature is an example name, use descriptive names for your tests
    """Test description."""
    # ... test implementation ...

    if test_passes:
        print("PASS: Test passed description")
        return True
    else:
        print("FAIL: Test failed description")
        return False
```

### 2. Register test in `_TESTS` list

```python
_TESTS = [
    ("Test Display Name", test_my_feature),
    # ... other tests ...
]
```

### 3. Test runs automatically

The test will automatically:
- Load each model once
- Run your test function
- Collect results
- Print summary report

## Test Output

Tests use compact output with color coding:

```
──────────────────────────────────────────────────────────
  PMX Bone Integration Tests
──────────────────────────────────────────────────────────

Acacia/Acacia.pmx
  ✓ Bone Group Under Root
  ✓ Joint Count + Valid MObjects
  ✓ Joint World Positions
  ...
```

Failed tests show error details inline next to the test name.

## Debugging Individual Models

To debug a specific model in an individual test suite, modify the test file directly:

```python
# In test_pmx_bone_integration.py (or any test file)

# Comment out the auto-generated model list
# TESTING_MODELS = get_all_pmx_model_paths()

# Use a custom list for debugging
TESTING_MODELS = [
    r"c:\Users\Sebastian\Desktop\MayaMMD\assets\models\HatsuneMiku\HatsuneMiku.pmx"
]
```

Then run that specific test file with mayapy.

## Framework Components

### test_helpers.py

Core infrastructure providing:
- `run_test_suite()` - Main test runner
- `run_main()` - Standard entry point
- `TestResult` - Result container
- `color_text()` - Console coloring
- `setup_test_environment()` - Clean Maya scene
- `load_plugin()` - Plugin loading
- Model filtering utilities

### Individual Test Files

Each test file follows this pattern:

```python
# 1. Maya standalone initialization
import maya.standalone
maya.standalone.initialize()

# 2. Maya imports (after initialization!)
import maya.api.OpenMaya as om
import maya.cmds as cmds

# 3. Project imports
from assets.assets_utils import get_all_pmx_model_paths
from mmd.core.pmx_importer import parse_pmx
from mmd.maya.pmx_scene_builder import build_pmx_scene
from tests.integration.test_helpers import run_main, run_test_suite

# 4. Define which models to test
TESTING_MODELS = get_all_pmx_model_paths()

# 5. Define test functions
def test_something(pmx_data, maya_pmx_data) -> bool:
    # ...

# 6. Register tests
_TESTS = [
    ("Test Name", test_something),
]

# 7. Define runner
def run_all_tests() -> bool:
    return run_test_suite(
        suite_name="My Test Suite",
        tests=_TESTS,
        testing_models=TESTING_MODELS,
        parse_fn=parse_pmx,
        build_fn=build_pmx_scene,
    )

# 8. Entry point
if __name__ == "__main__":
    run_main(run_all_tests)
```

## Best Practices

1. **One model load per test run** - Tests share the same loaded model data
2. **Fast failures** - Tests return immediately on first failure
3. **Descriptive output** - Print clear PASS/FAIL messages
4. **No external dependencies** - Only use Maya's built-in Python (no pytest, etc.)
5. **Clean test isolation** - Each model gets a fresh Maya scene
6. **Compact output during run** - One line per test
7. **Full error details in summary** - Nothing is lost

## Troubleshooting

### Maya Import Errors

Make sure Maya standalone is initialized **before** importing any `maya.*` modules:

```python
import maya.standalone

maya.standalone.initialize()

# Now safe to import Maya modules
import maya.cmds as cmds
```

### Plugin Load Failures

Check that `mmd/MayaMMD.mll` exists and is the compiled C++ entry point.

### Model Not Found

Verify PMX files exist in `assets/models/` and are listed in the generated file list.

### Unicode Errors (Windows Console)

The framework handles Unicode model names automatically with fallback to ASCII representation.
