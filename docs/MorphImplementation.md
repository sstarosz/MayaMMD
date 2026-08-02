# PMX Morph Support — Implementation Notes

## Overview

This document describes the design and implementation of PMX morph support
in the Maya plugin.  Two morph types are currently functional (read-only
weight manipulation); the remaining types are parsed from PMX files but not
yet wired to Maya nodes.

| Type       | Enum | Status | Implementation                             |
| ---------- | ---- | ------ | ------------------------------------------ |
| `GROUP`    | 0    | ❌      | Not implemented                            |
| `VERTEX`   | 1    | ✅      | Maya `blendShape` deformer                 |
| `BONE`     | 2    | ✅      | Custom `boneMorphNode` MPxNode (read-only) |
| `UV`       | 3    | ❌      | Not implemented                            |
| `UV1`      | 4    | ❌      | Not implemented                            |
| `UV2`      | 5    | ❌      | Not implemented                            |
| `UV3`      | 6    | ❌      | Not implemented                            |
| `UV4`      | 7    | ❌      | Not implemented                            |
| `MATERIAL` | 8    | ❌      | Not implemented                            |

> **Meta issue:** [#25 Morph system](https://github.com/sstarosz/MayaMMD/issues/25)
> tracks all morph-related work.

---

## Architecture

Morph support is split across two independent systems that share a common
UI layer:

| System        | Maya node               | Morph data stored on                             |
| ------------- | ----------------------- | ------------------------------------------------ |
| Vertex morphs | `blendShape` deformer   | Deformer weight plugs + `pmxMorphMapping` attr   |
| Bone morphs   | `boneMorphNode` MPxNode | `aMorphTargets` compound array + `aWeight` multi |

Both are exposed uniformly through the `MorphTreeWidget` via the
`_AbstractMorphSource` protocol (see [UI Components](#ui-components)).

### Bone Morph Data Flow

1. PMX bone morph data is parsed into `MorphBone` entries (bone index,
   position offset, rotation quaternion).
2. `create_bone_morph_node()` creates a `boneMorphNode` instance and stores
   target data on its `aMorphTargets` compound array.
3. `boneBlendShape -addTarget` stores each target, creates a weight entry
   (`aWeight` multi), and sets up:
   - **MORPH_ controller transforms** — DAG transform nodes inserted between
     each affected joint parent and the joint itself (same pattern as
     `INH_` controllers for INHERIT_ROTATION).
   - **DG connections** — `boneMorphNode.outputRotate[N]` →
     `MORPH_controller.rotate`, and `.outputTranslate[N]` →
     `MORPH_controller.translate` (for bones with non-zero position offsets).
   - **INH_ rewiring** — if the joint has INHERIT_ROTATION, the
     `multiplyDivide` input reads from the MORPH_ controller instead of the
     original joint.
4. The Morph Tree widget exposes weight sliders — each slider sets
   `boneMorphNode.weight[targetIndex]`.
5. DG propagation computes the blended output, driving the MORPH_
   controllers; main joints inherit rotation/translation through the DAG.
6. Mesh remains skinned to the original joints (not MORPH_ controllers).

---

## Implementation Details

### Vertex Morphs (VERTEX)

Uses Maya native `blendShape` deformer — the same system as the Shape
Editor.  Typical use: face expressions ("Smile", "Blink", "Angry").

- Created in `mmd/maya/pmx/morph_builder.py` →
  `create_blendshapes_from_vertex_morphs()`.
- For each morph target a duplicate mesh is created with offset vertices
  (Z-flipped for Maya right-handed coordinate system), then fed into a
  `blendShape` node.  Temporary meshes are deleted after the deformer is
  built.
- BlendShape is placed **before** `skinCluster` in the deformation chain.
- Morph-name mapping is stored in a `pmxMorphMapping` compound attribute
  on the blendShape node (PMX name → Maya alias pairs).

---

### Bone Morphs (BONE)

Skeletal pose corrections ("T-Pose", "A-Pose", "ElbowBlend",
"ShoulderBlend").  Applied as **additive offsets** on top of the current
skeleton pose, blended by weight.

#### Component Map

| Component                | File                                                         | Role                                                                                                                      |
| ------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| `boneMorphNode`          | `mmd/maya/nodes/bone_morph_node.py`                          | Custom MPxNode. Takes morph weight inputs, outputs blended rotation (Euler XYZ, degrees) and translation offsets per bone |
| `boneBlendShape` command | `mmd/maya/cmds/bone_blend_shape_cmd.py`                      | Maya command to create/add bone morph targets, build DG connections, create MORPH_ controller transforms                  |
| Morph builder            | `mmd/maya/pmx/morph_builder.py` → `create_bone_morph_node()` | Orchestrates creation: parses PMX bone morphs → calls `boneBlendShape -addTarget` for each → wires up outputs             |

#### Data Structure

Bone morph target data is stored on `boneMorphNode.aMorphTargets`:

```json
{
  "name_local": "T-Pose",
  "name_universal": "T-Pose",
  "panel_type": 4,
  "morph_type": "BONE",
  "data": [
    {
      "bone_index": 30,
      "position_offset": [0.0, 0.0, 0.0],
      "rotation_offset": [0.0, 0.0, 0.04361938685178757, 0.9990482330322266]
    },
    {
      "bone_index": 32,
      "position_offset": [0.0, 0.0, 0.0],
      "rotation_offset": [0.0, 0.0, 0.258819043636322, 0.9659258127212524]
    }
  ]
}
```

> **Note:** `rotation_offset` is a quaternion (x, y, z, w).

#### Current Limitations

| Feature                     | Status     | Notes                                                                                                    |
| --------------------------- | ---------- | -------------------------------------------------------------------------------------------------------- |
| Create targets at import    | ✅ Done     | All PMX BONE morphs created during `build_pmx_scene()`                                                   |
| Weight sliders in UI        | ✅ Done     | Via `MorphTreeWidget`                                                                                    |
| DG-driven updates           | ✅ Done     | Real-time via `boneMorphNode.compute()`                                                                  |
| Delete individual targets   | ❌ Deferred | Not exposed                                                                                              |
| Edit target data            | ❌ Deferred | No edit UI or command                                                                                    |
| Add new targets post-import | ❌ Deferred | Command supports syntax, but UI not exposed                                                              |
| Naming manager integration  | ❌ Deferred | `PMXNamingManager` does not yet generate names for `boneMorphNode` or MORPH_ controllers. Separate task. |

---

### Group Morphs (GROUP)

Meta-morphs that drive other morphs by weight.  Would require a composite
node or chained `boneMorphNode` inputs.  Parsed from PMX but not wired to
any Maya node.

---

### UV Morphs (UV, UV1–UV4)

Per-vertex texture-coordinate offsets across up to five UV layers.  Parsed
from PMX; could use additional `blendShape` deformers targeting UV
attributes in the future.

---

### Material Morphs (MATERIAL)

Dynamic material-property changes: diffuse, specular, ambient, edge colour,
edge size, and texture tints.  Parsed from PMX; would require shader
connections or a custom MPxNode for blend operations.

---

## UI Components

The `MorphTreeWidget` (`mmd/ui/morph_tree_widget.py`) is the primary UI
for inspecting and manipulating morph weights.  It is built around the
`_AbstractMorphSource` protocol so new morph types can be added without
modifying the tree-widget code.

### Widget Architecture

| Class                    | Role                                                                    |
| ------------------------ | ----------------------------------------------------------------------- |
| `_AbstractMorphSource`   | Protocol for a Maya node that exposes morph-target weights              |
| `_BlendShapeMorphSource` | Wraps a Maya `blendShape` deformer (vertex morphs)                      |
| `_BoneMorphSource`       | Wraps the custom `boneMorphNode` MPxNode (bone morphs)                  |
| `_MorphRow`              | Bundles slider, spinbox, visibility toggle, keyframe dot per target     |
| `MorphTreeWidget`        | Tabbed QTreeWidget with Maya-style tree decorations                     |
| `MayaStyleTreeDelegate`  | Custom QStyledItemDelegate drawing expand/collapse boxes and tree lines |

### Behaviour

- One tab per morph source type ("Vertex", "Bone").
- Each model root is a collapsible parent item with an envelope toggle.
- Each target row contains: visibility toggle, weight slider + spinbox,
  keyframe dot, and a disabled "Edit" button.
- Bidirectional sync with the Maya DG via `MNodeMessage` callbacks keeps
  the widget in sync with external weight changes (Shape Editor, anim
  curves, etc.).
- A `timeChanged` callback updates keyframe-dot styling during playback.
- New morph sources are added by subclassing `_AbstractMorphSource` — no
  tree-widget changes needed.

---

## Key Source Files

| File                                    | Purpose                                                                         |
| --------------------------------------- | ------------------------------------------------------------------------------- |
| `mmd/maya/pmx/morph_builder.py`         | `create_blendshapes_from_vertex_morphs()`, `create_bone_morph_node()`           |
| `mmd/maya/nodes/bone_morph_node.py`     | Custom MPxNode — bone morph solver (quaternion slerp + linear position blend)   |
| `mmd/maya/cmds/bone_blend_shape_cmd.py` | `boneBlendShape` command — addTarget, listTargets, MORPH_ controller creation   |
| `mmd/maya/maya_data_types.py`           | `MayaPmxData` — holds `bone_morph_node_name`, `blend_shape_node_name`           |
| `mmd/maya/model_context.py`             | `ModelContext` — lazy scene queries for bone maps, morph maps, IK handles       |
| `mmd/maya/pmx_model_utils.py`           | Scene discovery utilities — `find_blend_shape_node()`, `find_bone_morph_node()` |
| `mmd/ui/morph_tree_widget.py`           | Morph Tree UI — tabbed weight editor with inline sliders, vis toggles, key dots |
| `mmd/ui/tool_main_widget.py`            | Main UI — refreshes Morph Tree on model import/switch                           |

---

## Quick Reference

### Panel Types

The `panel_type` field on `PMXMorph` determines which MMD panel the morph
appears in.  Panel 0 is reserved/system; panels 1–4 are user-facing.

| Value | Panel   | Japanese Label | Typical contents                        |
| ----- | ------- | -------------- | --------------------------------------- |
| 0     | System  | (hidden)       | Reserved / internal morphs              |
| 1     | Eyebrow | 眉             | Eyebrow expressions                     |
| 2     | Eye     | 目             | Eye open/close, pupil movements         |
| 3     | Mouth   | 口             | Mouth shapes, lip sync                  |
| 4     | Other   | その他         | Everything else (full-face, body, etc.) |

---

## References

- [Doc template](./TEMPLATE.md)
- `MorphType` enum: `mmd/core/data_types.py`
- `PMXMorph` dataclass: `mmd/core/data_types.py`
- `MorphTreeWidget` UI: `mmd/ui/morph_tree_widget.py`
- `_AbstractMorphSource` protocol: `mmd/ui/morph_tree_widget.py`
- `boneMorphNode` plugin ID: `0x39390053` (temporary — should be registered)
- Memory: `/memories/repo/maya-quaternion-anim-curves.md`
- Meta issue: [#25 Morph system](https://github.com/sstarosz/MayaMMD/issues/25)
