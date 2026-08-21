# PMX Custom Attributes Reference

This document lists every custom attribute stored on Maya scene nodes during PMX import.  
These attributes make the scene **self-describing** — all PMX metadata can be reconstructed
from the scene by reading these attributes, enabling selection-driven model targeting
without relying on in-memory data structures.

---

## Per-Joint Attributes (`pmxBoneData` compound)

**Stored on:** Each Maya joint created from a PMX bone.  
**Added by:** `bone_builder.py:_add_pmx_bone_attributes()`  
**Compound attribute:** `pmxBoneData` (short name: `pmxBone`)

All children live under the compound so the Attribute Editor groups them cleanly.

### Identification

| Long name            | Short name     | Type | Description                         |
| -------------------- | -------------- | ---- | ----------------------------------- |
| `mayaJointIndex`     | `mayaIdx`      | int  | Sequential index in the joints list |
| `pmxBoneIndex`       | `pmxIdx`       | int  | 0-based PMX bone index              |
| `pmxParentBoneIndex` | `pmxParentIdx` | int  | PMX parent bone index (-1 = root)   |
| `pmxLevel`           | `pmxLvl`       | int  | Bone level / hierarchy depth        |

### Names

| Long name          | Short name   | Type   | Description                       |
| ------------------ | ------------ | ------ | --------------------------------- |
| `pmxNameLocal`     | `pmxNameLoc` | string | Original PMX bone name (Japanese) |
| `pmxNameUniversal` | `pmxNameUni` | string | Universal PMX bone name (English) |

These two attributes are the **key to selection-based targeting** — they allow
reconstructing the `bone_name_map` (PMX name → Maya joint name) by scanning
scene joints under a model root.

### World Position

| Long name          | Short name | Type   | Description                              |
| ------------------ | ---------- | ------ | ---------------------------------------- |
| `pmxWorldPosition` | `pmxPos`   | float3 | MMD-space world position (Z NOT flipped) |

### Tail Info

| Long name       | Short name   | Type   | Description                                        |
| --------------- | ------------ | ------ | -------------------------------------------------- |
| `pmxTailIndex`  | `pmxTailIdx` | int    | Index-based tail (only if `pmxTailIsIndex` is set) |
| `pmxTailOffset` | `pmxTailOfs` | float3 | Offset-based tail (only if tail is a Vec3)         |

### Flags (all bool)

| Long name                 | Short name        | PMXBoneFlagBits        |
| ------------------------- | ----------------- | ---------------------- |
| `pmxRotatable`            | `pmxRot`          | ROTATABLE              |
| `pmxTranslatable`         | `pmxTrans`        | TRANSLATABLE           |
| `pmxVisible`              | `pmxVis`          | VISIBLE                |
| `pmxEnabled`              | `pmxEnabled`      | ENABLED                |
| `pmxHasIK`                | `pmxIK`           | IK                     |
| `pmxInheritRotation`      | `pmxInhRot`       | INHERIT_ROTATION       |
| `pmxInheritTranslation`   | `pmxInhTrans`     | INHERIT_TRANSLATION    |
| `pmxUseFixedAxis`         | `pmxFixAxis`      | FIXED_AXIS             |
| `pmxUseLocalCoordinate`   | `pmxLocCoord`     | LOCAL_COORDINATE       |
| `pmxPhysicsAfterDeform`   | `pmxPhysDeform`   | PHYSICS_AFTER_DEFORM   |
| `pmxExternalParentDeform` | `pmxExtParDeform` | EXTERNAL_PARENT_DEFORM |

### Optional Data

Only present when the corresponding PMX flag is set:

| Long name                | Short name     | Type   | Condition                                        |
| ------------------------ | -------------- | ------ | ------------------------------------------------ |
| `pmxFixedAxis`           | `pmxFixAx`     | float3 | FIXED_AXIS flag set                              |
| `pmxLocalCoordX`         | `pmxLocX`      | float3 | LOCAL_COORDINATE flag set                        |
| `pmxLocalCoordZ`         | `pmxLocZ`      | float3 | LOCAL_COORDINATE flag set                        |
| `pmxInheritParentIndex`  | `pmxInhParIdx` | int    | INHERIT_ROTATION or INHERIT_TRANSLATION flag set |
| `pmxInheritFactor`       | `pmxInhFac`    | float  | INHERIT_ROTATION or INHERIT_TRANSLATION flag set |
| `pmxExternalParentIndex` | `pmxExtParIdx` | int    | EXTERNAL_PARENT_DEFORM flag set                  |

### Rest Pose (captured after skeleton build)

| Long name           | Short name  | Type  |
| ------------------- | ----------- | ----- |
| `pmxRestTranslateX` | `pmxRestTx` | float |
| `pmxRestTranslateY` | `pmxRestTy` | float |
| `pmxRestTranslateZ` | `pmxRestTz` | float |
| `pmxRestRotateX`    | `pmxRestRx` | float |
| `pmxRestRotateY`    | `pmxRestRy` | float |
| `pmxRestRotateZ`    | `pmxRestRz` | float |

---

## Per-IK-Handle Attributes

**Stored on:** Each IK handle created for a PMX IK bone.  
**Added by:** `bone_builder.py:_capture_rest_pose_on_ik_handles()`

### Rest Pose

| Long name             | Short name    | Type  |
| --------------------- | ------------- | ----- |
| `pmxIkRestTranslateX` | `pmxIkRestTx` | float |
| `pmxIkRestTranslateY` | `pmxIkRestTy` | float |
| `pmxIkRestTranslateZ` | `pmxIkRestTz` | float |
| `pmxIkRestRotateX`    | `pmxIkRestRx` | float |
| `pmxIkRestRotateY`    | `pmxIkRestRy` | float |
| `pmxIkRestRotateZ`    | `pmxIkRestRz` | float |

---

## Per-Model Root Attributes

**Stored on:** The root transform node (`{model}_Root`).  
**Added by:** `pmx_scene_builder.py:build_pmx_scene()`

These attributes make the model self-describing for fast root discovery and UI display.

| Long name          | Type   | Description                                                                    |
| ------------------ | ------ | ------------------------------------------------------------------------------ |
| `pmxModelName`     | string | Model display name (local name, falling back to universal name, then ASCII)    |
| `pmxRigidBodyNode` | string | Solver node name of the model's `pmxRigidBodyNode` (only when one was created) |

> **Note:** `pmxRigidBodyNode` is the model's native C++ rigid-body physics
> solver node — it simulates the imported rigid bodies and constraints with
> Bullet. The root attribute above simply records which solver node the model
> uses.
>
> On the solver node itself, the per-body data is **not** stored as solver
> attributes: `bodyShapes[]` is a message array connected to one
> `pmxRigidBodyShape` node per body, and each shape node carries the body's
> PMX-verbatim fields (`bodyColliderType`, `bodyShapeSize`, `bodyRestTranslate`/
> `bodyRestRotate`, `bodyGroupId`, `bodyMaskGroup0..15`, `bodyMass`,
> `bodyPhysicsMode`, …).  The shape's DAG parent transform is the body's
> viewport GUIDE — the solver drives it to the body's CURRENT pose each frame
> (`outGuideTranslate[]`/`outGuideRotate[]` on the solver), so the collider
> follows the animation.  The REST pose lives in the shape's
> `bodyRestTranslate`/`bodyRestRotate` attributes — edit those (or the other
> body attributes) in the Attribute Editor to change the simulation.

---

## Per-BlendShape Attributes

**Stored on:** The blendShape deformer node (`{model}_BlendShape`).  
**Added by:** `morph_builder.py:create_vertex_blend_shapes()`

| Long name         | Type             | Description                                                                                                                                                 |
| ----------------- | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `pmxMorphMapping` | compound (multi) | Array of `{pmxName, mayaAlias}` pairs. Each element has two string children. A "+" button in the Attribute Editor lets users add, remove, and edit entries. |

Example in the Attribute Editor:
```
pmxMorphMapping[0]
  ├── pmxName: "笑顔"
  └── mayaAlias: "Smile"
pmxMorphMapping[1]
  ├── pmxName: "Smile"
  └── mayaAlias: "Smile"
pmxMorphMapping[2]
  ├── pmxName: "目閉じ"
  └── mayaAlias: "EyeClose"
```

This attribute is the **key to reconstructing `morph_name_map`** from the scene,
analogous to how `pmxNameLocal`/`pmxNameUniversal` on joints reconstructs `bone_name_map`.

---

## Discovery Utilities

The following functions in `pmx_model_utils.py` read these attributes to reconstruct
model data from the scene:

| Function                           | Uses attributes                                                                    |
| ---------------------------------- | ---------------------------------------------------------------------------------- |
| `_is_pmx_root(name)`               | `pmxModelName` on self (fast path); `pmxNameLocal` on descendant joints (fallback) |
| `find_model_root_from_selection()` | Walks DAG → `_is_pmx_root`                                                         |
| `discover_model_roots_in_scene()`  | `_is_pmx_root` on all transforms                                                   |
| `build_bone_map_from_scene(root)`  | `pmxNameLocal`, `pmxNameUniversal` on joints                                       |
| `build_morph_map_from_scene(root)` | `pmxMorphMapping` compound array on blendShape node                                |
| `find_blend_shape_node(root)`      | Scans mesh deformation history for `blendShape` node type                          |
| `find_bone_morph_node(root)`       | Scans all `boneMorphNode` nodes, traces `outputRotate` connections to root         |
| `find_ik_handles(root)`            | (no attrs — uses DAG hierarchy)                                                    |

---

## Summary: What's Discoverable

| Data                 | Discoverable? | Via                                                            |
| -------------------- | ------------- | -------------------------------------------------------------- |
| Model display name   | ✅ Yes         | `pmxModelName` on root                                         |
| Bone map (PMX→Maya)  | ✅ Yes         | `pmxNameLocal`/`pmxNameUniversal` on joints                    |
| Morph map (PMX→Maya) | ✅ Yes         | `pmxMorphMapping` compound array on blendShape node            |
| BlendShape node name | ✅ Yes         | Mesh deformation history scan (listHistory, type=`blendShape`) |
| BoneMorph node name  | ✅ Yes         | Type scan + `outputRotate` connection tracing to root          |
| IK handle list       | ✅ Yes         | `listRelatives(root, type="ikHandle")`                         |
| Rigid body node name | ✅ Yes         | `pmxRigidBodyNode` on root                                     |
| Rest pose values     | ✅ Yes         | `pmxRest*` on joints, `pmxIkRest*` on IK handles               |
| Bone flags           | ✅ Yes         | `pmx*` bool attributes on joints                               |
