# PMX Naming Conventions

DAG nodes (joints, controllers, ikHandles, root/bones/mesh groups) use short names because their DAG path already disambiguates across multiple model imports. DG nodes (solvers, multiplyDivide, blendshape, boneMorph, materials, textures) include the model name prefix since they have no DAG path.

All naming constants are defined centrally in `pmx_naming_manager.py` (`mmd/maya/pmx_naming_manager.py`). Refer to those constants (e.g. `MORPH_CONTROLLER_SUFFIX`, `IK_HANDLE_SUFFIX`) instead of hardcoding suffix strings.

## Convention table

All suffixes start with a capital letter for consistency.

| Node                               | Type | Naming pattern                         | Example                                          |
| ---------------------------------- | ---- | -------------------------------------- | ------------------------------------------------ |
| Joint                              | DAG  | `{bone_name}_Jnt`                      | `upper_arm_R_Jnt` / `Bone_52_Jnt`                |
| Tail joint                         | DAG  | `{bone_name}_TailJnt`                  | `upper_arm_R_TailJnt`                            |
| inheritCtrl (transform)            | DAG  | `{bone_name}_InheritCtrl`              | `shoulderP_R_InheritCtrl`                        |
| morphCtrl (transform)              | DAG  | `{bone_name}_MorphCtrl`                | `arm_twist_R_Bone_52_MorphCtrl`                  |
| ikHandle                           | DAG  | `{bone_name}_IkHandle`                 | `Bone_126_IkHandle`                              |
| Root transform                     | DAG  | `{model}_Root`                         | `GirlsFrontline_TololoDefault_Root`              |
| Bones group                        | DAG  | `{model}_Bones`                        | `GirlsFrontline_TololoDefault_Bones`             |
| Mesh transform                     | DAG  | `{model}_Mesh`                         | `GirlsFrontline_TololoDefault_Mesh`              |
| Mesh shape                         | DAG  | `{model}_Mesh_Shape`                   | `GirlsFrontline_TololoDefault_Mesh_Shape`        |
| Geo group                          | DAG  | `{model}_Geo`                          | `GirlsFrontline_TololoDefault_Geo`               |
| RigidBodies group                  | DAG  | `{model}_RigidBodies`                  | `GirlsFrontline_TololoDefault_RigidBodies`       |
| RigidBodySolver (pmxRigidBodyNode) | DAG  | `{model}_RigidBodySolver`              | `GirlsFrontline_TololoDefault_RigidBodySolver`   |
| RigidBody guide (transform)        | DAG  | `{model}_{rigidbody_name}`             | `GirlsFrontline_TololoDefault_leg_rb`            |
| RigidBodyShape (pmxRigidBodyShape) | DAG  | `{model}_{rigidbody_name}Shape`        | `GirlsFrontline_TololoDefault_leg_rbShape`       |
| rotScale (multiplyDivide)          | DG   | `{model}_{bone_name}_RotScale`         | `GirlsFrontline_TololoDefault_Bone_11_RotScale`  |
| ccdSolver                          | DG   | `{model}_{bone_name}_CcdSolver`        | `GirlsFrontline_TololoDefault_Bone_24_CcdSolver` |
| BoneMorph node                     | DG   | `{model}_BoneMorph`                    | `GirlsFrontline_TololoDefault_BoneMorph`         |
| BlendShape node                    | DG   | `{model}_BlendShape`                   | `GirlsFrontline_TololoDefault_BlendShape`        |
| Material                           | DG   | `{model}_{material_name}_Mat`          | `GirlsFrontline_TololoDefault_Face_Mat`          |
| Shading group                      | DG   | `{model}_{material_name}_SG`           | `GirlsFrontline_TololoDefault_Face_SG`           |
| Texture                            | DG   | `{model}_{texture_name}_Tex`           | `GirlsFrontline_TololoDefault_body_Tex`          |
| Place2dTexture                     | DG   | `{model}_{texture_name}_Place2D`       | `GirlsFrontline_TololoDefault_body_Place2D`      |
| Blendshape target                  | attr | `{morph_name}` (attribute, not a node) | `eye_blink`                                      |

## Why DAG nodes omit the model prefix

Maya resolves nodes by their **long DAG path** (e.g. `|GirlsFrontline_TololoDefault_Root|GirlsFrontline_TololoDefault_Bones|...|Bone_13_InheritCtrl`), so the short name alone is unambiguous — the DAG hierarchy already identifies which model the node belongs to.

DG nodes have no DAG path, so they need the model name embedded in their short name to avoid collisions when multiple models are imported into the same scene.

## Why the `PMX_` prefix was removed

Originally a `PMX_` prefix was used on top-level containers (Root, Bones, Mesh, etc.) and DG nodes. This was inconsistent — some nodes had `PMX_`, others had the model name, and some had neither. The simplified rule is:

> **If a node already includes the model name, it's identifiable without `PMX_`. If it has no model name (DAG bone/controller), it's identified by its DAG path.**

Only blendshape target names are exceptions — they're attributes on the blendshape node, not standalone nodes, so they don't need any disambiguation prefix.

## `make_unique` disambiguation

Even with short names, `PMXNamingManager.make_unique()` appends numeric suffixes (`_1`, `_2`, …) if a name already exists in the scene. This covers edge cases such as:
- Importing the same model twice
- Two models with identically named bones
- Manual node creation that conflicts with an auto-generated name
