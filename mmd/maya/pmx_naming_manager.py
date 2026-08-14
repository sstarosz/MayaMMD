import os
import re
from collections import defaultdict
from typing import Optional

from mmd.core.data_types import PmxModel

# ---------------------------------------------------------------------------
# Naming constants — single source of truth for naming conventions.
# Tests and other modules can reference these instead of hardcoding patterns.
# ---------------------------------------------------------------------------

# Suffix for joint names.
# Example: ``upper_arm_R_Jnt``
JOINT_SUFFIX = "_Jnt"

# Suffix for bone morph controllers created by the boneBlendShape command.
# Example: ``upper_arm_twist_MorphCtrl``
MORPH_CONTROLLER_SUFFIX = "_MorphCtrl"

# Suffix for inheritance rotation controllers.
# Example: ``Bone_14_InheritCtrl``
INHERIT_ROTATION_CONTROLLER_SUFFIX = "_InheritCtrl"

# Suffix for the multiplyDivide node that scales inherited rotation.
# Example: ``Bone_14_RotScale``
INHERIT_ROTATION_MULTIPLYDIVIDE_SUFFIX = "_RotScale"

# Suffix for IK handle names.
# Example: ``Bone_126_IkHandle``
IK_HANDLE_SUFFIX = "_IkHandle"

# Suffix for CCD IK solver node names.
# Example: ``Bone_126_CcdSolver``
IK_SOLVER_SUFFIX = "_CcdSolver"

# Suffix for tail joint names.
# Example: ``Bone_126_TailJnt``
TAIL_JOINT_SUFFIX = "_TailJnt"

# ---------------------------------------------------------------------------
# Naming conventions summary
# ---------------------------------------------------------------------------
# DAG nodes (joints, controllers, ikHandles, root/bones/mesh groups) use
# short names because their DAG path already disambiguates across multiple
# model imports.  DG nodes (solvers, multiplyDivide, blendshape, boneMorph,
# materials, textures) include the model name prefix since they have no DAG
# path.
#
# The ``PMX_`` prefix is not used anywhere — the model name alone is
# sufficient for identifying tool-generated nodes.
#
# +-----------------------+------+-------------------------------------------------+
# | Node                  | Type | Naming pattern                                  |
# +-----------------------+------+-------------------------------------------------+
# | Joint                 | DAG  | ``{bone_name}_Jnt``                             |
# | Tail joint            | DAG  | ``{bone_name}_TailJnt``                         |
# | inheritCtrl           | DAG  | ``{bone_name}_InheritCtrl``                     |
# | morphCtrl             | DAG  | ``{bone_name}_MorphCtrl``                       |
# | ikHandle              | DAG  | ``{bone_name}_IkHandle``                        |
# | Root transform        | DAG  | ``{model}_Root``                                |
# | Bones group           | DAG  | ``{model}_Bones``                               |
# | Mesh transform        | DAG  | ``{model}_Mesh``                                |
# | Mesh shape            | DAG  | ``{model}_Mesh_Shape``                          |
# | Geo group             | DAG  | ``{model}_Geo``                                 |
# | rotScale              | DG   | ``{model}_{bone_name}_RotScale``                |
# | ccdSolver             | DG   | ``{model}_{bone_name}_CcdSolver``               |
# | BoneMorph node        | DG   | ``{model}_BoneMorph``                           |
# | BlendShape node       | DG   | ``{model}_BlendShape``                          |
# | Material              | DG   | ``{model}_{material_name}_Mat``                 |
# | Shading group         | DG   | ``{model}_{material_name}_SG``                  |
# | Texture               | DG   | ``{model}_{texture_name}_Tex``                  |
# | Place2dTexture        | DG   | ``{model}_{texture_name}_Place2D``              |
# | Blendshape target     | attr | ``{morph_name}`` (attribute, not a node)         |
# +-----------------------+------+-------------------------------------------------+
#


class PMXNamingManager:
    """Manages naming for PMX model elements in Maya.

    Pre-processes all PMX names to create Maya-compatible names, detects
    possible naming issues, and provides unique alternatives by checking
    the live Maya scene for existing nodes.

    Unlike an in-memory registry, this approach survives Maya restarts
    because it queries the scene directly via ``cmds.objExists()``.
    """

    def __init__(self, pmx_data: PmxModel):
        self.pmx_data = pmx_data

        # Cache of desired_name -> unique_name used during this import.
        # Ensures repeated calls for the same element return the same name
        # without flooding Maya with redundant objExists queries.
        self._name_cache: dict[str, str] = {}

        # Store sanitized model names
        self.model_name_local = self._sanitize_name(pmx_data.header.model_name_local)
        self.model_name_universal = self._sanitize_name(
            pmx_data.header.model_name_universal
        )

        # Pre-process all names
        self._material_name_map: dict[
            int, tuple[str, str]
        ] = {}  # material_index -> (material_name, shading_group_name)

        self._texture_name_map: dict[
            int, tuple[str, str]
        ] = {}  # texture_index -> (texture_name, place2d_name)

        self._bone_name_map: dict[int, str] = {}  # bone_index -> bone_name

        self._blendshape_name_map: dict[
            int, str
        ] = {}  # blendshape_index -> blendshape_name

        self._joint_name_map: dict[int, str] = {}  # joint_index -> joint_name #

        self._inherit_rotation_controller_map: dict[
            int, str
        ] = {}  # bone_index -> inheritCtrl name

        self._inherit_rotation_multiplydivide_map: dict[
            int, str
        ] = {}  # bone_index -> _rot_scale multiplyDivide name

        self._preprocess_all_names()

    # ------------------------------------------------------------------
    # Scene-aware unique naming
    # ------------------------------------------------------------------

    @staticmethod
    def _name_exists_in_maya(name: str) -> bool:
        """Check if a name already exists in the Maya scene.

        Gracefully handles cases where Maya is not available (unit tests).
        """
        try:
            import maya.cmds as cmds

            return cmds.objExists(name)
        except ImportError:
            return False
        except Exception:
            return False

    def make_unique(self, desired_name: str) -> str:
        """Return a Maya-unique variant of *desired_name*.

        If ``desired_name`` is already cached from this import, the cached
        version is returned.  Otherwise the method checks whether the name
        exists in the Maya scene and, if it does, appends a numeric suffix
        (``_1``, ``_2``, …) until a free name is found.

        The result is stored in ``_name_cache`` so that subsequent requests
        for the same *desired_name* (e.g. from a different getter) are
        consistent.
        """
        # 1. Already resolved during this import?
        cached = self._name_cache.get(desired_name)
        if cached is not None:
            return cached

        # 2. Does not exist anywhere → use as-is.
        if not self._name_exists_in_maya(desired_name):
            self._name_cache[desired_name] = desired_name
            return desired_name

        # 3. Name exists – append a numeric suffix.
        base_name = desired_name
        counter = 1

        # Check if name already has a number at the end, so we keep
        # incrementing from there instead of creating nested suffixes.
        match = re.search(r"(\d+)$", base_name)
        if match:
            base_name = base_name[: match.start()]
            counter = int(match.group(1)) + 1

        while True:
            candidate = f"{base_name}_{counter}"
            if (
                not self._name_exists_in_maya(candidate)
                and candidate not in self._name_cache
            ):
                self._name_cache[desired_name] = candidate
                return candidate
            counter += 1

    def _sanitize_name(self, name: str) -> str:
        """Sanitize any name for Maya compatibility.

        * Replaces spaces and special characters with underscores.
        * Removes leading/trailing underscores.
        * Ensures the result does not start with a digit (Maya rejects
          node names starting with numbers).
        * If the result would be empty, returns "Model".
        """
        if not name:
            return ""

        # Replace spaces with underscores
        name = name.replace(" ", "_")

        # Replace any non-ASCII or special characters
        name = re.sub(r"[^a-zA-Z0-9_]", "_", name)

        # Remove multiple consecutive underscores
        name = re.sub(r"_+", "_", name)

        # Remove leading/trailing underscores
        name = name.strip("_")

        # Ensure name doesn't start with a digit (Maya requirement)
        if name and name[0].isdigit():
            name = f"Model_{name}"

        # If the result is empty string, fall back to "Model"
        if not name:
            name = "Model"

        return name

    def _preprocess_all_names(self):
        """Pre-process all names in the PMX model to create Maya-compatible names."""
        # Step 1: Pre-process model names
        self._preprocess_model_names()

        # Step 2: Pre-process material names
        self._preprocess_material_names()

        # Step 3: Pre-process texture names
        self._preprocess_texture_names()

        # Step 4: Pre-process blendshape names
        self._preprocess_blendshape_names()

        # Step 5: Pre-process bone names
        self._preprocess_bone_names()

        # Step 6: Pre-process joint names
        self._preprocess_joint_names()

        # Step 8: Pre-process inheritCtrl names for inheritance rotation
        self._preprocess_inherit_rotation_controller_names()

        # Step 9: Pre-process multiplyDivide names for inheritance rotation
        self._preprocess_inherit_rotation_multiplydivide_names()

    def _preprocess_model_names(self):
        """Pre-process model-level names."""
        # These will be registered when requested
        pass

    def _preprocess_material_names(self):
        """Pre-process all material names to handle duplicates and special characters."""
        model_name = self.get_model_name()
        unique_names = set()
        count_of_unique = defaultdict(int)
        for idx, material in enumerate(self.pmx_data.materials):
            # Get the base name for this material
            if material.name_local and self._is_ascii_safe(material.name_local):
                base_name = self._sanitize_name(material.name_local)
            elif material.name_universal and self._is_ascii_safe(
                material.name_universal
            ):
                base_name = self._sanitize_name(material.name_universal)
            else:
                base_name = f"Material_{idx}"

            # Store unique base names
            unique_names.add(base_name)
            # Assign material name to map. First occurrence gets base name, subsequent get numbered suffixes
            count_of_unique[base_name] += 1
            occurrence_index = count_of_unique[base_name] - 1
            if occurrence_index == 0:
                material_name = f"{model_name}_{base_name}_Mat"
                shading_group_name = f"{model_name}_{base_name}_SG"
            else:
                material_name = f"{model_name}_{base_name}_{occurrence_index}_Mat"
                shading_group_name = f"{model_name}_{base_name}_{occurrence_index}_SG"
            self._material_name_map[idx] = (material_name, shading_group_name)

    def _preprocess_texture_names(self):
        """Pre-process texture names with sequential fallback for empty/non-ASCII names."""
        model_name = self.get_model_name()
        unique_names = set()
        count_of_unique = defaultdict(int)
        for idx, tex_path in enumerate(self.pmx_data.textures_paths):
            tex_name = self._sanitize_name(
                os.path.splitext(os.path.basename(tex_path))[0]
            )
            if not tex_name:
                tex_name = "Texture"
            base_name = tex_name  # Remember the original base name

            # Find a unique name
            occurrence_index = count_of_unique[base_name]
            unique_tex_name = (
                base_name
                if occurrence_index == 0
                else f"{base_name}_{occurrence_index}"
            )
            while unique_tex_name in unique_names:
                occurrence_index += 1
                unique_tex_name = f"{base_name}_{occurrence_index}"

            unique_names.add(unique_tex_name)
            count_of_unique[base_name] = occurrence_index + 1

            # Assign texture name to map
            texture_name = f"{model_name}_{unique_tex_name}_Tex"
            place2d_name = f"{model_name}_{unique_tex_name}_Place2D"
            self._texture_name_map[idx] = (texture_name, place2d_name)

    def _preprocess_bone_names(self):
        """Pre-process bone names to handle duplicates and special characters."""
        unique_names = set()
        count_of_unique = defaultdict(int)

        for idx, bone in enumerate(self.pmx_data.bones):
            # Get the base name for this bone
            if bone.nameLocal and self._is_ascii_safe(bone.nameLocal):
                base_name = self._sanitize_name(bone.nameLocal)
            elif bone.nameUniversal and self._is_ascii_safe(bone.nameUniversal):
                base_name = self._sanitize_name(bone.nameUniversal)
            else:
                base_name = f"Bone_{idx}"

            # Find a unique name
            occurrence_index = count_of_unique[base_name]
            unique_bone_name = (
                base_name
                if occurrence_index == 0
                else f"{base_name}_{occurrence_index}"
            )
            while unique_bone_name in unique_names:
                occurrence_index += 1
                unique_bone_name = f"{base_name}_{occurrence_index}"

            unique_names.add(unique_bone_name)
            count_of_unique[base_name] = occurrence_index + 1

            # Assign bone name to map
            self._bone_name_map[idx] = unique_bone_name

    def _preprocess_blendshape_names(self):
        """Pre-process blendshape names to handle duplicates and special characters."""
        unique_names = set()
        count_of_unique = defaultdict(int)

        for idx, morph in enumerate(self.pmx_data.morphs):
            # Get the base name for this morph
            if morph.name_universal and self._is_ascii_safe(morph.name_universal):
                base_name = self._sanitize_name(morph.name_universal)
            elif morph.name_local and self._is_ascii_safe(morph.name_local):
                base_name = self._sanitize_name(morph.name_local)
            else:
                base_name = f"Morph_{idx}"

            # Find a unique name
            occurrence_index = count_of_unique[base_name]
            unique_morph_name = (
                base_name
                if occurrence_index == 0
                else f"{base_name}_{occurrence_index}"
            )
            while unique_morph_name in unique_names:
                occurrence_index += 1
                unique_morph_name = f"{base_name}_{occurrence_index}"

            unique_names.add(unique_morph_name)
            count_of_unique[base_name] = occurrence_index + 1

            # Assign morph name to map
            self._blendshape_name_map[idx] = unique_morph_name

    def _preprocess_joint_names(self):
        """Pre-process joint names to handle duplicates and special characters."""
        unique_names = set()
        count_of_unique = defaultdict(int)

        for idx, joint in enumerate(self.pmx_data.joints):
            # Get the base name for this joint
            if joint.name_local and self._is_ascii_safe(joint.name_local):
                base_name = self._sanitize_name(joint.name_local)
            elif joint.name_universal and self._is_ascii_safe(joint.name_universal):
                base_name = self._sanitize_name(joint.name_universal)
            else:
                base_name = f"Joint_{idx}"

            # Find a unique name
            occurrence_index = count_of_unique[base_name]
            unique_joint_name = (
                base_name
                if occurrence_index == 0
                else f"{base_name}_{occurrence_index}"
            )
            while unique_joint_name in unique_names:
                occurrence_index += 1
                unique_joint_name = f"{base_name}_{occurrence_index}"

            unique_names.add(unique_joint_name)
            count_of_unique[base_name] = occurrence_index + 1

            # Assign joint name to map
            self._joint_name_map[idx] = unique_joint_name

    def _preprocess_inherit_rotation_controller_names(self):
        """Pre-process inherit rotation controller names for bones with INHERIT_ROTATION flag.

        Generates names in the pattern ``{bone_name}_inheritCtrl``.
        Controller transforms are DAG nodes, so their DAG path already
        disambiguates across multiple model imports — no model prefix needed.
        The actual transform name is resolved through ``make_unique()`` to
        avoid collisions.
        """
        from mmd.core.data_types import PMXBoneFlagBits

        unique_names = set()
        count_of_unique = defaultdict(int)

        for idx, bone in enumerate(self.pmx_data.bones):
            if not (bone.flags & PMXBoneFlagBits.INHERIT_ROTATION):
                continue

            bone_name = self._bone_name_map.get(idx, f"Bone_{idx}")
            base_name = f"{bone_name}{INHERIT_ROTATION_CONTROLLER_SUFFIX}"

            occurrence_index = count_of_unique[base_name]
            unique_ctrl_name = (
                base_name
                if occurrence_index == 0
                else f"{base_name}_{occurrence_index}"
            )
            while unique_ctrl_name in unique_names:
                occurrence_index += 1
                unique_ctrl_name = f"{base_name}_{occurrence_index}"

            unique_names.add(unique_ctrl_name)
            count_of_unique[base_name] = occurrence_index + 1

            self._inherit_rotation_controller_map[idx] = unique_ctrl_name

    def _preprocess_inherit_rotation_multiplydivide_names(self):
        """Pre-process multiplyDivide node names for bones with INHERIT_ROTATION.

        Generates names in the pattern ``{model_name}_{bone_name}_rotScale``,
        using the ``INHERIT_ROTATION_MULTIPLYDIVIDE_SUFFIX`` constant.
        """
        from mmd.core.data_types import PMXBoneFlagBits

        model_name = self.get_model_name()
        unique_names = set()
        count_of_unique = defaultdict(int)

        for idx, bone in enumerate(self.pmx_data.bones):
            if not (bone.flags & PMXBoneFlagBits.INHERIT_ROTATION):
                continue

            bone_name = self._bone_name_map.get(idx, f"Bone_{idx}")
            # multiplyDivide is a DG node (no DAG path for disambiguation),
            # so the model name prefix is needed for uniqueness across imports.
            base_name = (
                f"{model_name}_{bone_name}{INHERIT_ROTATION_MULTIPLYDIVIDE_SUFFIX}"
            )

            occurrence_index = count_of_unique[base_name]
            unique_md_name = (
                base_name
                if occurrence_index == 0
                else f"{base_name}_{occurrence_index}"
            )
            while unique_md_name in unique_names:
                occurrence_index += 1
                unique_md_name = f"{base_name}_{occurrence_index}"

            unique_names.add(unique_md_name)
            count_of_unique[base_name] = occurrence_index + 1

            self._inherit_rotation_multiplydivide_map[idx] = unique_md_name

    def _is_ascii_safe(self, name: str) -> bool:
        """Check if name is ASCII-safe (Maya-friendly)."""
        try:
            name.encode("ascii")
            return True
        except UnicodeEncodeError:
            return False

    # ---------------#
    # ---General-----#
    # ---------------#
    def get_model_name(self) -> str:
        """Get unique model name."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}"
        else:
            desired_name = "Model"

        return desired_name  # Model name doesn't need to be unique in Maya, as it's not a node

    def get_root_name(self) -> str:
        """Get unique root transform name."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_Root"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_Root"
        else:
            desired_name = "Root"

        return self.make_unique(desired_name)

    def get_geo_group_name(self) -> str:
        """Get unique geometry group name."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_Geo"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_Geo"
        else:
            desired_name = "Geo"

        return self.make_unique(desired_name)

    def get_mesh_name(self) -> str:
        """Get unique mesh transform name."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_Mesh"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_Mesh"
        else:
            desired_name = "Mesh"

        return self.make_unique(desired_name)

    def get_shape_name(self) -> str:
        """Get unique mesh shape name."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_Mesh_Shape"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_Mesh_Shape"
        else:
            desired_name = "Mesh_Shape"

        return self.make_unique(desired_name)

    def get_material_name(self, material_index: int) -> str:
        """Get unique material name."""
        if material_index in self._material_name_map:
            material_name, _ = self._material_name_map[material_index]
            return self.make_unique(material_name)

        # Fallback for unexpected indices
        model_name = self.get_model_name()
        base_name = f"Material_{material_index}"
        material_name = f"{model_name}_{base_name}_Mat"
        return self.make_unique(material_name)

    def get_shading_group_name(self, material_index: int) -> str:
        """Get unique shading group name."""
        if material_index in self._material_name_map:
            _, shading_group_name = self._material_name_map[material_index]
            return self.make_unique(shading_group_name)

        # Fallback for unexpected indices
        model_name = self.get_model_name()
        base_name = f"Material_{material_index}"
        shading_group_name = f"{model_name}_{base_name}_SG"
        return self.make_unique(shading_group_name)

    def get_texture_name(self, texture_index: int) -> str:
        """Get unique texture name."""
        if texture_index in self._texture_name_map:
            texture_name, _ = self._texture_name_map[texture_index]
            return self.make_unique(texture_name)

        # Fallback for unexpected indices
        model_name = self.get_model_name()
        base_name = f"Texture_{texture_index}"
        texture_name = f"{model_name}_{base_name}_Tex"
        return self.make_unique(texture_name)

    def get_place2d_name(self, texture_index: int) -> str:
        """Get unique place2dTexture name."""
        if texture_index in self._texture_name_map:
            _, place2d_name = self._texture_name_map[texture_index]
            return self.make_unique(place2d_name)

        # Fallback for unexpected indices
        model_name = self.get_model_name()
        base_name = f"Texture_{texture_index}"
        place2d_name = f"{model_name}_{base_name}_Place2D"
        return self.make_unique(place2d_name)

    # ---------------#
    # -----Bones-----#
    # ---------------#
    def get_bone_group_name(self) -> str:
        """Get unique bone group name."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_Bones"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_Bones"
        else:
            desired_name = "Bones"

        return self.make_unique(desired_name)

    def get_bone_name(self, bone_index: int) -> str:
        """Get unique bone joint name."""
        if bone_index in self._bone_name_map:
            bone_name = self._bone_name_map[bone_index]
            return self.make_unique(f"{bone_name}{JOINT_SUFFIX}")

        base_name = f"Bone_{bone_index}"
        bone_name = f"{base_name}{JOINT_SUFFIX}"
        return self.make_unique(bone_name)

    def get_tail_bone_name(self, bone_index: int) -> str:
        """Get unique tail bone joint name."""
        bone_name = self.get_bone_name(bone_index)
        # Remove the JOINT_SUFFIX and add TAIL_JOINT_SUFFIX
        if bone_name.endswith(JOINT_SUFFIX):
            base = bone_name[: -len(JOINT_SUFFIX)]
            desired_name = f"{base}{TAIL_JOINT_SUFFIX}"
        else:
            # Fallback: just append tail suffix
            desired_name = f"{bone_name}{TAIL_JOINT_SUFFIX}"

        return self.make_unique(desired_name)

    def get_inherit_rotation_controller_name(self, bone_index: int) -> str:
        """Get unique inherit rotation controller name for a bone with INHERIT_ROTATION."""
        if bone_index in self._inherit_rotation_controller_map:
            ctrl_name = self._inherit_rotation_controller_map[bone_index]
            return self.make_unique(ctrl_name)

        # Fallback for unexpected indices
        bone_name = f"Bone_{bone_index}"
        base_name = f"{bone_name}{INHERIT_ROTATION_CONTROLLER_SUFFIX}"
        return self.make_unique(base_name)

    def get_inherit_rotation_multiplydivide_name(self, bone_index: int) -> str:
        """Get unique multiplyDivide node name for a bone with INHERIT_ROTATION."""
        if bone_index in self._inherit_rotation_multiplydivide_map:
            md_name = self._inherit_rotation_multiplydivide_map[bone_index]
            return self.make_unique(md_name)

        # Fallback for unexpected indices
        model_name = self.get_model_name()
        bone_name = f"Bone_{bone_index}"
        base_name = f"{model_name}_{bone_name}{INHERIT_ROTATION_MULTIPLYDIVIDE_SUFFIX}"
        return self.make_unique(base_name)

    def get_ik_handle_name(self, bone_index: int) -> str:
        """Get unique IK handle name for an IK bone.

        IK handles are DAG nodes, so their DAG path already disambiguates
        across multiple model imports — no model prefix needed.
        """
        bone_name = self.get_bone_name(bone_index)
        # Remove the JOINT_SUFFIX and add IK_HANDLE_SUFFIX
        if bone_name.endswith(JOINT_SUFFIX):
            base = bone_name[: -len(JOINT_SUFFIX)]
            desired_name = f"{base}{IK_HANDLE_SUFFIX}"
        else:
            desired_name = f"{bone_name}{IK_HANDLE_SUFFIX}"

        return self.make_unique(desired_name)

    def get_ik_solver_name(self, bone_index: int) -> str:
        """Get unique CCD IK solver node name for an IK bone.

        ``ccdIKSolverNode`` is a DG node (no DAG path for disambiguation),
        so the model name prefix is needed for uniqueness across imports.
        """
        model_name = self.get_model_name()
        bone_name = self.get_bone_name(bone_index)
        desired_name = f"{model_name}_{bone_name}{IK_SOLVER_SUFFIX}"
        return self.make_unique(desired_name)

    # ------------------#
    # ---Blendshapes----#
    # ------------------#
    def get_blendshape_target_name(self, morph_index: int) -> str:
        """Get unique blendshape target name."""
        if morph_index in self._blendshape_name_map:
            morph_name = self._blendshape_name_map[morph_index]
            return self.make_unique(morph_name)

        # Fallback for unexpected indices
        base_name = f"Morph_{morph_index}"
        return self.make_unique(base_name)

    def get_blendshape_node_name(self) -> str:
        """Get unique blendshape deformer node name."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_BlendShape"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_BlendShape"
        else:
            desired_name = "BlendShape"

        return self.make_unique(desired_name)

    def get_bone_morph_node_name(self) -> str:
        """Get unique bone morph node name."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_BoneMorph"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_BoneMorph"
        else:
            desired_name = "BoneMorph"

        return self.make_unique(desired_name)

    # ----------------------------#
    # ---Rigid bodies (native)---#
    # ----------------------------#
    def get_rigid_bodies_group_name(self) -> str:
        """Get unique rigid bodies group name (holds the pmxRigidBodyNode solver)."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_RigidBodies"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_RigidBodies"
        else:
            desired_name = "RigidBodies"

        return self.make_unique(desired_name)

    def get_rigid_body_solver_name(self) -> str:
        """Get unique pmxRigidBodyNode solver shape name for this model."""
        if self.model_name_local:
            desired_name = f"{self.model_name_local}_RigidBodySolver"
        elif self.model_name_universal:
            desired_name = f"{self.model_name_universal}_RigidBodySolver"
        else:
            desired_name = "RigidBodySolver"

        return self.make_unique(desired_name)
