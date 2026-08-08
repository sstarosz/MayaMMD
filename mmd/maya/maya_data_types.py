"""Naming conventions:
Model have local name and universal name (both strings).
We should use local name if available and possible, because maya don't support unicode well.
If we can't use local name (e.g. invalid characters), we can fallback to universal name.
If neither is available, we can use default names like "PMX_Root" or "PMX_Mesh".

Each Maya object created from PMX data should be named accordingly.
Each object should have PMX_ prefix to avoid name clashes.

- Root transform: use PMX_ + model local name or "PMX_Root"
- Mesh transform: use model local name or "PMX_Mesh"
- Mesh shape: use PMX_Mesh_Shape

- Materials: use PMX_ + material local name or PMX_Material_{index}
- ShadingEngines: use PMX_ + material local name + _SG or PMX_Material_{index}_SG


"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import maya.api.OpenMaya as om


@dataclass
class MayaPmxData:
    root_obj: om.MObject
    mesh_node: Optional[om.MObject] = None
    joints: List[om.MObject] = field(default_factory=list)
    skin_cluster: Optional[om.MObject] = None
    bone_name_map: Dict[str, str] = field(
        default_factory=dict
    )  # PMX bone name -> Maya joint name
    morph_name_map: Dict[str, str] = field(
        default_factory=dict
    )  # PMX morph name -> Maya blend shape target name
    root_name: str = ""  # Name of the root transform node
    mesh_name: str = ""  # Name of the mesh transform node
    bone_morph_node_name: str = ""  # Name of the boneMorphNode (empty if none created)
    blend_shape_node_name: str = (
        ""  # Name of the blendShape node (empty if none created)
    )
    ik_handles: List[str] = field(
        default_factory=list
    )  # Maya IK handle names for this model

    def to_resolved(self) -> "ResolvedModelData":
        """Convert to a lightweight, serialisation-friendly snapshot.

        Returns a :class:`ResolvedModelData` containing only strings and
        plain dicts — no ``om.MObject`` references.  This is the currency
        type used by VMD/VPD/reset operations.
        """
        return ResolvedModelData(
            root_name=self.root_name,
            bone_map=self.bone_name_map,
            morph_map=self.morph_name_map,
            blend_shape_node=self.blend_shape_node_name,
            bone_morph_node=self.bone_morph_node_name,
            ik_handles=self.ik_handles,
        )


@dataclass
class ResolvedModelData:
    """Lightweight, serialisation-friendly bundle of model data resolved from
    the Maya scene at a point in time.

    Unlike :class:`MayaPmxData` (which carries `om.MObject` references and is
    tightly coupled to the PMX import pipeline), this dataclass contains only
    strings and plain dicts.  It is the currency type passed to VMD/VPD/reset
    operations so they never need to query the scene themselves.

    Construct one via :meth:`MayaPmxData.to_resolved` or
    :meth:`mmd.maya.model_context.ModelContext.resolve`.
    """

    root_name: str = ""
    bone_map: Dict[str, str] = field(default_factory=dict)
    morph_map: Dict[str, str] = field(default_factory=dict)
    blend_shape_node: str = ""
    bone_morph_node: str = ""
    ik_handles: List[str] = field(default_factory=list)
