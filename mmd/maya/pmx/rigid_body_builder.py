"""
rigid_body_builder.py

Responsible for creating rigid body visual guides from PMX rigid body data.

This is Phase 1 of the physics plan (see ``docs/PhysicsImplementation.md``):
guides show where physics colliders should be, color-coded by collision
``group_id``, without implementing full physics simulation.  The module also
owns the MMD → Maya coordinate conversions shared by the future physics
binding phase (Z-flip positions, ``(-rx, -ry, +rz)`` handedness rotation).

This module is part of the mmd.maya.pmx package and is designed to run inside
Autodesk Maya (requires maya.api.OpenMaya, maya.cmds, maya.mel).
"""

from __future__ import annotations

import logging
import math
import traceback
from typing import Optional

import maya.api.OpenMaya as om
import maya.cmds as cmds
import maya.mel as mel

from mmd.core.data_types import PmxModel, ShapeType
from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Collision-group color palette
# ---------------------------------------------------------------------------

# Distinct viewport colors for collision groups, indexed by PMX group_id (0-15).
# Four-bit group ids are the MMD convention; ids beyond 15 wrap around.
# Groups 0-7 use the classic rainbow (maximally distinct for the groups most
# models actually use); 8-15 extend it with clearly different hues/lightness so
# every group id has a unique, recognizable color.
_RIGID_BODY_GROUP_COLORS: tuple[tuple[float, float, float], ...] = (
    (0.90, 0.10, 0.10),   # 0  red
    (0.10, 0.75, 0.15),   # 1  green
    (0.15, 0.35, 0.95),   # 2  blue
    (1.00, 0.90, 0.10),   # 3  yellow
    (0.95, 0.15, 0.65),   # 4  magenta
    (0.00, 0.85, 0.90),   # 5  cyan
    (1.00, 0.55, 0.10),   # 6  orange
    (0.50, 0.10, 0.90),   # 7  purple
    (0.60, 0.90, 0.10),   # 8  lime
    (1.00, 0.35, 0.50),   # 9  rose
    (0.40, 0.65, 0.95),   # 10 sky blue
    (1.00, 0.65, 0.80),   # 11 pink
    (0.55, 0.35, 0.15),   # 12 brown
    (0.80, 0.60, 0.95),   # 13 lavender
    (0.10, 0.70, 0.55),   # 14 teal
    (0.10, 0.15, 0.50),   # 15 navy
)


def _group_color_hex(group_id: int) -> str:
    """Human-readable hex color for a collision group's palette entry."""
    r, g, b = _RIGID_BODY_GROUP_COLORS[group_id % len(_RIGID_BODY_GROUP_COLORS)]
    return f"#{int(r * 255):02X}{int(g * 255):02X}{int(b * 255):02X}"


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------


def mmd_euler_to_maya_degrees(
    rx_rad: float, ry_rad: float, rz_rad: float
) -> tuple[float, float, float]:
    """Convert MMD-space Euler angles (radians) to Maya rotate degrees.

    PMX/MMD is left-handed (+Z toward the viewer); Maya is right-handed.  The
    exact Maya-space rigid-body rotation is ``R_maya = F·R_mmd·F`` with the
    reflection ``F = diag(1, 1, -1)``, and it reproduces exactly as::

        rotateX = -rx, rotateY = -ry, rotateZ = +rz

    (verified numerically to machine precision over random rotations and
    against real rigid-body data).

    Do NOT replace this with a quaternion round-trip: Maya's
    ``MEulerRotation(x, y, z, kXYZ).asMatrix()`` is the *transpose* of the
    standard ``Rx·Ry·Rz`` product, so ``asQuaternion()`` reconstructs a negated
    rotation and produces wrong orientations after any handedness flip.

    Returns:
        Tuple of (rotateX, rotateY, rotateZ) in degrees.
    """
    return (
        math.degrees(-rx_rad),
        math.degrees(-ry_rad),
        math.degrees(rz_rad),
    )


# ---------------------------------------------------------------------------
# Guide creation
# ---------------------------------------------------------------------------


def _add_rigid_body_guide_attributes(shape_node: str, rb_idx: int, rigid_body) -> None:
    """Store PMX rigid-body metadata on a guide transform (self-describing scene).

    Mirrors the pattern documented in ``docs/CustomAttributes.md`` so collision
    guides can be targeted later by index/group without extra bookkeeping.
    """
    for attr_name, attr_type, value in (
        ("pmxRigidBodyIndex", "long", rb_idx),
        ("pmxGroupId", "long", rigid_body.group_id),
        ("pmxPhysicsMode", "string", rigid_body.physics_mode.name),
    ):
        try:
            if not cmds.attributeQuery(attr_name, node=shape_node, exists=True):
                if attr_type == "string":
                    # String attrs use dataType; numeric attrs use attributeType.
                    cmds.addAttr(shape_node, longName=attr_name, dataType="string")
                else:
                    cmds.addAttr(shape_node, longName=attr_name, attributeType=attr_type)
            if attr_type == "string":
                cmds.setAttr(f"{shape_node}.{attr_name}", value, type="string")
            else:
                cmds.setAttr(f"{shape_node}.{attr_name}", value)
        except Exception as e:
            log.debug("Could not add %s on %s: %s", attr_name, shape_node, e)


def _create_group_material(group_name: str, group_id: int) -> tuple[str, str]:
    """Create one unique shader + shading group for a collision group.

    Uses Maya 2024+'s standard surface shader ``openPBRSurface`` (colored via
    its ``baseColor`` attribute, matching the ``shadingNode -asShader
    openPBRSurface`` workflow in Maya 2026).  Falls back to a Lambert on older
    Maya releases where ``openPBRSurface`` does not exist.

    Returns:
        ``(shader_name, shading_group_name)``.
    """
    r, g, b = _RIGID_BODY_GROUP_COLORS[group_id % len(_RIGID_BODY_GROUP_COLORS)]
    shader = None
    shader_type = None
    for candidate in ("openPBRSurface", "lambert"):
        try:
            shader = cmds.shadingNode(
                candidate, asShader=True, name=f"{group_name}_Group{group_id:02d}"
            )
            shader_type = candidate
            break
        except Exception:
            continue
    if shader is None:
        raise RuntimeError("No supported surface shader node available")
    if shader_type == "openPBRSurface":
        cmds.setAttr(f"{shader}.baseColor", r, g, b, type="double3")
    else:  # lambert
        cmds.setAttr(f"{shader}.color", r, g, b, type="double3")
        cmds.setAttr(f"{shader}.diffuse", 0.8)
    sg = cmds.sets(
        name=f"{shader}SG", renderable=True, noSurfaceShader=True, empty=True
    )
    cmds.connectAttr(f"{shader}.outColor", f"{sg}.surfaceShader")
    return shader, sg


def _create_rigid_body_group_materials(
    rb_group_name: str, guides_by_group: dict[int, list[str]]
) -> dict[int, str]:
    """Create one shader per collision group and assign it to its guide shapes.

    Returns:
        Mapping of group_id -> shader name.
    """
    group_materials: dict[int, str] = {}
    for group_id in sorted(guides_by_group):
        try:
            shader, sg = _create_group_material(rb_group_name, group_id)
            for shape_node in guides_by_group[group_id]:
                cmds.sets(shape_node, edit=True, forceElement=sg)
            group_materials[group_id] = shader
        except Exception as e:
            log.warning(
                "Failed to create group material for group %d: %s", group_id, e
            )
            log.debug(traceback.format_exc())
    return group_materials


def create_rigid_body_guides_from_pmx_data(
    pmx_data: PmxModel,
    root_transform_obj,
    joints: list,
    name_registry: PMXNamingManager,
) -> Optional[om.MObject]:
    """
    Creates visual guide shapes for PMX rigid bodies.

    Phase 1 of the physics plan (see ``docs/PhysicsImplementation.md``): shows
    where physics colliders should be, color-coded by collision ``group_id``,
    without implementing full physics simulation.

    Args:
        pmx_data (PmxModel): PMX model data containing rigid bodies.
        root_transform_obj (MObject): Root transform object.
        joints (list): List of created joint MObjects.
        name_registry (PMXNamingManager): Naming manager for unique names.

    Returns:
        Optional[MObject]: The rigid body guide group object, or None if no rigid bodies.
    """
    # TODO: In v1.1, implement actual physics simulation using Maya's Bullet
    # plugin or a custom solution.
    log.info("Creating rigid body visual guides (physics simulation coming in v1.1)")

    if not pmx_data.rigid_bodies:
        log.debug("No rigid bodies found in PMX data")
        return None

    log.debug(
        "Creating rigid body visual guides for %d rigid bodies",
        len(pmx_data.rigid_bodies),
    )

    # Create group for rigid body guides
    rb_group_transform_fn = om.MFnTransform()
    rb_group_obj = rb_group_transform_fn.create(root_transform_obj)
    rb_group_name = name_registry.get_rigidbody_group_name()
    rb_group_transform_fn.setName(rb_group_name)

    # group_id -> guide shape transforms created so far (for material assignment)
    guides_by_group: dict[int, list[str]] = {}

    for rb_idx, rigid_body in enumerate(pmx_data.rigid_bodies):
        try:
            # Determine shape name
            rb_name = name_registry.get_rigidbody_name(rb_idx)
            # Create shape based on type
            shape_node = None
            shape_type = rigid_body.shape
            size = rigid_body.shape_size

            if shape_type == ShapeType.SPHERE:
                # Size.x is the radius
                shape_node = cmds.polySphere(
                    radius=size.x, subdivisionsX=12, subdivisionsY=12, name=rb_name
                )[0]

            elif shape_type == ShapeType.BOX:
                # Size components are half-extents (half width, half height, half depth)
                shape_node = cmds.polyCube(
                    width=size.x * 2, height=size.y * 2, depth=size.z * 2, name=rb_name
                )[0]

            elif shape_type == ShapeType.CAPSULE:
                # Size.x is radius, Size.y is total height (including hemispheres).
                # polyCylinder -rcp (round caps): -h is the total height incl. caps.
                # Use MEL directly - Python cmds.polyCylinder has a bug with roundCap.
                mel_cmd = (
                    f"polyCylinder -r {size.x} -h {size.y} "
                    f"-sx 12 -sh 1 -sc 12 -rcp true "
                    f'-n "{rb_name}";'
                )
                result = mel.eval(mel_cmd)
                shape_node = result[0] if isinstance(result, list) else result

            if shape_node:
                # World position FIRST (flip Z coordinate for Maya's right-handed space)
                pos = rigid_body.shape_position
                cmds.setAttr(f"{shape_node}.translateX", pos.x)
                cmds.setAttr(f"{shape_node}.translateY", pos.y)
                cmds.setAttr(f"{shape_node}.translateZ", -pos.z)

                # World rotation: handedness-correct MMD -> Maya conversion
                rot = rigid_body.shape_rotation
                rot_x, rot_y, rot_z = mmd_euler_to_maya_degrees(
                    rot.x, rot.y, rot.z
                )
                cmds.setAttr(f"{shape_node}.rotateX", rot_x)
                cmds.setAttr(f"{shape_node}.rotateY", rot_y)
                cmds.setAttr(f"{shape_node}.rotateZ", rot_z)

                # Parent to the rigid bodies group, preserving world transform
                cmds.parent(shape_node, rb_group_name, absolute=True)

                # Self-describing metadata (docs/CustomAttributes.md pattern)
                _add_rigid_body_guide_attributes(shape_node, rb_idx, rigid_body)

                guides_by_group.setdefault(rigid_body.group_id, []).append(shape_node)

        except Exception as e:
            log.warning(
                "Failed to create rigid body guide for index %d: %s", rb_idx, e
            )
            log.debug(traceback.format_exc())

    # One Lambert material per collision group so guides are color-coded
    group_materials = _create_rigid_body_group_materials(rb_group_name, guides_by_group)

    # Add informative notes to the group
    try:
        cmds.addAttr(rb_group_name, longName="rigidBodyNote", dataType="string")
        cmds.setAttr(
            f"{rb_group_name}.rigidBodyNote",
            "Visual guides only. Physics simulation coming in v1.1",
            type="string",
        )
        group_desc = ", ".join(
            f"{gid}:{_group_color_hex(gid)}" for gid in sorted(guides_by_group)
        )
        cmds.addAttr(
            rb_group_name, longName="rigidBodyGroupColors", dataType="string"
        )
        cmds.setAttr(
            f"{rb_group_name}.rigidBodyGroupColors",
            f"Collision group color map: {group_desc}",
            type="string",
        )
    except Exception as e:
        log.debug("Could not add note attribute: %s", e)

    log.info(
        "Created %d rigid body visual guides across %d collision groups. "
        "Physics simulation will be supported in a future version.",
        len(pmx_data.rigid_bodies),
        len(group_materials),
    )

    return rb_group_obj
