"""
rigid_body_builder.py — rigid bodies for PMX models.

Creates the native ``pmxPhysicsNode`` (embedded Bullet) for a PMX model.

MILESTONE (this PR): one EMPTY ``pmxPhysicsNode`` per model — the node and
its ``{model}_Physics`` group are created, and gravity is set.  The node's
``bodies`` / ``joints`` compound arrays are still empty: they are populated
through the native ``pmxRigidBody`` / ``pmxRigidBodyConstraint`` commands (a
later PR), which also wire the kinematic anchors and the Phase-3 direct
write-back into the related joints.  The node's ``time`` input is therefore
NOT connected yet — with empty bodies the solver has nothing to step, and a
``time`` connection would make compute() fail every frame.

The node is an ``MPxLocatorNode`` (a locator shape) that owns a Maya-free
Bullet world from ``mmd/core``.  It is parented under the physics group at
the origin, so the Bullet world runs in the group's local space — the same
layout the full body population later fills in.

Called from ``build_pmx_scene`` so every imported model gets its empty node.
The scene is the source of truth: the solver node name is stamped on the
model root (``pmxPhysicsNode`` string attribute) so discovery can find it
directly.

This module is part of the mmd.maya.pmx package and runs inside Autodesk Maya
(requires maya.api.OpenMaya, maya.cmds).
"""

from __future__ import annotations

import logging
from typing import Optional, Sequence

import maya.api.OpenMaya as om
import maya.cmds as cmds

from mmd.core.data_types import PmxModel
from mmd.maya.pmx_naming_manager import PMXNamingManager

log = logging.getLogger(__name__)


# ===========================================================================
# Physics binding — one native pmxPhysicsNode (embedded Bullet) per model
# ===========================================================================

_NODE_TYPE = "pmxPhysicsNode"

# Gravity — MMD's physics engine uses exactly -9.8 (Bullet's default) in the
# model's own unit scale.  We must match that: using -98 (a 10x guess) made
# EVERY force 10x too strong — the huge PMX hair masses (3276.8 at the root)
# times 10x gravity overloaded the rigid-weld constraints, so hair/skirt
# chains sagged visibly and collision pushes were 10x too violent.  -9.8
# matches MMD exactly.
_DEFAULT_GRAVITY_Y = -9.8


# ---------------------------------------------------------------------------
# Build functions — pure Maya-object creation (no class).  The scene is the
# source of truth; reconstruct handles later with the model_utils discovery
# helpers (wrapped by ModelContext.physics* getters).
# ---------------------------------------------------------------------------


def _create_physics_group(
    name_registry: PMXNamingManager, root_transform_obj: Optional[om.MObject]
) -> str:
    """Create the ``{model}_Physics`` transform group under the model root."""
    group_name = name_registry.get_physics_group_name()
    parent_name = None
    if root_transform_obj is not None and not root_transform_obj.isNull():
        try:
            parent_name = om.MFnDependencyNode(root_transform_obj).name()
        except Exception:
            parent_name = None
    if parent_name:
        return cmds.createNode("transform", name=group_name, parent=parent_name)
    return cmds.createNode("transform", name=group_name)


def _create_physics_solver(
    name_registry: PMXNamingManager, parent_group: Optional[str] = None
) -> str:
    """Create the ``pmxPhysicsNode`` (a locator shape) under the physics group.

    The node is an ``MPxLocatorNode``: it owns the Bullet world (``mmd/core``
    Simulation) and will draw its own guide visualization through a C++ draw
    override (planned, redesigned).  It is parented under the physics group at
    the origin, so the Bullet world runs in the group's local space.

    NOTE: ``time1.outTime`` is intentionally NOT connected yet — the node's
    bodies/joints arrays are empty until the native ``pmxRigidBody`` /
    ``pmxRigidBodyConstraint`` commands land, and a ``time`` connection would
    make compute() fail every frame.  The full builder (rigid-body commands
    PR) connects it together with the body population.
    """
    solver_name = name_registry.get_physics_solver_name()
    if parent_group:
        node = cmds.createNode(_NODE_TYPE, name=solver_name, parent=parent_group)
    else:
        node = cmds.createNode(_NODE_TYPE, name=solver_name)
    cmds.setAttr(f"{node}.gravity", 0.0, _DEFAULT_GRAVITY_Y, 0.0)
    return node


def create_physics_from_pmx_data(
    pmx_data: PmxModel,
    joints: Sequence[om.MObject],
    name_registry: PMXNamingManager,
    root_transform_obj: Optional[om.MObject] = None,
) -> Optional[str]:
    """Create the physics graph for a PMX model (no in-memory handle).

    MILESTONE: creates the ``{model}_Physics`` group and one EMPTY
    ``pmxPhysicsNode`` solver under it — per model.  The ``bodies`` /
    ``joints`` arrays are populated through the native ``pmxRigidBody`` /
    ``pmxRigidBodyConstraint`` commands in a later PR, which also wires the
    kinematic anchors, the write-back into the joints and the ``time``
    connection.

    Args:
        pmx_data:            Parsed PMX model (bodies/joints used by the
                             full builder in the rigid-body commands PR).
        joints:              Joint MObjects in PMX bone order (from bone
                             builder; used by the full builder).
        name_registry:       Naming manager for unique names.
        root_transform_obj:  MObject the physics group is parented under.

    Returns:
        The solver node name (the caller stamps it on the model root), or
        ``None`` if the node could not be created.
    """
    group = _create_physics_group(name_registry, root_transform_obj)
    # The solver is a locator shape parented under the physics group — its
    # object space is the group's local space, which is the Bullet world frame.
    try:
        return _create_physics_solver(name_registry, parent_group=group)
    except Exception as e:  # pragma: no cover - Maya-side failure path
        log.warning(
            "Could not create physics node for %s: %s",
            name_registry.get_model_name(),
            e,
        )
        return None
