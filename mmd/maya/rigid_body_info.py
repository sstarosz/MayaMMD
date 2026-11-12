"""Read-only snapshot of a model's PMX rigid bodies and joints.

The panel never re-reads the ``.pmx`` file: after import the *scene* holds the
authoritative data — one ``pmxRigidBodyShape`` per rigid body and one
``pmxRigidBodyJoint`` per constraint, each carrying every PMX field as an
attribute.  Reading the scene means the panel stays truthful after hand edits
made in the Attribute Editor or the Node Editor, after a scene reload, and for
models imported long ago.

Node shape
----------
Both node types are **shapes**: the data lives on the locator shape
(``pmxRigidBodyShape1``, ``pmxRigidBodyJoint9``) and the parent transform is
the guide / joint frame.  The solver's ``bodyShapes[]`` / ``jointShapes[]``
message arrays connect to the shapes, so :attr:`RigidBodyInfo.node` is the
shape (the node whose Attribute-Editor template shows the data) while
:attr:`RigidBodyInfo.guide` is its parent — the DAG node the viewport selects.
Both are matched when mirroring the Maya selection.

This module turns those nodes into plain frozen dataclasses, so the UI needs
neither ``cmds`` nor Qt and the whole read path can be unit-tested against a
faked scene.

Ordering comes from the solver's arrays, which the importer fills in PMX order
— the index MMD users see in PMX Editor.  A scene whose solver was deleted
still lists its nodes (they are then collected from the DAG, ordered by path).

Derived facts mirror the native core, so the panel explains exactly what the
viewport draws and the solver does:

* :class:`JointStatus` — ``RigidBodySimulation::jointStatus``;
* :class:`JointFreedom` — ``joint_display::describe`` / ``summarize`` (the
  joint hub's colour, in words).
"""

from __future__ import annotations

import logging
import math
import struct
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from enum import Enum
from typing import Any

from maya import cmds

log = logging.getLogger(__name__)

TYPE_SOLVER = "pmxRigidBodyNode"
TYPE_BODY_SHAPE = "pmxRigidBodyShape"
TYPE_JOINT = "pmxRigidBodyJoint"

COLLISION_GROUP_COUNT = 16
"""PMX collision groups, and therefore the number of ``bodyMaskGroup`` flags."""

FOLLOW_BONE = 0
"""``PhysicsMode`` value of a kinematic body (``eFollowBone``)."""

Vec3 = tuple[float, float, float]


# ── Bullet range model ─────────────────────────────────────────────────────


class RangeKind(Enum):
    """How Bullet treats one limit axis (``joint_display::RangeKind``)."""

    FIXED = "locked"
    LIMITED = "limited"
    FREE = "free"


def _float32(value: float) -> float:
    """``value`` at Bullet's ``btScalar`` precision (a 32-bit float).

    Limits are compared at that precision, or two values one float apart
    would read as a tiny range here and as locked in the solver.
    """
    try:
        return struct.unpack("<f", struct.pack("<f", float(value)))[0]
    except (OverflowError, TypeError, ValueError, struct.error):
        return float(value)


_F32_PI = _float32(math.pi)
_F32_TWO_PI = _float32(2.0 * math.pi)


def _normalize_angle(radians: float) -> float:
    """Bullet's ``btNormalizeAngle``: wrap to (-pi, pi] at float precision."""
    wrapped = math.fmod(_float32(radians), _F32_TWO_PI)
    if wrapped < -_F32_PI:
        wrapped = _float32(wrapped + _F32_TWO_PI)
    elif wrapped > _F32_PI:
        wrapped = _float32(wrapped - _F32_TWO_PI)
    return wrapped


def range_kind(lower: float, upper: float) -> RangeKind:
    """Classify one limit axis the way ``btGeneric6DofSpringConstraint`` does.

    Equal endpoints are **locked**, a lower bound above the upper bound means
    **free** (no limit at all), and anything in between is **limited** — even a
    range a hair wide.
    """
    low, high = _float32(lower), _float32(upper)
    if low > high:
        return RangeKind.FREE
    if low == high:
        return RangeKind.FIXED
    return RangeKind.LIMITED


def angle_to_radians(value: float, unit: str) -> float:
    """Convert a scene-angle value to radians (Maya only has deg and rad)."""
    return value if unit.lower().startswith("rad") else math.radians(value)


@dataclass(frozen=True)
class AxisRange:
    """One limit axis: its bounds, how Bullet reads them, and its spring."""

    lower: float = 0.0
    upper: float = 0.0
    kind: RangeKind = RangeKind.FIXED
    spring: bool = False
    """A spring acts on this axis (non-zero stiffness on a movable axis)."""


@dataclass(frozen=True)
class JointFreedom:
    """What one joint permits, per axis (``joint_display::Ranges``)."""

    linear: tuple[AxisRange, AxisRange, AxisRange] = (
        AxisRange(),
        AxisRange(),
        AxisRange(),
    )
    angular: tuple[AxisRange, AxisRange, AxisRange] = (
        AxisRange(),
        AxisRange(),
        AxisRange(),
    )

    @property
    def translates(self) -> bool:
        """Some linear axis can move."""
        return any(axis.kind is not RangeKind.FIXED for axis in self.linear)

    @property
    def rotates(self) -> bool:
        """Some angular axis can turn."""
        return any(axis.kind is not RangeKind.FIXED for axis in self.angular)

    @property
    def summary(self) -> str:
        """The joint hub's colour, in words: what kind of motion it allows."""
        if self.translates and self.rotates:
            return "rotates and translates"
        if self.rotates:
            return "rotates only"
        if self.translates:
            return "translates only"
        return "welded"


def _axis_range(
    lower: float, upper: float, stiffness: float, angular: bool
) -> AxisRange:
    """One axis of :func:`joint_freedom`.

    ``lower`` / ``upper`` are already in the unit the comparison needs —
    radians for an angular axis, scene units for a linear one.
    """
    low, high = (
        (_normalize_angle(lower), _normalize_angle(upper))
        if angular
        else (lower, upper)
    )
    kind = range_kind(low, high)
    # A spring on a locked axis has no effect in Bullet, so it is not reported.
    spring = stiffness != 0.0 and kind is not RangeKind.FIXED
    return AxisRange(low, high, kind, spring)


def joint_freedom(
    linear_min: Vec3,
    linear_max: Vec3,
    angular_min: Vec3,
    angular_max: Vec3,
    linear_stiffness: Vec3 = (0.0, 0.0, 0.0),
    angular_stiffness: Vec3 = (0.0, 0.0, 0.0),
) -> JointFreedom:
    """Describe a Spring6Dof joint's per-axis freedom.

    ``angular_*`` are taken in **radians** — the unit the core and Bullet work
    in — while the scene stores them in the user's angle unit.
    """
    return JointFreedom(
        linear=(
            _axis_range(linear_min[0], linear_max[0], linear_stiffness[0], False),
            _axis_range(linear_min[1], linear_max[1], linear_stiffness[1], False),
            _axis_range(linear_min[2], linear_max[2], linear_stiffness[2], False),
        ),
        angular=(
            _axis_range(angular_min[0], angular_max[0], angular_stiffness[0], True),
            _axis_range(angular_min[1], angular_max[1], angular_stiffness[1], True),
            _axis_range(angular_min[2], angular_max[2], angular_stiffness[2], True),
        ),
    )


def _to_radians(values: Vec3, unit: str) -> Vec3:
    """A scene-unit angle triple in radians."""
    return (
        angle_to_radians(values[0], unit),
        angle_to_radians(values[1], unit),
        angle_to_radians(values[2], unit),
    )


class JointStatus(Enum):
    """Whether a joint constrains anything (``RigidBodySimulation::JointStatus``)."""

    ACTIVE = "active"
    MISSING_BODY = "missing body"
    SAME_BODY = "same body"
    DISABLED_BODY = "disabled body"
    BOTH_KINEMATIC = "both follow bone"

    @property
    def is_active(self) -> bool:
        """``True`` when the joint actually constrains its two bodies."""
        return self is JointStatus.ACTIVE


# ── Scene data ─────────────────────────────────────────────────────────────


def _display_name(local: str, universal: str, prefix: str, index: int) -> str:
    """Local name, else universal name, else a stable ``{prefix}_{index}``."""
    return local or universal or f"{prefix}_{index}"


@dataclass(frozen=True)
class RigidBodyInfo:
    """One PMX rigid body, read from its ``pmxRigidBodyShape`` node."""

    index: int = 0
    """PMX index — the body's slot in the solver's ``bodyShapes[]`` array."""
    node: str = ""
    """Full DAG path of the data node (the ``pmxRigidBodyShape``)."""
    guide: str = ""
    """Full DAG path of the guide transform that carries the collider."""
    name_local: str = ""
    name_universal: str = ""
    bone: str = ""
    """Short name of the bone this body follows (``""`` = no related bone)."""
    group: int = 0
    collider_type: int = 0
    collider_label: str = ""
    shape_size: Vec3 = (0.0, 0.0, 0.0)
    rest_translate: Vec3 = (0.0, 0.0, 0.0)
    rest_rotate: Vec3 = (0.0, 0.0, 0.0)
    physics_mode: int = FOLLOW_BONE
    physics_mode_label: str = ""
    enabled: bool = True
    mass: float = 0.0
    linear_damping: float = 0.0
    angular_damping: float = 0.0
    restitution: float = 0.0
    friction: float = 0.0
    collision_mask: tuple[bool, ...] = ()
    """One flag per collision group: ``True`` = collides with that group."""
    draw_mode: int = 0
    draw_mode_label: str = ""

    @property
    def name(self) -> str:
        """Best display name: local, else universal, else ``RigidBody_{index}``."""
        return _display_name(
            self.name_local, self.name_universal, "RigidBody", self.index
        )

    @property
    def is_kinematic(self) -> bool:
        """``True`` when the body follows its bone instead of the simulation."""
        return self.physics_mode == FOLLOW_BONE

    @property
    def collides_with(self) -> tuple[int, ...]:
        """Collision-group indices this body collides with."""
        return tuple(i for i, hit in enumerate(self.collision_mask) if hit)

    @property
    def ignored_groups(self) -> tuple[int, ...]:
        """Collision-group indices this body ignores."""
        return tuple(i for i, hit in enumerate(self.collision_mask) if not hit)

    @property
    def paths(self) -> tuple[str, ...]:
        """Every DAG node this row stands for (data node first)."""
        return (self.node, self.guide)


@dataclass(frozen=True)
class JointInfo:
    """One PMX joint (constraint), read from its ``pmxRigidBodyJoint`` node."""

    index: int = 0
    """PMX index — the joint's slot in the solver's ``jointShapes[]`` array."""
    node: str = ""
    """Full DAG path of the data node (the ``pmxRigidBodyJoint``)."""
    guide: str = ""
    """Full DAG path of the transform that holds the joint frame."""
    name_local: str = ""
    name_universal: str = ""
    body_a_index: int = -1
    body_a: str = ""
    """Display name of body A (``""`` = the joint names no body)."""
    body_a_node: str = ""
    body_b_index: int = -1
    body_b: str = ""
    body_b_node: str = ""
    linear_min: Vec3 = (0.0, 0.0, 0.0)
    linear_max: Vec3 = (0.0, 0.0, 0.0)
    angular_min: Vec3 = (0.0, 0.0, 0.0)
    """Angular lower limits in the scene's angle unit (degrees by default)."""
    angular_max: Vec3 = (0.0, 0.0, 0.0)
    linear_stiffness: Vec3 = (0.0, 0.0, 0.0)
    angular_stiffness: Vec3 = (0.0, 0.0, 0.0)
    angle_unit: str = "deg"
    draw_mode: int = 0
    draw_mode_label: str = ""
    status: JointStatus = JointStatus.ACTIVE

    @property
    def name(self) -> str:
        """Best display name: local, else universal, else ``Joint_{index}``."""
        return _display_name(self.name_local, self.name_universal, "Joint", self.index)

    @property
    def freedom(self) -> JointFreedom:
        """What the joint permits, per axis — computed, never stored."""
        return joint_freedom(
            self.linear_min,
            self.linear_max,
            _to_radians(self.angular_min, self.angle_unit),
            _to_radians(self.angular_max, self.angle_unit),
            self.linear_stiffness,
            self.angular_stiffness,
        )

    @property
    def paths(self) -> tuple[str, ...]:
        """Every DAG node this row stands for (data node first)."""
        return (self.node, self.guide)


@dataclass(frozen=True)
class PhysicsInfo:
    """Everything the Physics section shows for one model."""

    root: str = ""
    """Full DAG path of the model root (``""`` = no model targeted)."""
    solver: str = ""
    bodies: tuple[RigidBodyInfo, ...] = ()
    joints: tuple[JointInfo, ...] = ()
    angle_unit: str = "deg"
    """The scene's current angle unit, for the limit rows."""
    warnings: tuple[str, ...] = ()
    """Problems hit while reading (logged too; shown in the panel footer)."""

    @property
    def counts(self) -> tuple[int, int]:
        """``(rigid bodies, joints)``."""
        return len(self.bodies), len(self.joints)

    @property
    def is_empty(self) -> bool:
        """``True`` when the model has no rigid bodies and no joints."""
        return not self.bodies and not self.joints

    def body(self, index: int) -> RigidBodyInfo | None:
        """The body at PMX ``index``, or ``None``."""
        return next((body for body in self.bodies if body.index == index), None)

    def joint(self, index: int) -> JointInfo | None:
        """The joint at PMX ``index``, or ``None``."""
        return next((joint for joint in self.joints if joint.index == index), None)

    def row_at(self, node: str) -> RigidBodyInfo | JointInfo | None:
        """The body or joint whose data node *or* guide is ``node``."""
        for body in self.bodies:
            if node in body.paths:
                return body
        for joint in self.joints:
            if node in joint.paths:
                return joint
        return None


# ── Safe scene access ──────────────────────────────────────────────────────


def _safe(query: Callable[[], Any], default: Any) -> Any:
    """Run a scene query, falling back to ``default``.

    Reading must never take the panel down: a node deleted mid-read, an
    attribute the user removed, or a custom node type that failed to register
    all surface here as "no value" instead of an exception.
    """
    try:
        return query()
    except Exception:  # noqa: BLE001 - a scene read must never raise
        return default


def _exists(node: str) -> bool:
    return bool(_safe(lambda: cmds.objExists(node), False))


def _angle_unit() -> str:
    """The scene's current angle unit (``deg`` unless the user changed it)."""
    return str(_safe(lambda: cmds.currentUnit(query=True, angle=True), "deg"))


def _parent_path(node: str) -> str:
    """``node``'s parent path (``""`` when it has none / is not a DAG path)."""
    return node.rsplit("|", 1)[0] if "|" in node else ""


def _descendants(root_name: str, node_type: str) -> list[str]:
    """Full paths of ``root_name``'s ``node_type`` descendants."""
    return list(
        _safe(
            lambda: cmds.listRelatives(
                root_name, allDescendents=True, type=node_type, fullPath=True
            ),
            [],
        )
        or []
    )


def _source_of(plug: str) -> str:
    """Source node short name feeding ``plug``, or ``""``.

    ``plugs=True`` is required: on a ``message`` connection Maya reports the
    connected node's **transform** when the plug is not requested, which is the
    guide rather than the data shape.  Querying one plug at a time is
    deliberate — ``listConnections`` with ``-connections`` returns its pairs in
    an order that is not documented and differs between runs, which would
    silently mis-pair array elements.
    """
    sources = (
        _safe(
            lambda: cmds.listConnections(
                plug, source=True, destination=False, plugs=True
            ),
            [],
        )
        or []
    )
    return str(sources[0]).rpartition(".")[0] if sources else ""


def _long_paths(names: Sequence[str]) -> list[str]:
    """Long DAG paths for ``names``, in the same order, in one call.

    Match full paths and Maya's shortest unique suffixes. An ambiguous or
    missing name resolves to an empty string, never an arbitrary sibling.
    """
    if not names:
        return []
    nonempty = [name for name in names if name]
    if not nonempty:
        return [""] * len(names)
    found = _safe(lambda: cmds.ls(nonempty, long=True), []) or []
    by_suffix: dict[str, set[str]] = {}
    for path in set(map(str, found)):
        parts = path.split("|")
        for start in range(len(parts)):
            by_suffix.setdefault("|".join(parts[start:]), set()).add(path)
    return [
        next(iter(by_suffix[name])) if len(by_suffix.get(name, ())) == 1 else ""
        for name in names
    ]


def _type_of(node: str) -> str:
    """One node's type, or ``""``."""
    return str(_safe(lambda: cmds.nodeType(node), ""))


def _types_of(nodes: Sequence[str]) -> list[str]:
    """Node types for ``nodes``, in the same order, in one call."""
    if not nodes:
        return []
    found = _safe(lambda: list(cmds.nodeType(list(nodes))), None)
    if found is not None and len(found) == len(nodes):
        return [str(node_type) for node_type in found]
    return [_type_of(node) for node in nodes]


_ENUM_FIELDS: dict[tuple[str, str], dict[int, str]] = {}


def enum_fields(node: str, attr: str) -> dict[int, str]:
    """``{value: name}`` from the node type's own enum definition.

    Maya reports the field names as one colon-separated string and writes the
    numeric value only for fields that need it (``Box=1:Sphere:Capsule``), so
    values are filled in sequentially.  Cached per node type: a model can carry
    hundreds of bodies and the definition is per type.  The panel itself uses
    :meth:`_Columns.labels` (``getAttr -asString``), which needs no mapping;
    this is here for the editors that will write these enums back.
    """
    node_type = str(_safe(lambda: cmds.nodeType(node), ""))
    if not node_type:
        return {}
    key = (node_type, attr)
    if key in _ENUM_FIELDS:
        return _ENUM_FIELDS[key]
    entries = (
        _safe(lambda: cmds.attributeQuery(attr, node=node, listEnum=True), []) or []
    )
    fields: dict[int, str] = {}
    value = 0
    for entry in str(entries[0]).split(":") if entries else []:
        name, sep, explicit = entry.partition("=")
        if sep and explicit.lstrip("-").isdigit():
            value = int(explicit)
        fields[value] = name
        value += 1
    _ENUM_FIELDS[key] = fields
    return fields


def clear_enum_cache() -> None:
    """Drop the cached enum fields (after a plugin reload, or in tests)."""
    _ENUM_FIELDS.clear()


# ── Bulk attribute reads ───────────────────────────────────────────────────


class _Columns:
    """Attribute columns for a list of same-typed nodes.

    One ``getAttr`` per *attribute* instead of one per node: a model easily
    carries 700 physics nodes, and reading ~30 fields each would cost tens of
    thousands of round trips per refresh.
    """

    def __init__(self, nodes: Sequence[str]) -> None:
        self._nodes = list(nodes)
        self._values: dict[tuple[str, bool], list[Any]] = {}

    def __len__(self) -> int:
        return len(self._nodes)

    def _read(self, attr: str, as_string: bool = False) -> list[Any]:
        key = (attr, as_string)
        if key not in self._values:
            self._values[key] = self._fetch(attr, as_string)
        return self._values[key]

    def _fetch(self, attr: str, as_string: bool) -> list[Any]:
        plugs = [f"{node}.{attr}" for node in self._nodes]
        if not plugs:
            return []
        if len(plugs) == 1:
            # Maya unwraps a single requested attribute, including strings
            # and compounds. Keep it as one column entry, not an iterable.
            return [_read_plug(plugs[0], as_string)]
        values: list[Any] | None = _safe(
            lambda: list(cmds.getAttr(plugs, asString=as_string)), None
        )
        if values is not None:
            return values
        # One node missing the attribute fails the whole batch; read the rest
        # individually so a partial scene still shows everything it can.
        return [_read_plug(plug, as_string) for plug in plugs]

    def numbers(self, attr: str, default: float = 0.0) -> list[float]:
        return [_as_float(value, default) for value in self._read(attr)]

    def ints(self, attr: str, default: int = 0) -> list[int]:
        return [_as_int(value, default) for value in self._read(attr)]

    def flags(self, attr: str, default: bool = False) -> list[bool]:
        return [default if value is None else bool(value) for value in self._read(attr)]

    def texts(self, attr: str) -> list[str]:
        return ["" if value is None else str(value) for value in self._read(attr)]

    def vectors(self, attr: str) -> list[Vec3]:
        return [_as_vec3(value) for value in self._read(attr)]

    def labels(self, attr: str) -> list[str]:
        """The enum field names, straight from Maya (never a number)."""
        return ["" if value is None else str(value) for value in self._read(attr, True)]


def _read_plug(plug: str, as_string: bool) -> Any:
    """One attribute read, or ``None`` when it cannot be read."""
    return _safe(lambda: cmds.getAttr(plug, asString=as_string), None)


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_vec3(value: Any) -> Vec3:
    """A ``float3`` / ``double3`` / ``angle3`` value as a plain tuple.

    Maya reports a compound as ``[(x, y, z)]``; a child plug reports a bare
    triple, so both shapes are accepted.
    """
    if isinstance(value, (list, tuple)) and value:
        if isinstance(value[0], (list, tuple)):
            value = value[0]
        try:
            return (float(value[0]), float(value[1]), float(value[2]))
        except (TypeError, ValueError, IndexError):
            return (0.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


# ── Scene readers ──────────────────────────────────────────────────────────


def read_physics(root_name: str) -> PhysicsInfo:
    """Read ``root_name``'s rigid bodies and joints into a :class:`PhysicsInfo`.

    Never raises: a half-built scene, or one whose custom nodes are missing,
    yields whatever could be read plus a warning.
    """
    warnings: list[str] = []
    if not root_name or not _exists(root_name):
        return PhysicsInfo()

    solver = _find_solver(root_name)
    shapes = _solver_sources(solver, "bodyShapes", TYPE_BODY_SHAPE)
    joint_nodes = _solver_sources(solver, "jointShapes", TYPE_JOINT)

    if not solver:
        shapes = _dag_nodes(root_name, TYPE_BODY_SHAPE)
        joint_nodes = _dag_nodes(root_name, TYPE_JOINT)
        if shapes or joint_nodes:
            warnings.append("No pmxRigidBodyNode: listing physics nodes from the DAG")
    elif not shapes or not joint_nodes:
        # The nodes exist but are not (all) wired into the solver: show what
        # the DAG has, ordered by path, rather than hiding the model's physics.
        loose_bodies = _dag_nodes(root_name, TYPE_BODY_SHAPE)
        loose_joints = _dag_nodes(root_name, TYPE_JOINT)
        if shapes or joint_nodes or loose_bodies or loose_joints:
            shapes = shapes or loose_bodies
            joint_nodes = joint_nodes or loose_joints
            warnings.append("Some physics nodes are not wired into the solver")

    angle_unit = _angle_unit()
    bodies = _read_bodies(shapes, warnings)
    by_node: dict[str, RigidBodyInfo] = {}
    for body in bodies:
        by_node[body.node] = body
        by_node[body.node.rsplit("|", 1)[-1]] = body
    joints = _read_joints(joint_nodes, by_node, angle_unit, warnings)
    for warning in warnings:
        log.debug("Physics read (%s): %s", root_name, warning)

    return PhysicsInfo(
        root=root_name,
        solver=solver,
        bodies=tuple(bodies),
        joints=tuple(joints),
        angle_unit=angle_unit,
        warnings=tuple(warnings),
    )


def read_rigid_bodies(root_name: str) -> list[RigidBodyInfo]:
    """Rigid bodies of ``root_name``, in PMX order."""
    return list(read_physics(root_name).bodies)


def read_joints(root_name: str) -> list[JointInfo]:
    """Joints of ``root_name``, in PMX order."""
    return list(read_physics(root_name).joints)


def _find_solver(root_name: str) -> str:
    """Full path of the model's ``pmxRigidBodyNode`` solver, or ``""``."""
    found = _descendants(root_name, TYPE_SOLVER)
    return found[0] if found else ""


def _solver_sources(solver: str, attr: str, node_type: str) -> list[tuple[int, str]]:
    """Sources wired into ``solver.<attr>[n]``, in array-index (PMX) order."""
    if not solver:
        return []
    indices = (
        _safe(lambda: cmds.getAttr(f"{solver}.{attr}", multiIndices=True), []) or []
    )
    sources = [
        (int(index), _source_of(f"{solver}.{attr}[{int(index)}]"))
        for index in sorted(indices)
    ]
    sources = [(index, name) for index, name in sources if name]
    paths = _long_paths([name for _, name in sources])
    return [
        (index, path)
        for (index, _), path, node_type_of in zip(
            sources, paths, _types_of(paths), strict=True
        )
        if node_type_of == node_type
    ]


def _dag_nodes(root_name: str, node_type: str) -> list[tuple[int, str]]:
    """Descendants of ``root_name`` of ``node_type``, ordered by path."""
    return list(enumerate(sorted(_descendants(root_name, node_type))))


def _read_bodies(
    sources: Sequence[tuple[int, str]], warnings: list[str]
) -> list[RigidBodyInfo]:
    shapes = [path for _, path in sources]
    if not shapes:
        return []
    columns = _Columns(shapes)
    bones = [_source_of(f"{shape}.bodyJoint") for shape in shapes]
    try:
        # Each column is materialized ONCE and then indexed: reading it inside
        # the loop would rebuild the whole column for every node (O(n^2)).
        name_local = columns.texts("bodyNameLocal")
        name_universal = columns.texts("bodyNameUniversal")
        groups = columns.ints("bodyGroupId")
        collider_type = columns.ints("bodyColliderType")
        collider_label = columns.labels("bodyColliderType")
        shape_size = columns.vectors("bodyShapeSize")
        rest_translate = columns.vectors("bodyRestTranslate")
        rest_rotate = columns.vectors("bodyRestRotate")
        physics_mode = columns.ints("bodyPhysicsMode")
        physics_mode_label = columns.labels("bodyPhysicsMode")
        enabled = columns.flags("bodyEnabled", True)
        mass = columns.numbers("bodyMass")
        linear_damping = columns.numbers("bodyLinearDamping")
        angular_damping = columns.numbers("bodyAngularDamping")
        restitution = columns.numbers("bodyRestitution")
        friction = columns.numbers("bodyFriction")
        masks = list(
            zip(
                *(
                    columns.flags(f"bodyMaskGroup{group}", True)
                    for group in range(COLLISION_GROUP_COUNT)
                ),
                strict=False,
            )
        )
        draw_mode = columns.ints("drawMode")
        draw_mode_label = columns.labels("drawMode")
        return [
            RigidBodyInfo(
                index=sources[index][0],
                node=shape,
                guide=_parent_path(shape),
                name_local=name_local[index],
                name_universal=name_universal[index],
                bone=bones[index],
                group=groups[index],
                collider_type=collider_type[index],
                collider_label=collider_label[index],
                shape_size=shape_size[index],
                rest_translate=rest_translate[index],
                rest_rotate=rest_rotate[index],
                physics_mode=physics_mode[index],
                physics_mode_label=physics_mode_label[index],
                enabled=enabled[index],
                mass=mass[index],
                linear_damping=linear_damping[index],
                angular_damping=angular_damping[index],
                restitution=restitution[index],
                friction=friction[index],
                collision_mask=masks[index],
                draw_mode=draw_mode[index],
                draw_mode_label=draw_mode_label[index],
            )
            for index, shape in enumerate(shapes)
        ]
    except Exception as exc:  # noqa: BLE001 - a broken scene must not raise
        warnings.append(f"Could not read the rigid bodies: {exc}")
        return []


def _read_joints(
    sources: Sequence[tuple[int, str]],
    bodies_by_node: dict[str, RigidBodyInfo],
    angle_unit: str,
    warnings: list[str],
) -> list[JointInfo]:
    joint_nodes = [path for _, path in sources]
    if not joint_nodes:
        return []
    columns = _Columns(joint_nodes)
    try:
        # One read per column, then index — see _read_bodies.
        ends = [
            (
                _source_of(f"{node}.jointBodyA"),
                _source_of(f"{node}.jointBodyB"),
            )
            for node in joint_nodes
        ]
        canonical_ends = _long_paths([node for pair in ends for node in pair])
        ends = list(zip(canonical_ends[::2], canonical_ends[1::2], strict=True))
        name_local = columns.texts("jointNameLocal")
        name_universal = columns.texts("jointNameUniversal")
        linear_min = columns.vectors("jointLinearMin")
        linear_max = columns.vectors("jointLinearMax")
        angular_min = columns.vectors("jointAngularMin")
        angular_max = columns.vectors("jointAngularMax")
        linear_stiffness = columns.vectors("jointLinearStiffness")
        angular_stiffness = columns.vectors("jointAngularStiffness")
        draw_mode = columns.ints("drawMode")
        draw_mode_label = columns.labels("drawMode")
        joints: list[JointInfo] = []
        for index, node in enumerate(joint_nodes):
            body_a = bodies_by_node.get(ends[index][0])
            body_b = bodies_by_node.get(ends[index][1])
            joints.append(
                JointInfo(
                    index=sources[index][0],
                    node=node,
                    guide=_parent_path(node),
                    name_local=name_local[index],
                    name_universal=name_universal[index],
                    body_a_index=body_a.index if body_a else -1,
                    body_a=body_a.name if body_a else "",
                    body_a_node=body_a.node if body_a else "",
                    body_b_index=body_b.index if body_b else -1,
                    body_b=body_b.name if body_b else "",
                    body_b_node=body_b.node if body_b else "",
                    linear_min=linear_min[index],
                    linear_max=linear_max[index],
                    angular_min=angular_min[index],
                    angular_max=angular_max[index],
                    linear_stiffness=linear_stiffness[index],
                    angular_stiffness=angular_stiffness[index],
                    angle_unit=angle_unit,
                    draw_mode=draw_mode[index],
                    draw_mode_label=draw_mode_label[index],
                    status=_joint_status(body_a, body_b),
                )
            )
        return joints
    except Exception as exc:  # noqa: BLE001 - a broken scene must not raise
        warnings.append(f"Could not read the joints: {exc}")
        return []


def refresh_joint_bodies(info: PhysicsInfo) -> PhysicsInfo:
    """Recompute cached joint endpoints and status from this model's bodies."""
    bodies = {body.node: body for body in info.bodies}
    joints = []
    for joint in info.joints:
        body_a = bodies.get(joint.body_a_node)
        body_b = bodies.get(joint.body_b_node)
        joints.append(replace(
            joint,
            body_a_index=body_a.index if body_a else -1,
            body_a=body_a.name if body_a else "",
            body_b_index=body_b.index if body_b else -1,
            body_b=body_b.name if body_b else "",
            status=_joint_status(body_a, body_b),
        ))
    return replace(info, joints=tuple(joints))


def _joint_status(
    body_a: RigidBodyInfo | None, body_b: RigidBodyInfo | None
) -> JointStatus:
    """Whether a joint constrains anything, in ``jointStatus``' check order."""
    if body_a is None or body_b is None:
        return JointStatus.MISSING_BODY
    if body_a.node == body_b.node:
        return JointStatus.SAME_BODY
    if not body_a.enabled or not body_b.enabled:
        return JointStatus.DISABLED_BODY
    if body_a.is_kinematic and body_b.is_kinematic:
        return JointStatus.BOTH_KINEMATIC
    return JointStatus.ACTIVE
