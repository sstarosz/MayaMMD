"""
rigid_body_builder.py — rigid bodies for PMX models.

The single rigid-body module (formerly split across the Phase-1 visual-guide
builder and physics_builder.py).  It owns:

* the MMD → Maya coordinate conversions shared by the rigid-body code
  (Z-flip positions, ``(-rx, -ry, +rz)`` handedness rotation),
* the collision-group color palette and per-group surface shaders,
* the **``PhysicsBinding``** — the binding layer that drives one native
  ``mmdPhysicsNode`` (embedded Bullet) per model: visible guide meshes for
  every rigid body, the node's ``bodies`` / ``joints`` compound arrays, the
  kinematic-anchor and dynamic-output connections, and the DG write-back
  constraints to the skeleton.

The C++ node is time-driven (``time1.outTime -> node.time``) and evaluated by
the evaluation manager on every time step; it declares itself non-cacheable so
Cached Playback always re-evaluates it (see docs/PhysicsImplementation.md).
The Bullet world advances inside the node's ``compute()`` — no solver plugin,
no scriptJob, no pairBlend, no external stateful nodes.

Run it by calling :func:`create_physics_from_pmx_data` (or pass
``build_physics=True`` to ``build_pmx_scene``).  A binding handle can be
reconstructed from an existing scene with :meth:`PhysicsBinding.from_scene`.

This module is part of the mmd.maya.pmx package and runs inside Autodesk Maya
(requires maya.api.OpenMaya, maya.cmds, maya.mel).
"""

from __future__ import annotations

import logging
import math
from typing import Optional

import maya.api.OpenMaya as om
import maya.cmds as cmds
import maya.mel as mel

from mmd.core.data_types import PhysicsMode, PmxModel, ShapeType

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
# Collision-group shaders
# ---------------------------------------------------------------------------


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


# ===========================================================================
# Physics binding — one native mmdPhysicsNode (embedded Bullet) per model
# ===========================================================================

_NODE_TYPE = "mmdPhysicsNode"

# PMX shape -> node bodyColliderType (sphere=2, box=1, capsule=3)
_PMX_TO_COLLIDER_TYPE: dict[ShapeType, int] = {
    ShapeType.SPHERE: 2,
    ShapeType.BOX: 1,
    ShapeType.CAPSULE: 3,
}

# Gravity — MMD's physics engine uses exactly -9.8 (Bullet's default) in the
# model's own unit scale.  We must match that: using -98 (a 10x guess for the
# "18-unit" Tololo model) made EVERY force 10x too strong — the huge PMX hair
# masses (3276.8 at the root) × 10x gravity overloaded the rigid-weld
# constraints, so hair/skirt chains sagged visibly (a rigid bang extended ~1.1
# units) and collision pushes were 10x too violent.  -9.8 matches MMD exactly.
_DEFAULT_GRAVITY_Y = -9.8

# Playback frames per second — the node converts Maya frame deltas to seconds
# with this.
_DEFAULT_FPS = 30.0


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


# How deeply (in model units) two cloth bodies must interpenetrate at rest for
# the cloth-on-cloth collision correction to treat them as "draped over each
# other" (e.g. bangs resting in the skirt).  A shallow touch (cape tips
# brushing the jacket back, ~0.11) must not qualify — the contact would shove
# the shared chain around.  Tololo's bangs penetrate the skirt 0.21-0.81 deep,
# the cape tips only 0.11, so 0.15 cleanly separates them.
_CLOTH_OVERLAP_PENETRATION = 0.15
# Chain-size guards for the cloth-on-cloth correction: only a SHORT chain
# (≤ _CLOTH_SMALL_CHAIN bodies) draping a LARGE sheet (≥ _CLOTH_LARGE_SHEET
# bodies) qualifies — the bangs (8) resting on the skirt (144).  This keeps the
# rule from adding collisions in models with different cloth layouts (a large
# skirt draping small belts/ribbons, long hair chains, etc.), which destabilize.
_CLOTH_SMALL_CHAIN = 10
_CLOTH_LARGE_SHEET = 50


def _approx_extent(rb) -> float:
    """Rough bounding radius of a rigid body's rest collider.

    Used by the proximity-based collision-mask correction to decide whether two
    bodies overlap at rest.  Box -> largest half-extent; sphere -> radius;
    capsule -> max(radius, cylinder-half + radius).
    """
    s = rb.shape_size
    if rb.shape == ShapeType.BOX:
        return max(s.x, s.y, s.z)
    if rb.shape == ShapeType.SPHERE:
        return s.x
    return max(s.x, s.y / 2.0 + s.x)


class PhysicsBinding:
    """Owns the ``mmdPhysicsNode`` graph for one PMX model.

    The node + ALL rigid bodies (FOLLOW_BONE kinematic guides and dynamic
    PHYSICS / PHYSICS_BONE bodies), all PMX joints, and the per-frame
    write-back from dynamic bodies to their related bones.

    Attributes:
        pmx_data:       Parsed PMX model (rigid bodies + joints).
        joints:         List of joint MObjects in PMX bone order.
        name_registry:  PMXNamingManager for unique node names.
        root_transform_obj: MObject the physics group is parented under.
        solver:         Name of the ``mmdPhysicsNode`` (alias of ``node``).
        node:           Name of the ``mmdPhysicsNode``.
        bodies:         ``{rb_index: guide_xform}`` — every rigid body created
                        (kinematic guides + dynamic guides).
        constraints:    ``{rb_index: constraint_name}`` — the DG constraint
                        binding the body to its bone (parentConstraint for
                        FOLLOW_BONE / PHYSICS, orientConstraint for
                        PHYSICS_BONE).
        physics_joints: Number of PMX joints written into the node.
    """

    def __init__(self, pmx_data, joints, name_registry, root_transform_obj=None):
        self.pmx_data = pmx_data
        self.joints = joints
        self.name_registry = name_registry
        self.root_transform_obj = root_transform_obj

        self.solver: Optional[str] = None
        self.node: Optional[str] = None
        self._group: Optional[str] = None
        self.bodies: dict[int, str] = {}
        # rb index -> DG constraint node name (binding to the bone)
        self.constraints: dict[int, str] = {}
        # number of PMX joints written into the node
        self.physics_joints: list[int] = []
        # collision group id -> (lambert shader, shading group) — one unique
        # material per group, shared by all guide meshes in that group.
        self._group_materials: dict[int, tuple[str, str]] = {}

        # bone index -> joint full path name
        self._joint_names: dict[int, str] = {}
        for b_idx, j_obj in enumerate(joints):
            if not j_obj.isNull():
                try:
                    self._joint_names[b_idx] = om.MFnDagNode(j_obj).fullPathName()
                except Exception as e:
                    log.debug("Could not resolve joint %d path: %s", b_idx, e)

    @classmethod
    def from_scene(
        cls, root_name: str, pmx_data: Optional[PmxModel] = None
    ) -> "PhysicsBinding":
        """Reconstruct a binding handle from an existing scene (no build).

        Discovers the physics subgraph of the model rooted at ``root_name``:
        the ``{model}_Physics`` group (first child transform ending in
        ``_Physics``), the ``mmdPhysicsNode`` solver (via the root's
        ``pmxPhysicsNode`` attr stamped at import, falling back to the guide
        connections), every guide mesh (via the ``pmxRigidBodyIndex`` metadata
        attr) and the DG constraint binding each guide to its bone.

        ``pmx_data`` is optional — attach it when the caller has the parsed
        model (e.g. integration tests) so the body→bone mapping
        (``related_bone_index``) and ``step()``/``write_back()`` work.

        The returned handle is for discovery/editing/headless use only; the
        interactive simulation is pure DG and needs no handle.
        """
        binding = cls.__new__(cls)
        binding.pmx_data = pmx_data
        binding.joints = []
        binding.name_registry = None
        binding.root_transform_obj = None
        binding.solver = None
        binding.node = None
        binding._group = None
        binding.bodies = {}
        binding.constraints = {}
        binding.physics_joints = []
        binding._group_materials = {}
        binding._joint_names = {}

        # Locate the physics group: first child transform ending in _Physics.
        group = None
        for child in cmds.listRelatives(root_name, children=True, type="transform") or []:
            if child.endswith("_Physics"):
                group = child
                break
        if group is None:
            return binding
        binding._group = group

        # Solver node: prefer the root's stamped pmxPhysicsNode attr, else
        # derive it from the guide connections (old scenes without the attr).
        node = None
        if cmds.attributeQuery("pmxPhysicsNode", node=root_name, exists=True):
            node = cmds.getAttr(f"{root_name}.pmxPhysicsNode")
            if not node or not cmds.objExists(node):
                node = None
        if node is None:
            for child in cmds.listRelatives(group, children=True, type="transform") or []:
                links = cmds.listConnections(child, type=_NODE_TYPE) or []
                if links:
                    node = links[0]
                    break
        binding.node = node
        binding.solver = node

        # Bodies: every guide under the group carrying pmxRigidBodyIndex.
        for child in cmds.listRelatives(group, children=True, type="transform") or []:
            if not cmds.attributeQuery("pmxRigidBodyIndex", node=child, exists=True):
                continue
            try:
                rb_idx = int(cmds.getAttr(f"{child}.pmxRigidBodyIndex"))
            except Exception:
                continue
            binding.bodies[rb_idx] = child

        # Constraints: the DG constraint(s) touching each guide.
        for rb_idx, guide in binding.bodies.items():
            cons = []
            for ctype in ("parentConstraint", "orientConstraint"):
                cons.extend(cmds.listConnections(guide, type=ctype) or [])
            if cons:
                binding.constraints[rb_idx] = cons[0]

        return binding

    # ------------------------------------------------------------------
    # Creation
    # ------------------------------------------------------------------
    def create(self) -> "PhysicsBinding":
        """Create the node, every rigid body and every PMX joint."""
        group = self._create_group()
        self._group = group
        self.node = self._create_solver()
        self.solver = self.node

        created_follow = 0
        created_dynamic = 0
        # (rb_idx -> body spec) in PMX order, plus ordered lists of the
        # kinematic guides (for the anchor inputs).
        body_specs: dict[int, dict] = {}
        kinematic_order: list[int] = []

        # Collision-mask correction, PROXIMITY-BASED.  Converted game models
        # often ship degenerate non_collision_group values (every body = "own
        # group only", e.g. the Tololo PMX: skirt mask 0x0004, legs mask
        # 0x0002), which would let the skirt pass straight through the legs.
        # But blanket-adding ALL kinematic groups to every dynamic body
        # over-broadens: e.g. the bangs (Beg, PMX mask 0x0012 = head group 1
        # only) would also get the huge torso capsule (group 7) and jitter on
        # it.  Instead, a DYNAMIC body only gains the KINEMATIC groups whose
        # colliders actually overlap its rest collider, and a KINEMATIC body
        # gains the DYNAMIC groups that overlap it — so the skirt collides
        # with the legs/hips it wraps, while the bangs keep colliding only
        # with the head they rest on.
        centers = [rb.shape_position for rb in self.pmx_data.rigid_bodies]
        extents = [_approx_extent(rb) for rb in self.pmx_data.rigid_bodies]
        kin_overlap: dict[int, int] = {}  # dynamic rb_idx -> kinematic group bits
        dyn_overlap: dict[int, int] = {}  # kinematic rb_idx -> dynamic group bits
        for i, rb in enumerate(self.pmx_data.rigid_bodies):
            bits = 0
            if rb.physics_mode == PhysicsMode.FOLLOW_BONE:
                others = (j for j, rbj in enumerate(self.pmx_data.rigid_bodies)
                          if rbj.physics_mode != PhysicsMode.FOLLOW_BONE)
                store = dyn_overlap
            else:
                others = (j for j, rbj in enumerate(self.pmx_data.rigid_bodies)
                          if rbj.physics_mode == PhysicsMode.FOLLOW_BONE)
                store = kin_overlap
            for j in others:
                rbj = self.pmx_data.rigid_bodies[j]
                dx = centers[i].x - centers[j].x
                dy = centers[i].y - centers[j].y
                dz = centers[i].z - centers[j].z
                rr = extents[i] + extents[j] + 0.2
                if dx * dx + dy * dy + dz * dz < rr * rr:
                    bits |= 1 << rbj.group_id
            store[i] = bits

        # Cloth-on-cloth correction (hair/bangs draping over the skirt/jacket).
        # The kinematic correction above only covers body↔cloth.  Converted
        # game models also ship masks where e.g. the bangs (Beg chain, anchored
        # to the torso) do NOT collide with the skirt (also anchored to the
        # torso) — so the bangs hang free, sag into the skirt and swing "under
        # the arm" instead of resting ON the jacket colliders.  Adding the
        # skirt group to the bangs (and vice-versa) fixes that.  Guards keep
        # this from creating new instabilities:
        #   1. CHAINS: connectivity uses DYNAMIC bodies only (kinematic bodies
        #      never merge chains).  Bodies jointed into the same chain (bangs
        #      vs their own tail capsules) never collide, and a cape sharing
        #      only the torso ANCHOR with the bangs is a separate chain.
        #   2. ANCHOR: two cloth chains only interact when they are jointed to
        #      the SAME body part (same collision group of their FOLLOW_BONE
        #      anchor, e.g. both anchored to the torso) — the bangs and the
        #      skirt both hang off the torso, while the sleeve hangs off the
        #      arm and the hair off the head, so bangs↔sleeve/hair are never
        #      added (two hanging chains colliding shove each other around).
        #   3. SMALL↔LARGE: only a SHORT cloth chain (≤ 10 bodies) resting on
        #      a LARGE cloth sheet (≥ 50 bodies) qualifies.  The bangs (8
        #      bodies) draping the skirt (144) is exactly this.  Everything
        #      else is excluded — a big skirt draping a small belt, two small
        #      ribbons, long hair chains, etc. — so this never adds collisions
        #      in models where the cloth layout differs (validated: adding
        #      skirt↔ribbon/belt/hair contacts there destabilizes the sim).
        #   4. DRAPE: the small chain must genuinely DRAPE the large one — some
        #      body of it must INTERPENETRATE a body of the large chain at rest
        #      (centre distance < extents sum − 0.15).  A mere touch (e.g. the
        #      cape tips brushing the jacket back 0.11 deep) does NOT qualify —
        #      that contact would shove the shared chain around and jitter the
        #      bangs.  Once a small chain drapes, EVERY body of it that
        #      overlaps the large chain gains its group (so the bangs tail,
        #      which only touches the skirt, still rests on it).
        # Own-group bits stay cleared (hair↔hair pass-through preserved).
        n_rb = len(self.pmx_data.rigid_bodies)
        parent = list(range(n_rb))

        def _find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def _union(a: int, c: int) -> None:
            ra, rc = _find(a), _find(c)
            if ra != rc:
                parent[rc] = ra

        for jn in self.pmx_data.joints:
            a, b = jn.rigid_body_index_a, jn.rigid_body_index_b
            if a < 0 or b < 0:
                continue
            ba = self.pmx_data.rigid_bodies[a]
            bb = self.pmx_data.rigid_bodies[b]
            if (ba.physics_mode != PhysicsMode.FOLLOW_BONE and
                    bb.physics_mode != PhysicsMode.FOLLOW_BONE):
                _union(a, b)  # dynamic↔dynamic joint = same cloth chain
        anchor_groups: dict[int, int] = {}  # chain root -> kinematic group bits
        chain_size: dict[int, int] = {}  # chain root -> dynamic body count
        for i, rb in enumerate(self.pmx_data.rigid_bodies):
            if rb.physics_mode == PhysicsMode.FOLLOW_BONE:
                continue
            r = _find(i)
            chain_size[r] = chain_size.get(r, 0) + 1
        for jn in self.pmx_data.joints:
            a, b = jn.rigid_body_index_a, jn.rigid_body_index_b
            if a < 0 or b < 0:
                continue
            ba = self.pmx_data.rigid_bodies[a]
            bb = self.pmx_data.rigid_bodies[b]
            if ba.physics_mode == PhysicsMode.FOLLOW_BONE and \
                    bb.physics_mode != PhysicsMode.FOLLOW_BONE:
                anchor_groups[_find(b)] = anchor_groups.get(_find(b), 0) | (1 << ba.group_id)
            elif bb.physics_mode == PhysicsMode.FOLLOW_BONE and \
                    ba.physics_mode != PhysicsMode.FOLLOW_BONE:
                anchor_groups[_find(a)] = anchor_groups.get(_find(a), 0) | (1 << bb.group_id)
        # Chain pairs where the small chain drapes the large one.
        draped: set[frozenset] = set()
        for i, rb in enumerate(self.pmx_data.rigid_bodies):
            if rb.physics_mode == PhysicsMode.FOLLOW_BONE:
                continue
            ri = _find(i)
            for j, rbj in enumerate(self.pmx_data.rigid_bodies):
                if j <= i or rbj.physics_mode == PhysicsMode.FOLLOW_BONE:
                    continue
                rj = _find(j)
                if rbj.group_id == rb.group_id or rj == ri:
                    continue
                # Guard 3: one chain small, the other large.
                si, sj = chain_size.get(ri, 0), chain_size.get(rj, 0)
                if not ((si <= _CLOTH_SMALL_CHAIN and sj >= _CLOTH_LARGE_SHEET) or
                        (sj <= _CLOTH_SMALL_CHAIN and si >= _CLOTH_LARGE_SHEET)):
                    continue
                # Guard 2: same body part.
                if (anchor_groups.get(ri, 0) & anchor_groups.get(rj, 0)) == 0:
                    continue
                dx = centers[i].x - centers[j].x
                dy = centers[i].y - centers[j].y
                dz = centers[i].z - centers[j].z
                # Guard 4: real draping interpenetration, not a mere touch.
                rr = extents[i] + extents[j] - _CLOTH_OVERLAP_PENETRATION
                if dx * dx + dy * dy + dz * dz < rr * rr:
                    draped.add(frozenset((ri, rj)))
        cloth_overlap: dict[int, int] = {}  # dynamic rb_idx -> dynamic group bits
        for i, rb in enumerate(self.pmx_data.rigid_bodies):
            if rb.physics_mode == PhysicsMode.FOLLOW_BONE:
                continue
            ri = _find(i)
            bits = 0
            for j, rbj in enumerate(self.pmx_data.rigid_bodies):
                if i == j or rbj.physics_mode == PhysicsMode.FOLLOW_BONE:
                    continue
                if rbj.group_id == rb.group_id:
                    continue  # own group stays cleared (hair↔hair pass-through)
                rj = _find(j)
                if rj == ri:
                    continue  # same cloth chain — jointed bodies never collide
                if (anchor_groups.get(ri, 0) & anchor_groups.get(rj, 0)) == 0:
                    continue  # different body part (arm/head vs torso)
                if frozenset((ri, rj)) not in draped:
                    continue  # chains only touch / wrong size mix, no drape
                dx = centers[i].x - centers[j].x
                dy = centers[i].y - centers[j].y
                dz = centers[i].z - centers[j].z
                rr = extents[i] + extents[j] + 0.2
                if dx * dx + dy * dy + dz * dz < rr * rr:
                    bits |= 1 << rbj.group_id
            if bits:
                cloth_overlap[i] = bits

        for rb_idx, body in enumerate(self.pmx_data.rigid_bodies):
            spec = self._create_guide(
                rb_idx,
                body,
                group,
                kin_overlap.get(rb_idx, 0),
                dyn_overlap.get(rb_idx, 0),
                cloth_overlap.get(rb_idx, 0),
            )
            if spec is None:
                continue
            body_specs[rb_idx] = spec
            if body.physics_mode == PhysicsMode.FOLLOW_BONE:
                kinematic_order.append(rb_idx)
                created_follow += 1
            else:  # PHYSICS / PHYSICS_BONE
                created_dynamic += 1

        # Populate the node's body compound array (indices = PMX rb index).
        reset_index = self._compute_reset_anchor_map(kinematic_order)
        self._set_body_attributes(body_specs, reset_index)

        # Connect the kinematic anchor matrices (in PMX order).
        self._connect_kinematic_anchors(body_specs, kinematic_order)

        # Connect the dynamic body outputs (solved pose -> guide transform).
        self._connect_dynamic_outputs(body_specs)

        # Populate the node's joint compound array.
        self._set_joint_attributes()

        # Belt-and-suspenders on top of the node's native cache opt-out:
        # never cache the DG results of the physics subgraph.
        self._exclude_from_dg_cache()

        log.info(
            "Physics binding: %d FOLLOW_BONE bodies, %d dynamic bodies, %d joints",
            created_follow,
            created_dynamic,
            len(self.physics_joints),
        )
        return self

    # ------------------------------------------------------------------
    # Group / solver
    # ------------------------------------------------------------------
    def _create_group(self) -> Optional[str]:
        group_name = self.name_registry.get_physics_group_name()
        parent_name = None
        if self.root_transform_obj is not None and not self.root_transform_obj.isNull():
            try:
                parent_name = om.MFnDependencyNode(self.root_transform_obj).name()
            except Exception:
                parent_name = None
        if parent_name:
            group = cmds.createNode("transform", name=group_name, parent=parent_name)
        else:
            group = cmds.createNode("transform", name=group_name)
        return group

    def _create_solver(self) -> str:
        """Create the ``mmdPhysicsNode`` and make it time-driven through the DG.

        The node owns the Bullet world; connecting ``time1.outTime`` to its
        ``time`` input makes the evaluation manager step it on every frame.
        """
        node = cmds.createNode(
            _NODE_TYPE, name=self.name_registry.get_physics_solver_name()
        )
        try:
            cmds.connectAttr("time1.outTime", f"{node}.time")
        except Exception as e:
            log.warning("Could not connect time1 to node time: %s", e)
        cmds.setAttr(f"{node}.gravity", 0.0, _DEFAULT_GRAVITY_Y, 0.0)
        cmds.setAttr(f"{node}.fps", _DEFAULT_FPS)
        return node

    def _exclude_from_dg_cache(self) -> None:
        """Disable the DG value cache for the physics subgraph.

        The node already opts out of Cached Playback natively
        (``getCacheSetup``).  This additionally sets ``caching=0`` on every
        node so the classic DG cache never reuses stale solver outputs either.
        """
        nodes: list[str] = [self.node] if self.node else []
        nodes.extend(self.bodies.values())
        nodes.extend(self.constraints.values())
        for n in nodes:
            if not n or not cmds.objExists(n):
                continue
            try:
                cmds.setAttr(f"{n}.caching", 0)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Bodies
    # ------------------------------------------------------------------
    def _create_guide(
        self,
        rb_idx: int,
        body,
        group,
        kin_overlap_bits: int,
        dyn_overlap_bits: int,
        cloth_overlap_bits: int = 0,
    ) -> Optional[dict]:
        """Create one visible guide mesh for a PMX rigid body.

        Returns a body spec dict (used to populate the node's ``bodies``
        compound array) or None on failure.  The guide is placed at the PMX
        rest pose (Z-flip + handedness-correct rotation) and parented under the
        physics group; the node solves in the group's local space, so the
        guide's LOCAL transform is the Bullet rest pose.
        """
        jn = body.related_bone_index
        jpath = self._joint_for(jn) if jn >= 0 else None
        mode = body.physics_mode
        kinematic = mode == PhysicsMode.FOLLOW_BONE

        try:
            world_t = (body.shape_position.x, body.shape_position.y, -body.shape_position.z)
            world_r = mmd_euler_to_maya_degrees(
                body.shape_rotation.x, body.shape_rotation.y, body.shape_rotation.z
            )

            guide_name = self.name_registry.get_physics_rigidbody_name(rb_idx)
            guide = self._create_guide_mesh(body, guide_name)
            cmds.xform(guide, ws=True, translation=world_t)
            cmds.xform(guide, ws=True, rotation=world_r)
            cmds.parent(guide, group, absolute=True)

            # Rest local transform (relative to the physics group) — the
            # Bullet world's initial pose for this body.
            local_t = cmds.getAttr(f"{guide}.translate")[0]
            local_r = cmds.getAttr(f"{guide}.rotate")[0]

            # DG binding to the bone.
            if jpath is not None:
                if kinematic:
                    # Bone drives the guide (collider follows the bone).
                    con = cmds.parentConstraint(jpath, guide, maintainOffset=True)[0]
                elif mode == PhysicsMode.PHYSICS_BONE:
                    # Guide (solver) drives bone rotation only.
                    con = cmds.orientConstraint(guide, jpath, maintainOffset=True)[0]
                else:  # PHYSICS
                    # Guide (solver) drives the bone's full transform.
                    con = cmds.parentConstraint(guide, jpath, maintainOffset=True)[0]
                self.constraints[rb_idx] = con
            else:
                log.warning(
                    "Body %d (mode %s) has no related joint (bone %d) — left "
                    "at rest under the physics group",
                    rb_idx,
                    mode.name,
                    jn,
                )

            # Self-describing PMX metadata (docs/CustomAttributes.md pattern).
            for attr, value in (
                ("pmxRigidBodyIndex", rb_idx),
                ("pmxGroupId", body.group_id),
                ("pmxPhysicsMode", body.physics_mode.value),
            ):
                if not cmds.attributeQuery(attr, node=guide, exists=True):
                    cmds.addAttr(guide, longName=attr, attributeType="long")
                cmds.setAttr(f"{guide}.{attr}", value)

            # Unique surface shader per collision group, shared by all guides
            # in the group (see _group_shading_group).  Assign the MESH SHAPES
            # (not the transform): assigning a constrained transform (e.g. a
            # FOLLOW_BONE guide) emits "cannot add parentConstraint to set"
            # warnings, and per-shape assignment is precise anyway.
            try:
                sg = self._group_shading_group(body.group_id)
                for ms in cmds.listRelatives(guide, shapes=True, type="mesh") or []:
                    cmds.sets(ms, edit=True, forceElement=sg)
            except Exception as e:
                log.debug("Could not shade guide %s: %s", guide, e)

            self.bodies[rb_idx] = guide

            size = body.shape_size
            # Collision mask.
            #
            # The PMX non_collision_group is respected as the base, then two
            # MMD-intent corrections are applied, PROXIMITY-BASED (see
            # create()): a dynamic body collides with the kinematic "body"
            # colliders that overlap it at rest (so the skirt blocks on the
            # legs/hips it wraps, but the bangs keep colliding only with the
            # head, not the huge torso capsule), and a kinematic body blocks
            # the dynamic bodies that overlap it.  Dynamic bodies also clear
            # their OWN group bit: MMD models store hair/skirt spheres DEEPLY
            # OVERLAPPING, and self-collision pushes the chains apart ("the
            # bang is longer than normal"); like a well-configured MMD model,
            # hair strands pass through each other but still collide with the
            # body.
            mask = (~body.non_collision_group) & 0xFFFF
            if kinematic:
                mask |= dyn_overlap_bits
            else:
                mask &= ~(1 << body.group_id)
                mask |= kin_overlap_bits
                # Cloth-on-cloth: bangs/hair resting on the skirt/jacket that
                # shares the same kinematic anchor (see create()).  Both sides
                # gain the other's group, so the bangs rest ON the jacket
                # colliders instead of falling through them / under the arm.
                mask |= cloth_overlap_bits
            return {
                "restT": local_t,
                "restR": local_r,
                "mass": body.mass,
                # MMD's move_attenuation / rotation_damping ARE the damping
                # coefficients (1.0 = fully damped -> the body settles; 0.0 =
                # no damping -> it swings forever).  A previous
                # `damping = 1 - attenuation` was INVERTED: high-attenuation
                # cloth (skirt ~0.96, bangs/cape/hair 1.0) got near-ZERO
                # damping and never settled — the bangs "jumped back and
                # forth" on the torso/jacket instead of resting on it.
                "linearDamping": _clamp01(body.move_attenuation),
                "angularDamping": _clamp01(body.rotation_damping),
                "friction": _clamp01(body.friction_force),
                "restitution": _clamp01(body.repulsion),
                "collider": _PMX_TO_COLLIDER_TYPE.get(body.shape, 2),
                "radius": size.x,
                "extents": (size.x, size.y, size.z),
                "length": size.y,
                "group": 1 << body.group_id,
                "mask": mask,
                "kinematic": kinematic,
                "guide": guide,
            }
        except Exception as e:
            log.warning("Failed to create body %d: %s", rb_idx, e)
            return None

    def _set_body_attributes(
        self, body_specs: dict[int, dict], reset_index: dict[int, int]
    ) -> None:
        """Write every PMX rigid body into the node's ``bodies`` array."""
        if not self.node:
            return
        for rb_idx, spec in body_specs.items():
            base = f"{self.node}.bodies[{rb_idx}]"
            try:
                cmds.setAttr(f"{base}.bodyRestTranslate", *spec["restT"])
                cmds.setAttr(f"{base}.bodyRestRotate", *spec["restR"])
                cmds.setAttr(f"{base}.bodyMass", float(spec["mass"]))
                cmds.setAttr(f"{base}.bodyLinearDamping", float(spec["linearDamping"]))
                cmds.setAttr(f"{base}.bodyAngularDamping", float(spec["angularDamping"]))
                cmds.setAttr(f"{base}.bodyFriction", float(spec["friction"]))
                cmds.setAttr(f"{base}.bodyRestitution", float(spec["restitution"]))
                cmds.setAttr(f"{base}.bodyColliderType", int(spec["collider"]))
                cmds.setAttr(f"{base}.bodyRadius", float(spec["radius"]))
                cmds.setAttr(f"{base}.bodyExtents", *spec["extents"])
                cmds.setAttr(f"{base}.bodyLength", float(spec["length"]))
                cmds.setAttr(f"{base}.bodyGroup", int(spec["group"]))
                cmds.setAttr(f"{base}.bodyMask", int(spec["mask"]))
                cmds.setAttr(f"{base}.bodyKinematic", bool(spec["kinematic"]))
                cmds.setAttr(
                    f"{base}.bodyResetAnchorIndex", int(reset_index.get(rb_idx, -1))
                )
            except Exception as e:
                log.warning("Could not set body %d attributes: %s", rb_idx, e)

    def _connect_kinematic_anchors(
        self, body_specs: dict[int, dict], kinematic_order: list[int]
    ) -> None:
        """Connect each FOLLOW_BONE guide's world/parent-inverse matrices.

        ``anchorWorldMatrix[k]`` / ``anchorParentInverseMatrix[k]`` map 1:1 to
        the kinematic bodies in PMX body order (the C++ node expects exactly
        that).  local = world * parentInverse keeps the Bullet world in the
        physics group's local space.
        """
        if not self.node:
            return
        for k, rb_idx in enumerate(kinematic_order):
            guide = body_specs[rb_idx]["guide"]
            try:
                cmds.connectAttr(
                    f"{guide}.worldMatrix[0]",
                    f"{self.node}.anchorWorldMatrix[{k}]",
                    force=True,
                )
                cmds.connectAttr(
                    f"{guide}.parentInverseMatrix[0]",
                    f"{self.node}.anchorParentInverseMatrix[{k}]",
                    force=True,
                )
            except Exception as e:
                log.warning("Could not connect anchor %d (%s): %s", rb_idx, guide, e)

    def _compute_reset_anchor_map(self, kinematic_order: list[int]) -> dict[int, int]:
        """Map each dynamic body to the anchor that drives its scrub-back reset.

        When time is scrubbed backwards the C++ node teleports dynamic bodies
        to their rest pose transformed by the CURRENT skeleton pose.  The
        skeleton pose is captured from the kinematic ANCHORS (FOLLOW_BONE
        guides — non-circular: the bone drives the guide, no write-back).  For
        each dynamic body we use the anchor of its NEAREST KINEMATIC ANCESTOR
        bone (walking the PMX parent chain), so hair uses the head anchor,
        skirt uses the pelvis anchor, sleeves use the shoulder/arm anchor, etc.

        Returns ``{rb_index: anchor_index}`` (anchor_index = position in
        ``kinematic_order``); dynamic bodies without a kinematic ancestor are
        omitted (no reset).
        """
        # bone index -> anchor index of the FOLLOW_BONE body bound to it.
        bone_to_anchor: dict[int, int] = {}
        for a, rb_idx in enumerate(kinematic_order):
            rb = self.pmx_data.rigid_bodies[rb_idx]
            if rb.related_bone_index >= 0:
                bone_to_anchor.setdefault(rb.related_bone_index, a)

        def _find_anchor(bone_idx: int) -> int:
            seen: set[int] = set()
            while bone_idx >= 0 and bone_idx not in seen:
                seen.add(bone_idx)
                if bone_idx in bone_to_anchor:
                    return bone_to_anchor[bone_idx]
                if bone_idx >= len(self.pmx_data.bones):
                    return -1
                bone_idx = self.pmx_data.bones[bone_idx].parentIndex
            return -1

        result: dict[int, int] = {}
        for rb_idx, rb in enumerate(self.pmx_data.rigid_bodies):
            if rb.physics_mode == PhysicsMode.FOLLOW_BONE or rb.related_bone_index < 0:
                continue
            anchor = _find_anchor(rb.related_bone_index)
            if anchor >= 0:
                result[rb_idx] = anchor
        return result

    def _connect_dynamic_outputs(self, body_specs: dict[int, dict]) -> None:
        """Drive each dynamic guide's transform from the node's solved pose."""
        if not self.node:
            return
        for rb_idx, spec in body_specs.items():
            if spec["kinematic"]:
                continue
            guide = spec["guide"]
            try:
                cmds.connectAttr(
                    f"{self.node}.outTranslate[{rb_idx}].outTranslateValue",
                    f"{guide}.translate",
                    force=True,
                )
                cmds.connectAttr(
                    f"{self.node}.outRotate[{rb_idx}].outRotateValue",
                    f"{guide}.rotate",
                    force=True,
                )
            except Exception as e:
                log.warning(
                    "Could not connect dynamic output %d (%s): %s", rb_idx, guide, e
                )

    # ------------------------------------------------------------------
    # Joints
    # ------------------------------------------------------------------
    def _set_joint_attributes(self) -> None:
        """Write every PMX joint into the node's ``joints`` array."""
        if not self.node:
            return
        for jt_idx, joint in enumerate(self.pmx_data.joints):
            base = f"{self.node}.joints[{jt_idx}]"
            try:
                # Joint frame in the physics group's local space.
                jp = joint.position
                jr = joint.rotation
                frame_t = (jp.x, jp.y, -jp.z)
                frame_r = mmd_euler_to_maya_degrees(jr.x, jr.y, jr.z)

                pmin = joint.position_min
                pmax = joint.position_max
                rmin = joint.rotation_min
                rmax = joint.rotation_max
                psc = joint.position_spring_constant
                rsc = joint.rotation_spring_constant

                cmds.setAttr(f"{base}.jointBodyA", int(joint.rigid_body_index_a))
                cmds.setAttr(f"{base}.jointBodyB", int(joint.rigid_body_index_b))
                cmds.setAttr(f"{base}.jointType", int(joint.type.value))
                cmds.setAttr(f"{base}.jointFrameTranslate", *frame_t)
                cmds.setAttr(f"{base}.jointFrameRotate", *frame_r)

                # Linear limits: PMX units.  Angular limits: PMX radians — the
                # node passes them straight to Bullet (radians).
                if pmin is not None and pmax is not None:
                    cmds.setAttr(f"{base}.jointLinearMin", pmin.x, pmin.y, pmin.z)
                    cmds.setAttr(f"{base}.jointLinearMax", pmax.x, pmax.y, pmax.z)
                if rmin is not None and rmax is not None:
                    cmds.setAttr(f"{base}.jointAngularMin", rmin.x, rmin.y, rmin.z)
                    cmds.setAttr(f"{base}.jointAngularMax", rmax.x, rmax.y, rmax.z)
                if psc is not None:
                    cmds.setAttr(f"{base}.jointLinearSpring", psc.x, psc.y, psc.z)
                if rsc is not None:
                    cmds.setAttr(f"{base}.jointAngularSpring", rsc.x, rsc.y, rsc.z)
                self.physics_joints.append(jt_idx)
            except Exception as e:
                log.warning("Could not set joint %d attributes: %s", jt_idx, e)

    # ------------------------------------------------------------------
    # Misc helpers
    # ------------------------------------------------------------------
    def _joint_for(self, bone_idx: int) -> Optional[str]:
        return self._joint_names.get(bone_idx)

    @staticmethod
    def _create_guide_mesh(body, guide_name: str) -> str:
        """Create a polygonal guide mesh (sphere / box / capsule) for a body.

        The guide is the VISIBLE rigid body — a regular mesh always draws from
        its DAG matrix, so it follows its bone (via the parentConstraint)
        reliably, including under Cached Playback.
        """
        size = body.shape_size
        if body.shape == ShapeType.SPHERE:
            # Size.x is the radius.
            return cmds.polySphere(
                radius=size.x, subdivisionsX=12, subdivisionsY=12, name=guide_name
            )[0]
        if body.shape == ShapeType.BOX:
            # Size components are half-extents.
            return cmds.polyCube(
                width=size.x * 2, height=size.y * 2, depth=size.z * 2, name=guide_name
            )[0]
        # CAPSULE: size.x is radius, size.y is total height incl. caps.  Use MEL
        # directly — cmds.polyCylinder has a bug with roundCap.
        mel_cmd = (
            f"polyCylinder -r {size.x} -h {size.y} "
            f"-sx 12 -sh 1 -sc 12 -rcp true "
            f'-n "{guide_name}";'
        )
        result = mel.eval(mel_cmd)
        return result[0] if isinstance(result, list) else result

    def _group_shading_group(self, group_id: int) -> str:
        """Return (creating on first use) the shading group for a collision group.

        Each collision group gets ONE unique surface shader (Maya 2024+:
        ``openPBRSurface``; older releases fall back to a Lambert), colored from
        the group palette and shared by every guide mesh in that group.  This
        shades the guide meshes reliably in the viewport — unlike the old
        draw-override tint (``overrideEnabled`` + ``overrideColorRGB``) which
        was a leftover from the mayaBullet collider-shape era and does not
        color mesh guides.
        """
        existing = self._group_materials.get(group_id)
        if existing:
            return existing[1]
        group_name = self._group or self.name_registry.get_physics_group_name()
        shader, sg = _create_group_material(group_name, group_id)
        self._group_materials[group_id] = (shader, sg)
        return sg

    # ------------------------------------------------------------------
    # Headless stepping (interactive playback is resolved by the DG — the
    # node steps on every time1 change; no scriptJob)
    # ------------------------------------------------------------------
    def step(self) -> None:
        """Force a fresh node evaluation at the current time.

        Only needed for headless/batch use (or to manually advance the sim).
        In interactive Maya the binding is pure DG — the node's output
        connections pull it on every time step, so playback advances the
        simulation.
        """
        if not self.node:
            return
        try:
            cmds.dgdirty(self.node)
            cmds.dgeval(self.node)
        except Exception as e:
            log.debug("physics step dgeval failed: %s", e)

    def write_back(self) -> None:
        """Write solved dynamic-body transforms back to their related bones.

        The DG constraints (``parentConstraint`` / ``orientConstraint``) already
        perform the per-frame write-back interactively.  This method exists for
        headless/batch stepping: after :meth:`step`, it pushes the solved pose
        through the guide transforms so the DG constraints propagate it to the
        bones.
        """
        try:
            for guide in self.bodies.values():
                cmds.dgdirty(guide)
                cmds.dgeval(guide)
            for con in self.constraints.values():
                cmds.dgdirty(con)
                cmds.dgeval(con)
        except Exception as e:
            log.debug("physics write_back failed: %s", e)

    def teardown(self) -> None:
        """Remove the physics nodes for this model."""
        if self._group is not None and cmds.objExists(self._group):
            try:
                cmds.delete(self._group)
            except Exception as e:
                log.debug("teardown delete failed: %s", e)
        if self.node is not None and cmds.objExists(self.node):
            try:
                cmds.delete(self.node)
            except Exception as e:
                log.debug("teardown node delete failed: %s", e)
        # Group materials are standalone DG nodes — remove them so re-imports
        # don't accumulate orphaned Lambert shaders.
        for shader, sg in self._group_materials.values():
            try:
                cmds.delete(sg)
            except Exception:
                pass
            try:
                cmds.delete(shader)
            except Exception:
                pass
        self._group_materials.clear()
        self.bodies.clear()
        self.constraints.clear()


def create_physics_from_pmx_data(
    pmx_data: PmxModel,
    joints,
    name_registry,
    root_transform_obj=None,
) -> PhysicsBinding:
    """Create an ``mmdPhysicsNode`` :class:`PhysicsBinding` for a PMX model.

    Args:
        pmx_data:            Parsed PMX model.
        joints:              Joint MObjects in PMX bone order (from bone builder).
        name_registry:       Naming manager for unique names.
        root_transform_obj:  MObject the physics group is parented under.

    Returns:
        The created :class:`PhysicsBinding` (may be empty if the model has no
        rigid bodies).
    """
    binding = PhysicsBinding(pmx_data, joints, name_registry, root_transform_obj)
    return binding.create()
