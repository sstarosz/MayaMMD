"""Unit tests for :mod:`mmd.maya.rigid_body_info`.

The scene is faked (see :class:`_FakeScene`) so ordering, attribute reads,
connection resolution and the derived freedom / status models can be exercised
without a Maya runtime.  The fake reproduces what real Maya does, including the
two behaviours that made the first version read nothing:

* shapes are the nodes wired into the solver's message arrays, and their
  parent transform is the guide;
* ``listConnections`` without ``plugs=True`` reports the guide, not the shape.

It also reproduces ``getAttr -asString`` and the ``-listEnum`` encoding
(``'Box=1:Sphere:Capsule'``).
"""

from __future__ import annotations

import math
import unittest
from typing import ClassVar
from unittest.mock import MagicMock, patch

from mmd.maya import rigid_body_info as rbi

MODEL = "Miku"


class _FakeScene:
    """Minimal DAG + DG fake for the ``cmds`` calls the module makes.

    Mirrors the hierarchy produced by PMX import::

        |{model}_Root
        |{model}_Root|{model}_RigidBodies
            |{model}_RigidBodySolver        pmxRigidBodyNode (shape)
            |{model}_Bodies
                |{model}_{body}             guide transform
                    |pmxRigidBodyShape{n}   pmxRigidBodyShape (data)
            |{model}_Joints
                |{model}_{joint}_Constraint frame transform
                    |pmxRigidBodyJoint{n}   pmxRigidBodyJoint (data)
    """

    ENUMS: ClassVar[dict[str, dict[str, str]]] = {
        "pmxRigidBodyShape": {
            # Sparse: the first value carries the explicit index, exactly as
            # Maya reports it.
            "bodyColliderType": "Box=1:Sphere:Capsule",
            "bodyPhysicsMode": "FollowBone:Physics:PhysicsBone",
            "drawMode": "Off:Wire:Solid:WireSolid",
        },
        "pmxRigidBodyJoint": {"drawMode": "Off:Hub:Ranges"},
    }

    def __init__(self, *, solver: bool = True) -> None:
        self.root = f"|{MODEL}_Root"
        physics = f"{self.root}|{MODEL}_RigidBodies"
        self.bodies_group = f"{physics}|{MODEL}_Bodies"
        self.joints_group = f"{physics}|{MODEL}_Joints"
        self.solver = f"{physics}|{MODEL}_RigidBodySolver"

        self.children: dict[str, list[str]] = {
            self.root: [physics],
            physics: [self.bodies_group, self.joints_group],
            self.bodies_group: [],
            self.joints_group: [],
        }
        self.types: dict[str, str] = {
            self.root: "transform",
            physics: "transform",
            self.bodies_group: "transform",
            self.joints_group: "transform",
        }
        self.attrs: dict[str, dict[str, object]] = {}
        self.sources: dict[str, str] = {}
        self.multi: dict[str, dict[int, str]] = {}
        self.body_shapes: dict[str, str] = {}
        if solver:
            self.children[physics].append(self.solver)
            self.types[self.solver] = rbi.TYPE_SOLVER
            self.multi[f"{self.solver}.bodyShapes"] = {}
            self.multi[f"{self.solver}.jointShapes"] = {}

    # ── Scene building ───────────────────────────────────────────────────

    def _link(self, parent: str, child: str) -> None:
        self.children.setdefault(parent, []).append(child)
        self.children.setdefault(child, [])

    def add_body(
        self,
        name: str,
        *,
        slot: int | None = None,
        mode: int = 1,
        enabled: bool = True,
        bone: str | None = None,
        universal: str = "",
        collider: int = 2,
    ) -> str:
        """Add a rigid body; ``slot`` is its solver index (default: last)."""
        guide = f"{self.bodies_group}|{MODEL}_{name}"
        shape = f"{guide}|pmxRigidBodyShape{len(self.body_shapes) + 1}"
        self._link(self.bodies_group, guide)
        self._link(guide, shape)
        self.types[guide] = "transform"
        self.types[shape] = rbi.TYPE_BODY_SHAPE
        self.attrs[shape] = {
            "bodyNameLocal": name,
            "bodyNameUniversal": universal,
            "bodyGroupId": 4,
            "bodyColliderType": collider,
            "bodyShapeSize": [(0.5, 1.0, 1.5)],
            "bodyRestTranslate": [(1.0, 2.0, 3.0)],
            "bodyRestRotate": [(0.0, 0.0, 90.0)],
            "bodyPhysicsMode": mode,
            "bodyEnabled": enabled,
            "bodyMass": 2.5,
            "bodyLinearDamping": 0.1,
            "bodyAngularDamping": 0.2,
            "bodyRestitution": 0.3,
            "bodyFriction": 0.4,
            "drawMode": 1,
        }
        for group in range(rbi.COLLISION_GROUP_COUNT):
            self.attrs[shape][f"bodyMaskGroup{group}"] = group != 3
        if bone:
            self.sources[f"{shape}.bodyJoint"] = bone
        self.body_shapes[name] = shape
        wired = self.multi.get(f"{self.solver}.bodyShapes")
        if wired is not None:
            wired[len(wired) if slot is None else slot] = shape
        return shape

    def add_joint(
        self,
        name: str,
        body_a: str | None,
        body_b: str | None,
        *,
        linear: tuple[tuple[float, float, float], tuple[float, float, float]] = (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
        angular: tuple[tuple[float, float, float], tuple[float, float, float]] = (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
        linear_stiffness: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular_stiffness: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> str:
        frame = f"{self.joints_group}|{MODEL}_{name}_Constraint"
        node = f"{frame}|pmxRigidBodyJoint{len(self.attrs) + 1}"
        self._link(self.joints_group, frame)
        self._link(frame, node)
        self.types[frame] = "transform"
        self.types[node] = rbi.TYPE_JOINT
        self.attrs[node] = {
            "jointNameLocal": name,
            "jointNameUniversal": "",
            "jointLinearMin": [linear[0]],
            "jointLinearMax": [linear[1]],
            "jointAngularMin": [angular[0]],
            "jointAngularMax": [angular[1]],
            "jointLinearStiffness": [linear_stiffness],
            "jointAngularStiffness": [angular_stiffness],
            "drawMode": 1,
        }
        if body_a:
            self.sources[f"{node}.jointBodyA"] = self.body_shapes[body_a]
        if body_b:
            self.sources[f"{node}.jointBodyB"] = self.body_shapes[body_b]
        wired = self.multi.get(f"{self.solver}.jointShapes")
        if wired is not None:
            wired[len(wired)] = node
        return node

    def add_bone(self, name: str) -> str:
        """Add a bone; its short name is what Maya reports for the connection."""
        bone = f"{self.root}|{MODEL}_Bones|{name}"
        self.types[bone] = "joint"
        self.children.setdefault(bone, [])
        return name

    # ── Maya command emulation ───────────────────────────────────────────

    def obj_exists(self, node: str) -> bool:
        return node in self.children or node in self.types

    def list_relatives(self, node, **kwargs):
        if kwargs.get("allDescendents"):
            wanted = kwargs.get("type")
            found: list[str] = []
            stack = list(self.children.get(node, []))
            while stack:
                child = stack.pop(0)
                if wanted is None or self.types.get(child) == wanted:
                    found.append(child)
                stack.extend(self.children.get(child, []))
            return found
        return list(self.children.get(node, []))

    def list_connections(self, objects, source=False, destination=False, **_kwargs):
        """Mirror Maya, including *not* honouring ``fullPath``."""
        if "fullPath" in _kwargs:
            raise TypeError("Invalid flag 'fullPath'")
        asked = objects if isinstance(objects, list) else [objects]
        found: list[str] = []
        for plug in asked:
            node, _, attr = plug.rpartition(".")
            if attr and "[" in attr:
                attr = attr.split("[", 1)[0]
            if "[" in plug:
                index = int(plug[plug.rfind("[") + 1 : plug.rfind("]")])
                source_node = self.multi.get(f"{node}.{attr}", {}).get(index)
                if source_node and source:
                    found.append(f"{source_node}.solver")
                continue
            if not attr:
                continue
            source_node = self.sources.get(f"{node}.{attr}")
            if source_node and source:
                found.append(f"{source_node}.message")
        return found or None

    def node_type(self, nodes):
        if isinstance(nodes, list):
            return [self.types.get(node, "transform") for node in nodes]
        return self.types.get(nodes, "transform")

    def ls(self, names, long=False, **_kwargs):
        if not isinstance(names, list):
            return [names]
        return list(names)

    def get_attr(self, objects, asString=False, multiIndices=False, **_kwargs):
        if multiIndices:
            wired = self.multi.get(objects)
            if wired is None:
                raise RuntimeError(f"no array {objects}")
            return sorted(wired) or None
        if isinstance(objects, list):
            if len(objects) == 1:
                return self.get_attr(objects[0], asString=asString)
            return [self.get_attr(plug, asString=asString) for plug in objects]
        node, _, attr = objects.rpartition(".")
        table = self.attrs.get(node, {})
        if attr not in table:
            raise RuntimeError(f"no attribute {objects}")
        value = table[attr]
        if not asString:
            return value
        for name, index in self.enum_pairs(node, attr).items():
            if index == value:
                return name
        raise RuntimeError(f"{objects} is not an enum")

    def enum_pairs(self, node: str, attr: str) -> dict[str, int]:
        raw = self.ENUMS.get(self.types.get(node, ""), {}).get(attr)
        if raw is None:
            return {}
        pairs: dict[str, int] = {}
        value = 0
        for entry in raw.split(":"):
            name, sep, explicit = entry.partition("=")
            if sep:
                value = int(explicit)
            pairs[name] = value
            value += 1
        return pairs

    def attribute_query(self, attr, **kwargs):
        raw = self.ENUMS.get(self.types.get(kwargs.get("node"), ""), {}).get(attr)
        return [raw] if raw else None

    def install(self, cmds: MagicMock) -> None:
        cmds.objExists.side_effect = self.obj_exists
        cmds.listRelatives.side_effect = self.list_relatives
        cmds.listConnections.side_effect = self.list_connections
        cmds.nodeType.side_effect = self.node_type
        cmds.ls.side_effect = self.ls
        cmds.getAttr.side_effect = self.get_attr
        cmds.attributeQuery.side_effect = self.attribute_query
        cmds.currentUnit.return_value = "deg"


class _PhysicsTestBase(unittest.TestCase):
    def setUp(self) -> None:
        rbi.clear_enum_cache()
        self.addCleanup(rbi.clear_enum_cache)
        patcher = patch("mmd.maya.rigid_body_info.cmds")
        self.cmds = patcher.start()
        self.addCleanup(patcher.stop)

    def scene(self, **kwargs) -> _FakeScene:
        scene = _FakeScene(**kwargs)
        scene.install(self.cmds)
        return scene


class TestRangeKind(unittest.TestCase):
    def test_equal_bounds_are_locked(self) -> None:
        self.assertIs(rbi.range_kind(0.0, 0.0), rbi.RangeKind.FIXED)

    def test_inverted_bounds_are_free(self) -> None:
        self.assertIs(rbi.range_kind(1.0, -1.0), rbi.RangeKind.FREE)

    def test_any_open_range_is_limited(self) -> None:
        self.assertIs(rbi.range_kind(0.0, 0.001), rbi.RangeKind.LIMITED)

    def test_comparison_is_at_float_precision(self) -> None:
        """A gap below a 32-bit float reads as locked, exactly like Bullet."""
        self.assertIs(rbi.range_kind(1.0, 1.0 + 1e-9), rbi.RangeKind.FIXED)
        self.assertIs(rbi.range_kind(1.0, 1.0 + 1e-6), rbi.RangeKind.LIMITED)

    def test_float32_helper_survives_garbage(self) -> None:
        self.assertEqual(rbi._float32(float("inf")), float("inf"))


class TestEmptyScenes(_PhysicsTestBase):
    def test_no_root(self) -> None:
        self.assertTrue(rbi.read_physics("").is_empty)

    def test_unknown_root(self) -> None:
        self.scene()
        info = rbi.read_physics("|nope_Root")
        self.assertTrue(info.is_empty)
        self.assertEqual(info.warnings, ())

    def test_model_without_physics(self) -> None:
        scene = self.scene(solver=False)
        info = rbi.read_physics(scene.root)
        self.assertTrue(info.is_empty)
        self.assertEqual(info.warnings, ())


class TestRigidBodies(_PhysicsTestBase):
    def test_partial_paths_resolve_without_aliasing_duplicate_shape_names(self) -> None:
        paths = ["|Root|guide0|sharedShape", "|Root|guide1|sharedShape"]
        self.cmds.ls.return_value = paths
        self.assertEqual(
            rbi._long_paths(["guide0|sharedShape", "guide1|sharedShape", "sharedShape", ""]),
            [*paths, "", ""],
        )

    def test_single_body_and_joint_keep_complete_strings(self) -> None:
        scene = self.scene()
        scene.add_body("Waist", universal="Waist universal")
        scene.add_joint("Waist joint", "Waist", "Waist")
        info = rbi.read_physics(scene.root)
        self.assertEqual(info.bodies[0].name_local, "Waist")
        self.assertEqual(info.bodies[0].name_universal, "Waist universal")
        self.assertEqual(info.bodies[0].collider_label, "Sphere")
        self.assertEqual(info.bodies[0].draw_mode_label, "Wire")
        self.assertEqual(info.joints[0].name_local, "Waist joint")
        self.assertEqual(info.joints[0].draw_mode_label, "Hub")

    def test_deleted_slots_do_not_renumber_bodies_or_joints(self) -> None:
        scene = self.scene()
        scene.add_body("hip", slot=0)
        scene.add_body("knee", slot=1)
        scene.add_body("ankle", slot=2)
        scene.add_joint("removed", "hip", "knee")
        scene.add_joint("remaining", "hip", "ankle")
        # Maya can retain unconnected array elements after node deletion.
        scene.multi[f"{scene.solver}.bodyShapes"][1] = ""
        scene.multi[f"{scene.solver}.jointShapes"][0] = ""
        info = rbi.read_physics(scene.root)
        self.assertEqual([body.index for body in info.bodies], [0, 2])
        self.assertEqual([joint.index for joint in info.joints], [1])
        self.assertEqual(info.joints[0].body_a_index, 0)
        self.assertEqual(info.joints[0].body_b_index, 2)
        # Removing the unused element altogether must give the same result.
        del scene.multi[f"{scene.solver}.bodyShapes"][1]
        self.assertEqual(
            [body.index for body in rbi.read_physics(scene.root).bodies], [0, 2]
        )

    def test_reads_every_field(self) -> None:
        scene = self.scene()
        bone = scene.add_bone("hip_Jnt")
        shape = scene.add_body("hip", bone=bone, universal="Hip")
        body = rbi.read_physics(scene.root).bodies[0]

        self.assertEqual(body.index, 0)
        self.assertEqual(body.name, "hip")
        self.assertEqual(body.name_local, "hip")
        self.assertEqual(body.name_universal, "Hip")
        self.assertEqual(body.bone, "hip_Jnt")
        self.assertEqual(body.group, 4)
        self.assertEqual(body.collider_label, "Sphere")
        self.assertEqual(body.physics_mode_label, "Physics")
        self.assertEqual(body.shape_size, (0.5, 1.0, 1.5))
        self.assertEqual(body.rest_translate, (1.0, 2.0, 3.0))
        self.assertEqual(body.rest_rotate, (0.0, 0.0, 90.0))
        self.assertEqual(body.mass, 2.5)
        self.assertEqual(body.linear_damping, 0.1)
        self.assertAlmostEqual(body.friction, 0.4)
        self.assertTrue(body.enabled)
        self.assertEqual(body.draw_mode_label, "Wire")
        # The data node is the shape; the guide is its parent transform.
        self.assertEqual(body.node, shape)
        self.assertEqual(body.guide, f"{scene.bodies_group}|{MODEL}_hip")

    def test_order_follows_solver_index_not_dag_order(self) -> None:
        """The list must read like PMX Editor, i.e. ``bodyShapes[]`` order."""
        scene = self.scene()
        scene.add_body("second", slot=0)
        scene.add_body("first", slot=1)

        bodies = rbi.read_physics(scene.root).bodies
        self.assertEqual([body.index for body in bodies], [0, 1])
        self.assertEqual([body.name for body in bodies], ["second", "first"])

    def test_kinematic_flag(self) -> None:
        scene = self.scene()
        scene.add_body("kinematic", mode=0)
        scene.add_body("dynamic", mode=1)
        bodies = rbi.read_physics(scene.root).bodies
        self.assertTrue(bodies[0].is_kinematic)
        self.assertFalse(bodies[1].is_kinematic)

    def test_collision_mask(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        body = rbi.read_physics(scene.root).bodies[0]
        self.assertEqual(body.collides_with, tuple(g for g in range(16) if g != 3))
        self.assertEqual(body.ignored_groups, (3,))

    def test_name_falls_back_to_universal_then_index(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        scene.attrs[scene.body_shapes["hip"]]["bodyNameLocal"] = ""
        scene.add_body("knee")
        scene.attrs[scene.body_shapes["knee"]]["bodyNameLocal"] = ""
        scene.attrs[scene.body_shapes["knee"]]["bodyNameUniversal"] = "Knee"
        scene.add_body("ankle")
        scene.attrs[scene.body_shapes["ankle"]]["bodyNameLocal"] = ""
        bodies = rbi.read_physics(scene.root).bodies
        self.assertEqual(bodies[0].name, "RigidBody_0")
        self.assertEqual(bodies[1].name, "Knee")
        self.assertEqual(bodies[2].name, "RigidBody_2")

    def test_missing_optional_attributes_use_defaults(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        attrs = scene.attrs[scene.body_shapes["hip"]]
        for attr in ("bodyMass", "bodyRestTranslate", "bodyEnabled", "drawMode"):
            del attrs[attr]
        body = rbi.read_physics(scene.root).bodies[0]
        self.assertEqual(body.mass, 0.0)
        self.assertEqual(body.rest_translate, (0.0, 0.0, 0.0))
        self.assertTrue(body.enabled)
        self.assertEqual(body.draw_mode_label, "")


class TestJoints(_PhysicsTestBase):
    def test_resolves_both_bodies(self) -> None:
        scene = self.scene()
        scene.add_body("hip", mode=0)
        scene.add_body("knee", mode=1)
        node = scene.add_joint("knee_joint", "hip", "knee")
        joint = rbi.read_physics(scene.root).joints[0]

        self.assertEqual(joint.index, 0)
        self.assertEqual(joint.name, "knee_joint")
        self.assertEqual((joint.body_a_index, joint.body_a), (0, "hip"))
        self.assertEqual((joint.body_b_index, joint.body_b), (1, "knee"))
        self.assertEqual(joint.node, node)
        self.assertEqual(
            joint.guide, f"{scene.joints_group}|{MODEL}_knee_joint_Constraint"
        )
        self.assertIs(joint.status, rbi.JointStatus.ACTIVE)

    def test_status_missing_body(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        scene.add_joint("joint", "hip", None)
        joint = rbi.read_physics(scene.root).joints[0]
        self.assertIs(joint.status, rbi.JointStatus.MISSING_BODY)
        self.assertEqual(joint.body_b_index, -1)
        self.assertEqual(joint.body_b, "")
        self.assertEqual(joint.body_b_node, "")

    def test_status_same_body(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        scene.add_joint("joint", "hip", "hip")
        joint = rbi.read_physics(scene.root).joints[0]
        self.assertIs(joint.status, rbi.JointStatus.SAME_BODY)

    def test_status_disabled_body(self) -> None:
        scene = self.scene()
        scene.add_body("hip", enabled=False)
        scene.add_body("knee")
        scene.add_joint("joint", "hip", "knee")
        joint = rbi.read_physics(scene.root).joints[0]
        self.assertIs(joint.status, rbi.JointStatus.DISABLED_BODY)

    def test_status_both_kinematic(self) -> None:
        scene = self.scene()
        scene.add_body("hip", mode=0)
        scene.add_body("knee", mode=0)
        scene.add_joint("joint", "hip", "knee")
        joint = rbi.read_physics(scene.root).joints[0]
        self.assertIs(joint.status, rbi.JointStatus.BOTH_KINEMATIC)
        self.assertFalse(joint.status.is_active)


class TestJointFreedom(_PhysicsTestBase):
    def _joint(self, **kwargs) -> rbi.JointInfo:
        scene = self.scene()
        scene.add_body("hip")
        scene.add_body("knee")
        scene.add_joint("joint", "hip", "knee", **kwargs)
        return rbi.read_physics(scene.root).joints[0]

    def test_zero_limits_are_welded(self) -> None:
        joint = self._joint()
        self.assertEqual(joint.freedom.summary, "welded")
        self.assertIs(joint.freedom.linear[0].kind, rbi.RangeKind.FIXED)

    def test_limited_rotation_only(self) -> None:
        joint = self._joint(angular=((-30.0, -10.0, 0.0), (30.0, 10.0, 0.0)))
        self.assertEqual(joint.freedom.summary, "rotates only")
        self.assertIs(joint.freedom.angular[0].kind, rbi.RangeKind.LIMITED)
        self.assertIs(joint.freedom.angular[2].kind, rbi.RangeKind.FIXED)

    def test_translation_only(self) -> None:
        joint = self._joint(linear=((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)))
        self.assertEqual(joint.freedom.summary, "translates only")

    def test_inverted_limits_are_free(self) -> None:
        joint = self._joint(linear=((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)))
        self.assertIs(joint.freedom.linear[0].kind, rbi.RangeKind.FREE)
        self.assertEqual(joint.freedom.summary, "translates only")

    def test_wide_angle_limits_wrap_to_free(self) -> None:
        """A range wider than +-180 degrees wraps, exactly as Bullet does."""
        joint = self._joint(angular=((-200.0, 0.0, 0.0), (200.0, 0.0, 0.0)))
        self.assertIs(joint.freedom.angular[0].kind, rbi.RangeKind.FREE)

    def test_spring_reported_only_on_a_movable_axis(self) -> None:
        joint = self._joint(
            angular=((0.0, -10.0, 0.0), (0.0, 10.0, 0.0)),
            angular_stiffness=(5.0, 7.0, 0.0),
        )
        self.assertFalse(joint.freedom.angular[0].spring)  # locked, ignored
        self.assertTrue(joint.freedom.angular[1].spring)

    def test_angle_unit_is_honoured(self) -> None:
        scene = self.scene()
        self.cmds.currentUnit.return_value = "rad"
        scene.add_body("hip")
        scene.add_body("knee")
        scene.add_joint(
            "joint", "hip", "knee", angular=((0.0, 0.0, 0.0), (math.pi, 0.0, 0.0))
        )
        radians = rbi.read_physics(scene.root).joints[0]
        self.assertEqual(radians.angle_unit, "rad")
        self.assertIs(radians.freedom.angular[0].kind, rbi.RangeKind.LIMITED)
        self.assertAlmostEqual(radians.freedom.angular[0].upper, math.pi, places=5)


class TestFallbacks(_PhysicsTestBase):
    def test_solver_removed_falls_back_to_the_dag(self) -> None:
        scene = self.scene(solver=False)
        scene.add_body("hip")
        scene.add_body("knee")
        info = rbi.read_physics(scene.root)
        self.assertEqual(info.solver, "")
        self.assertEqual([body.name for body in info.bodies], ["hip", "knee"])
        self.assertEqual(len(info.warnings), 1)
        self.assertIn("DAG", info.warnings[0])

    def test_unwired_nodes_still_show(self) -> None:
        """Nodes that exist but are not wired into the solver are listed."""
        scene = self.scene()
        scene.add_body("hip")
        scene.multi[f"{scene.solver}.bodyShapes"] = {}
        info = rbi.read_physics(scene.root)
        self.assertEqual([body.name for body in info.bodies], ["hip"])
        self.assertTrue(info.warnings)

    def test_bodies_without_joints(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        self.assertEqual(rbi.read_physics(scene.root).counts, (1, 0))

    def test_convenience_readers(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        scene.add_body("knee")
        scene.add_joint("joint", "hip", "knee")
        self.assertEqual(
            [b.name for b in rbi.read_rigid_bodies(scene.root)], ["hip", "knee"]
        )
        self.assertEqual([j.name for j in rbi.read_joints(scene.root)], ["joint"])

    def test_row_lookup_accepts_node_and_guide(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        info = rbi.read_physics(scene.root)
        body = info.bodies[0]
        self.assertIs(info.row_at(body.node), body)
        self.assertIs(info.row_at(body.guide), body)
        self.assertIsNone(info.row_at("|nope"))

    def test_body_and_joint_lookup_by_index(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        scene.add_body("knee")
        scene.add_joint("joint", "hip", "knee")
        info = rbi.read_physics(scene.root)
        knee = info.body(1)
        joint = info.joint(0)
        assert knee is not None
        assert joint is not None
        self.assertEqual(knee.name, "knee")
        self.assertEqual(joint.name, "joint")
        self.assertIsNone(info.body(9))
        self.assertIsNone(info.joint(9))


class TestEnumFields(_PhysicsTestBase):
    def test_sparse_enum_values_are_read_from_maya(self) -> None:
        """``Box=1:Sphere:Capsule`` means values 1..3, not 0..2."""
        scene = self.scene()
        scene.add_body("hip")
        shape = scene.body_shapes["hip"]
        self.assertEqual(
            rbi.enum_fields(shape, "bodyColliderType"),
            {1: "Box", 2: "Sphere", 3: "Capsule"},
        )

    def test_dense_enum_values_start_at_zero(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        self.assertEqual(
            rbi.enum_fields(scene.body_shapes["hip"], "bodyPhysicsMode"),
            {0: "FollowBone", 1: "Physics", 2: "PhysicsBone"},
        )

    def test_unknown_enum_is_empty(self) -> None:
        scene = self.scene()
        scene.add_body("hip")
        self.assertEqual(rbi.enum_fields(scene.body_shapes["hip"], "nope"), {})


if __name__ == "__main__":
    unittest.main()
