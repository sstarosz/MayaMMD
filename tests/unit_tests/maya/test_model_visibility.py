"""Unit tests for :mod:`mmd.maya.model_visibility`.

The Maya hierarchy is faked (see :class:`_FakeScene`) so container resolution,
visibility reads/writes and the import defaults can be exercised without a Maya
runtime.
"""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from mmd.maya import model_visibility as mv


class _FakeScene:
    """Minimal DAG fake covering the ``cmds`` calls used by the module.

    Mirrors the hierarchy produced by PMX import::

        |{model}_Root
        |{model}_Root|{model}_Geo            -> {model}_Mesh (mesh shape)
        |{model}_Root|{model}_Bones          -> root_Jnt -> spine_Jnt
        |{model}_Root|{model}_RigidBodies
            |{model}_RigidBodySolver         -> pmxRigidBodyNode shape
            |{model}_Bodies                  -> {model}_hip_rb (pmxRigidBodyShape)
            |{model}_Joints                  -> {model}_knee_Constraint (pmxRigidBodyJoint)
    """

    def __init__(
        self, *, physics: bool = True, with_joints: bool = True, suffix: str = ""
    ) -> None:
        model = "Miku"
        self.root = f"|{model}_Root"
        geo = f"{self.root}|{model}_Geo{suffix}"
        mesh = f"{geo}|{model}_Mesh{suffix}"
        bones = f"{self.root}|{model}_Bones{suffix}"
        root_jnt = f"{bones}|root_Jnt{suffix}"
        spine_jnt = f"{root_jnt}|spine_Jnt{suffix}"
        inherit_ctrl = f"{bones}|shoulder_R_InheritCtrl{suffix}"
        physics_group = f"{self.root}|{model}_RigidBodies{suffix}"
        solver = f"{physics_group}|{model}_RigidBodySolver{suffix}"
        bodies = f"{physics_group}|{model}_Bodies{suffix}"
        hip = f"{bodies}|{model}_hip_rb{suffix}"
        joints = f"{physics_group}|{model}_Joints{suffix}"
        knee = f"{joints}|{model}_knee_Constraint{suffix}"

        self.children = {
            self.root: [geo, bones] + ([physics_group] if physics else []),
            geo: [mesh],
            mesh: [],
            bones: [root_jnt, inherit_ctrl],
            root_jnt: [spine_jnt],
            spine_jnt: [],
            inherit_ctrl: [],
        }
        self.shapes = {
            mesh: ["mesh"],
            root_jnt: ["joint"],
            spine_jnt: ["joint"],
        }
        if physics:
            # The per-body / per-joint sub-groups always exist, even when the
            # model only has bodies (no joints) or vice versa.
            self.children.update(
                {
                    physics_group: [solver, bodies, joints],
                    solver: [],
                    bodies: [hip],
                    hip: [],
                    joints: [knee] if with_joints else [],
                    knee: [],
                }
            )
            self.shapes.update(
                {
                    solver: ["pmxRigidBodyNode"],
                    hip: ["pmxRigidBodyShape"],
                }
            )
            if with_joints:
                self.shapes[knee] = ["pmxRigidBodyJoint"]

        self.visibility = dict.fromkeys(self.children, True)
        self.parents = {
            child: parent for parent, kids in self.children.items() for child in kids
        }

        # The importer hides its helper controllers on purpose; they must not
        # be reported as "hidden individually" nor revealed.
        self.inherit_ctrl: str = inherit_ctrl
        self.visibility[inherit_ctrl] = False

        self.geo: str = geo
        self.bones: str = bones
        # Empty when the fake model has no physics nodes.
        self.bodies: str = bodies if physics else ""
        self.joints: str = joints if physics else ""
        # Nodes *inside* the containers (these are the "hidden individually" ones).
        self.mesh: str = mesh
        self.root_jnt: str = root_jnt
        self.spine_jnt: str = spine_jnt
        self.hip: str = hip if physics else ""
        self.knee: str = knee if (physics and with_joints) else ""

    # ── Maya command emulation ───────────────────────────────────────────

    def obj_exists(self, node: str) -> bool:
        return node in self.children

    def list_relatives(self, node: str, **kwargs) -> list[str]:
        if kwargs.get("parent"):
            nodes = node if isinstance(node, list) else [node]
            return [self.parents[n] for n in nodes if n in self.parents]
        if kwargs.get("children"):
            return list(self.children.get(node, []))
        shape_type = kwargs.get("type")
        if kwargs.get("shapes"):
            return [s for s in self.shapes.get(node, []) if shape_type in (None, s)]
        if kwargs.get("allDescendents"):
            found: list[str] = []
            stack = list(self.children.get(node, []))
            while stack:
                child = stack.pop()
                if shape_type is None or shape_type in self.shapes.get(child, []):
                    found.append(child)
                stack.extend(self.children.get(child, []))
            return found
        return []

    def node_type(self, nodes):
        def one(node: str) -> str:
            return (self.shapes.get(node) or ["transform"])[0]

        if isinstance(nodes, list):
            return [one(node) for node in nodes]
        return one(nodes)

    def get_attr(self, plug):
        if isinstance(plug, list):
            return [self.get_attr(item) for item in plug]
        node, _, attr = plug.rpartition(".")
        if attr != "visibility" or node not in self.visibility:
            raise RuntimeError(f"no attribute {plug}")
        return self.visibility[node]

    def set_attr(self, plug: str, value) -> None:
        node, _, attr = plug.rpartition(".")
        if attr != "visibility" or node not in self.visibility:
            raise RuntimeError(f"no attribute {plug}")
        self.visibility[node] = bool(value)

    def install(self, cmds: MagicMock) -> None:
        cmds.objExists.side_effect = self.obj_exists
        cmds.listRelatives.side_effect = self.list_relatives
        cmds.getAttr.side_effect = self.get_attr
        cmds.setAttr.side_effect = self.set_attr


class _VisibilityTestBase(unittest.TestCase):
    def setUp(self) -> None:
        patcher = patch("mmd.maya.model_visibility.cmds")
        self.cmds = patcher.start()
        self.addCleanup(patcher.stop)
        self.scene = _FakeScene()
        self.scene.install(self.cmds)


class TestPartTables(unittest.TestCase):
    def test_tables_cover_every_part(self) -> None:
        self.assertEqual(set(mv.DEFAULT_VISIBILITY), set(mv.PARTS))
        self.assertEqual(set(mv.PART_LABELS), set(mv.PARTS))

    def test_physics_is_hidden_by_default(self) -> None:
        self.assertTrue(mv.DEFAULT_VISIBILITY[mv.MESH])
        self.assertTrue(mv.DEFAULT_VISIBILITY[mv.BONES])
        self.assertFalse(mv.DEFAULT_VISIBILITY[mv.RIGID_BODIES])
        self.assertFalse(mv.DEFAULT_VISIBILITY[mv.JOINTS])


class TestResolvePartNodes(_VisibilityTestBase):
    def test_finds_every_container(self) -> None:
        nodes = mv.resolve_part_nodes(self.scene.root)
        self.assertEqual(nodes[mv.MESH], self.scene.geo)
        self.assertEqual(nodes[mv.BONES], self.scene.bones)
        self.assertEqual(nodes[mv.RIGID_BODIES], self.scene.bodies)
        self.assertEqual(nodes[mv.JOINTS], self.scene.joints)

    def test_missing_root_returns_no_containers(self) -> None:
        self.assertEqual(
            mv.resolve_part_nodes("|does_not_exist_Root"),
            dict.fromkeys(mv.PARTS),
        )

    def test_empty_root_returns_no_containers(self) -> None:
        self.assertEqual(mv.resolve_part_nodes(""), dict.fromkeys(mv.PARTS))

    def test_model_without_physics(self) -> None:
        scene = _FakeScene(physics=False)
        scene.install(self.cmds)
        nodes = mv.resolve_part_nodes(scene.root)
        self.assertEqual(nodes[mv.MESH], scene.geo)
        self.assertEqual(nodes[mv.BONES], scene.bones)
        self.assertIsNone(nodes[mv.RIGID_BODIES])
        self.assertIsNone(nodes[mv.JOINTS])

    def test_model_with_bodies_but_no_joints(self) -> None:
        scene = _FakeScene(with_joints=False)
        scene.install(self.cmds)
        nodes = mv.resolve_part_nodes(scene.root)
        self.assertEqual(nodes[mv.RIGID_BODIES], scene.bodies)
        self.assertIsNone(nodes[mv.JOINTS])

    def test_renamed_containers_still_resolve(self) -> None:
        """``make_unique`` suffixes lookups after a repeated import."""
        scene = _FakeScene(suffix="_1")
        scene.install(self.cmds)
        nodes = mv.resolve_part_nodes(scene.root)
        self.assertEqual(nodes[mv.MESH], scene.geo)
        self.assertEqual(nodes[mv.BONES], scene.bones)
        self.assertEqual(nodes[mv.RIGID_BODIES], scene.bodies)
        self.assertEqual(nodes[mv.JOINTS], scene.joints)


class TestGetPartVisibility(_VisibilityTestBase):
    def test_reads_container_visibility(self) -> None:
        self.scene.visibility[self.scene.bodies] = False
        states = mv.get_part_visibility(self.scene.root)
        self.assertEqual(
            states,
            {
                mv.MESH: True,
                mv.BONES: True,
                mv.RIGID_BODIES: False,
                mv.JOINTS: True,
            },
        )

    def test_missing_parts_report_none(self) -> None:
        scene = _FakeScene(physics=False)
        scene.install(self.cmds)
        states = mv.get_part_visibility(scene.root)
        self.assertIsNone(states[mv.RIGID_BODIES])
        self.assertIsNone(states[mv.JOINTS])
        self.assertTrue(states[mv.MESH])

    def test_unreadable_visibility_reports_none(self) -> None:
        self.cmds.getAttr.side_effect = RuntimeError("boom")
        states = mv.get_part_visibility(self.scene.root)
        self.assertEqual(states, dict.fromkeys(mv.PARTS))


class TestSetPartVisibility(_VisibilityTestBase):
    def test_hides_and_shows_a_part(self) -> None:
        self.assertTrue(mv.set_part_visibility(self.scene.root, mv.JOINTS, False))
        self.assertFalse(self.scene.visibility[self.scene.joints])

        self.assertTrue(mv.set_part_visibility(self.scene.root, mv.JOINTS, True))
        self.assertTrue(self.scene.visibility[self.scene.joints])

    def test_only_the_requested_container_changes(self) -> None:
        mv.set_part_visibility(self.scene.root, mv.MESH, False)
        self.assertFalse(self.scene.visibility[self.scene.geo])
        self.assertTrue(self.scene.visibility[self.scene.bones])
        self.assertTrue(self.scene.visibility[self.scene.bodies])

    def test_missing_part_is_not_written(self) -> None:
        scene = _FakeScene(physics=False)
        scene.install(self.cmds)
        self.assertFalse(mv.set_part_visibility(scene.root, mv.RIGID_BODIES, True))
        self.cmds.setAttr.assert_not_called()

    def test_unknown_part_raises(self) -> None:
        with self.assertRaises(ValueError):
            mv.set_part_visibility(self.scene.root, "shadows", True)

    def test_maya_failure_is_reported(self) -> None:
        self.cmds.setAttr.side_effect = RuntimeError("locked")
        self.assertFalse(mv.set_part_visibility(self.scene.root, mv.MESH, False))


class TestApplyDefaultVisibility(_VisibilityTestBase):
    def test_hides_physics_and_shows_geometry(self) -> None:
        states = mv.apply_default_visibility(self.scene.root)
        self.assertEqual(
            states,
            {
                mv.MESH: True,
                mv.BONES: True,
                mv.RIGID_BODIES: False,
                mv.JOINTS: False,
            },
        )
        self.assertTrue(self.scene.visibility[self.scene.geo])
        self.assertTrue(self.scene.visibility[self.scene.bones])
        self.assertFalse(self.scene.visibility[self.scene.bodies])
        self.assertFalse(self.scene.visibility[self.scene.joints])

    def test_is_a_noop_without_a_model(self) -> None:
        mv.apply_default_visibility("|does_not_exist_Root")
        self.cmds.setAttr.assert_not_called()

    def test_resolves_the_hierarchy_only_once(self) -> None:
        """Resolution walks descendants; import must not pay for it per part."""
        with patch.object(
            mv, "resolve_part_nodes", wraps=mv.resolve_part_nodes
        ) as resolve:
            mv.apply_default_visibility(self.scene.root)
        self.assertEqual(resolve.call_count, 1)

    def test_reports_what_the_scene_ended_up_with(self) -> None:
        """The returned dict is read back, not just the requested values."""
        self.scene.visibility[self.scene.geo] = False  # locked/overridden
        original_set = self.cmds.setAttr.side_effect

        def reject_mesh(plug: str, value) -> None:
            if plug == f"{self.scene.geo}.visibility":
                return  # silently ignored, like a locked attribute
            original_set(plug, value)

        self.cmds.setAttr.side_effect = reject_mesh
        states = mv.apply_default_visibility(self.scene.root)
        self.assertFalse(states[mv.MESH], "should report the real scene value")


class TestSetAllPartsVisibility(_VisibilityTestBase):
    def test_hides_every_part(self) -> None:
        changed = mv.set_all_parts_visibility(self.scene.root, False)

        self.assertEqual(changed, len(mv.PARTS))
        for node in (
            self.scene.geo,
            self.scene.bones,
            self.scene.bodies,
            self.scene.joints,
        ):
            self.assertFalse(self.scene.visibility[node], f"{node} was not hidden")

    def test_shows_every_part(self) -> None:
        mv.set_all_parts_visibility(self.scene.root, False)
        changed = mv.set_all_parts_visibility(self.scene.root, True)

        self.assertEqual(changed, len(mv.PARTS))
        for node in (
            self.scene.geo,
            self.scene.bones,
            self.scene.bodies,
            self.scene.joints,
        ):
            self.assertTrue(self.scene.visibility[node], f"{node} was not shown")

    def test_skips_parts_the_model_lacks(self) -> None:
        scene = _FakeScene(physics=False)
        scene.install(self.cmds)
        changed = mv.set_all_parts_visibility(scene.root, False)

        self.assertEqual(changed, 2, "only mesh + bones exist")
        self.cmds.setAttr.assert_called()

    def test_missing_model_changes_nothing(self) -> None:
        self.assertEqual(mv.set_all_parts_visibility("|nope_Root", True), 0)
        self.cmds.setAttr.assert_not_called()

    def test_keeps_nodes_hidden_individually(self) -> None:
        """Showing parts must not undo a deliberate per-node choice."""
        self.scene.visibility[self.scene.mesh] = False  # hidden by hand
        mv.set_all_parts_visibility(self.scene.root, False)
        mv.set_all_parts_visibility(self.scene.root, True)

        self.assertFalse(self.scene.visibility[self.scene.mesh])
        self.assertTrue(self.scene.visibility[self.scene.geo])


class TestHiddenDescendantCounts(_VisibilityTestBase):
    def test_reports_nothing_when_everything_is_visible(self) -> None:
        self.assertEqual(
            mv.hidden_descendant_counts(self.scene.root), dict.fromkeys(mv.PARTS, 0)
        )

    def test_counts_nodes_hidden_individually(self) -> None:
        self.scene.visibility[self.scene.mesh] = False  # the mesh transform
        self.scene.visibility[self.scene.hip] = False  # one rigid-body guide

        counts = mv.hidden_descendant_counts(self.scene.root)
        self.assertEqual(counts[mv.MESH], 1)
        self.assertEqual(counts[mv.RIGID_BODIES], 1)
        self.assertEqual(counts[mv.BONES], 0)

    def test_container_itself_is_not_counted(self) -> None:
        self.scene.visibility[self.scene.bones] = False  # the container
        self.assertEqual(mv.hidden_descendant_counts(self.scene.root)[mv.BONES], 0)

    def test_ignores_helper_controllers_hidden_by_the_importer(self) -> None:
        """``_InheritCtrl`` / ``_MorphCtrl`` are hidden on purpose."""
        self.assertFalse(self.scene.visibility[self.scene.inherit_ctrl])
        self.assertEqual(mv.hidden_descendant_counts(self.scene.root)[mv.BONES], 0)

    def test_missing_model_reports_zero(self) -> None:
        self.assertEqual(
            mv.hidden_descendant_counts("|nope_Root"), dict.fromkeys(mv.PARTS, 0)
        )


class TestRevealAll(_VisibilityTestBase):
    def test_reveals_parts_and_individually_hidden_nodes(self) -> None:
        self.scene.visibility[self.scene.mesh] = False
        self.scene.visibility[self.scene.root_jnt] = False
        self.scene.visibility[self.scene.bones] = False
        self.scene.visibility[self.scene.joints] = False

        revealed = mv.reveal_all(self.scene.root)

        self.assertGreater(revealed, 0)
        self.assertTrue(self.scene.visibility[self.scene.mesh])
        self.assertTrue(self.scene.visibility[self.scene.root_jnt])
        self.assertTrue(self.scene.visibility[self.scene.bones])
        self.assertTrue(self.scene.visibility[self.scene.joints])
        self.assertEqual(
            mv.hidden_descendant_counts(self.scene.root), dict.fromkeys(mv.PARTS, 0)
        )

    def test_returns_zero_when_nothing_is_hidden(self) -> None:
        self.assertEqual(mv.reveal_all(self.scene.root), 0)

    def test_leaves_helper_controllers_hidden(self) -> None:
        self.scene.visibility[self.scene.mesh] = False
        mv.reveal_all(self.scene.root)
        self.assertFalse(
            self.scene.visibility[self.scene.inherit_ctrl],
            "an intentionally hidden helper was revealed",
        )

    def test_missing_model_changes_nothing(self) -> None:
        self.assertEqual(mv.reveal_all("|nope_Root"), 0)
        self.cmds.setAttr.assert_not_called()
