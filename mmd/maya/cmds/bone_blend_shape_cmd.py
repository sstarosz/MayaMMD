"""
boneBlendShape command for managing bone morph targets.

v1.0 Operations:
    -addTarget      : Add bone morph target. Stores target data on the node,
                      creates MORPH_ controller transforms, and inserts them
                      in the DAG hierarchy above each main joint.
                      Follows the same pattern as INH_ controllers for INHERIT_ROTATION.
                      The MORPH_ controller is driven by boneMorphNode via DG;
                      the main joint inherits rotation through the DAG hierarchy.
                      Mesh stays skinned to original joints. MORPH_ controllers are plain DAG nodes,
                      invisible in the viewport, not clickable.)
                      Tuple syntax: -addTarget (name, "joint1,joint2", "x,y,z;...", "x,y,z,w;...")
                      String syntax: -addTarget "name|joint1,joint2|x,y,z;...|x,y,z,w;..."
    -listTargets    : Query list of target names.

Note MORPH_ controllers are created by
``create_bone_morph_helper_joints`` in this module. The controller is inserted
above the main joint in the DAG hierarchy (same pattern as INH_ controllers
for INHERIT_ROTATION). ``boneMorphNode`` drives the controller via DG; the
main joint inherits rotation through DAG. The Pose Tree widget sliders only
set weights — DG propagation handles the rest.
"""

import logging
import re
import sys
import maya.api.OpenMaya as om
import maya.cmds as cmds

from mmd.maya.pmx_naming_manager import (
    INHERIT_ROTATION_CONTROLLER_SUFFIX,
    MORPH_CONTROLLER_SUFFIX,
)

log = logging.getLogger(__name__)


def maya_useNewAPI():
    """Tell Maya to use the Python API 2.0."""
    pass


# ----------------------------------------------------------------------
# Helper functions for robust controller naming
# ----------------------------------------------------------------------


def _sanitize_name_for_maya(name: str) -> str:
    """
    Sanitize a name for Maya compatibility.
    Replaces special characters with underscores and ensures valid Maya node name.

    Args:
        name: Name to sanitize

    Returns:
        Sanitized name safe for Maya node creation
    """
    # Replace special characters (|, :, etc.) with underscores
    name = re.sub(r"[^a-zA-Z0-9_]", "_", name)

    # Remove multiple consecutive underscores
    name = re.sub(r"_+", "_", name)

    # Remove leading/trailing underscores
    name = name.strip("_")

    # Ensure name doesn't start with a number
    if name and name[0].isdigit():
        name = f"n_{name}"

    return name


def _get_controller_name_for_joint(
    joint_name: str, suffix: str = MORPH_CONTROLLER_SUFFIX
) -> str:
    """
    Generate a controller name for a joint.

    Uses only the joint's short name (last DAG component) + suffix, because
    joint short names are unique within a single PMX model.  The naming
    manager (``pmx_naming_manager.make_unique``) handles disambiguation
    across multiple model imports by appending numeric suffixes to the
    *joint* name itself, which naturally carries through to the controller.

    Args:
        joint_name: Joint name (short or long DAG path)
        suffix: Suffix for the controller name (e.g., ``_morphCtrl`` or ``_inheritCtrl``)

    Returns:
        Controller name (short name only, no path hash)
    """
    # Always use the short (last) component — no path hashing needed.
    short_name = joint_name.split("|")[-1].split(":")[-1]
    sanitized = _sanitize_name_for_maya(short_name)
    return f"{sanitized}{suffix}"


def _find_existing_controller(
    joint_name: str, suffix: str = MORPH_CONTROLLER_SUFFIX
) -> str | None:
    """
    Find an existing controller for a joint.
    Checks both the expected controller name and searches the hierarchy.

    Args:
        joint_name: Joint name to find controller for
        suffix: Suffix of the controller (e.g., ``_morphCtrl`` or ``_inheritCtrl``)

    Returns:
        Controller name if found, None otherwise
    """
    # Check if the joint exists
    if not cmds.objExists(joint_name):
        return None

    # 1. Check if the joint's parent is a controller (most reliable after
    #    reparenting — the DAG path is always current).
    parents = cmds.listRelatives(joint_name, parent=True, fullPath=True)
    if parents:
        parent_short_name = parents[0].split("|")[-1]
        # Use 'in' instead of 'endswith' to match both:
        #   ``joint_morphCtrl``           (plain)
        #   ``joint_morphCtrl_6921``      (hash-suffixed, path changed)
        if suffix in parent_short_name:
            return parent_short_name

    # 2. Fallback: check the expected name (works before any reparenting
    #    has occurred, or when the hash is still correct).
    expected_ctrl_name = _get_controller_name_for_joint(joint_name, suffix)
    if cmds.objExists(expected_ctrl_name):
        return expected_ctrl_name

    return None


# ----------------------------------------------------------------------
# Command Flags (without dash prefix - Maya API style)
kAddTargetFlag = "at"  # short
kAddTargetFlagLong = "addTarget"

kListTargetsFlag = "lt"
kListTargetsFlagLong = "listTargets"


# ----------------------------------------------------------------------
class BoneBlendShapeCmd(om.MPxCommand):
    """Command to manage boneBlendShape node targets and connections."""

    kName = "boneBlendShape"

    def __init__(self):
        om.MPxCommand.__init__(self)
        self._nodeName = ""  # node being edited
        self._isQuery = False  # query mode
        self._modifier = None  # MDGModifier for undoable DG connections

    @staticmethod
    def cmdCreator():
        return BoneBlendShapeCmd()

    @staticmethod
    def syntaxCreator():
        syntax = om.MSyntax()
        syntax.addArg(om.MSyntax.kString)  # node name

        # v1.0 flags
        syntax.addFlag(
            kAddTargetFlag,
            kAddTargetFlagLong,
            (
                om.MSyntax.kString,
                om.MSyntax.kString,
                om.MSyntax.kString,
                om.MSyntax.kString,
            ),
        )
        syntax.addFlag(kListTargetsFlag, kListTargetsFlagLong, om.MSyntax.kNoArg)

        syntax.enableEdit = True
        syntax.enableQuery = True
        return syntax

    # ------------------------------------------------------------------
    # MPxCommand overrides
    def isUndoable(self):
        # Undo only for operations that modify the node (add/remove)
        if self._isQuery:
            return False
        return True

    def doIt(self, args):
        """Execute the command."""
        try:
            # Parse arguments
            argData = om.MArgParser(BoneBlendShapeCmd.syntaxCreator(), args)

            # Get node name and validate
            self._nodeName = argData.commandArgumentString(0)
            self._isQuery = argData.isQuery

            if not self._nodeExistsAndIsValid(self._nodeName):
                raise RuntimeError(
                    f"Node '{self._nodeName}' does not exist or is not a boneMorphNode"
                )

            # Dispatch to query or edit mode
            if self._isQuery:
                self._doQuery(argData)
            else:
                self._doEdit(argData)
        except Exception as e:
            log.error(f"Command execution failed: {e}")
            raise

    def redoIt(self):
        """Redo the command."""
        if self._modifier is not None:
            self._modifier.doIt()

    def undoIt(self):
        """Undo the command."""
        if self._modifier is not None:
            self._modifier.undoIt()

    # ------------------------------------------------------------------
    # Internal helpers
    def _nodeExistsAndIsValid(self, nodeName):
        """Check if node exists and is a boneMorphNode."""
        if not cmds.objExists(nodeName):
            return False
        nodeType = cmds.nodeType(nodeName)
        return nodeType == "boneMorphNode"

    def _doQuery(self, argData):
        """Handle query mode."""
        result = None

        if argData.isFlagSet(kListTargetsFlag):
            result = self._queryListTargets()
        # Note: targetData query removed for v1.0 - not critical for read-only UI
        else:
            # No query flag: default query lists targets
            result = self._queryListTargets()

        if result is not None:
            self._setResult(result)

    def _doEdit(self, argData):
        """Handle edit mode (v1.0: addTarget only).

        Maya 2026 standalone has an intermittent bug where MPxCommand
        multi-string flags (addTarget with 4 args) sometimes fail to be
        detected by ``numberOfFlagUses`` even though the arguments were
        correctly passed.  We try multiple strategies to extract the flag.
        """
        name = None
        joints_str = None
        positions_str = None
        rotations_str = None

        # Strategy 1: numberOfFlagUses (normal path)
        if argData.numberOfFlagUses(kAddTargetFlag) > 0:
            try:
                name = argData.flagArgumentString(kAddTargetFlag, 0)
                joints_str = argData.flagArgumentString(kAddTargetFlag, 1)
                positions_str = argData.flagArgumentString(kAddTargetFlag, 2)
                rotations_str = argData.flagArgumentString(kAddTargetFlag, 3)
            except Exception:
                pass

        # Strategy 2: direct flagArgumentString (sometimes works when Strategy 1 doesn't)
        if name is None:
            try:
                name = argData.flagArgumentString(kAddTargetFlag, 0)
                joints_str = argData.flagArgumentString(kAddTargetFlag, 1)
                positions_str = argData.flagArgumentString(kAddTargetFlag, 2)
                rotations_str = argData.flagArgumentString(kAddTargetFlag, 3)
            except Exception:
                pass

        # Strategy 3: isFlagSet + flagArgumentString
        if name is None and argData.isFlagSet(kAddTargetFlag):
            try:
                name = argData.flagArgumentString(kAddTargetFlag, 0)
                joints_str = argData.flagArgumentString(kAddTargetFlag, 1)
                positions_str = argData.flagArgumentString(kAddTargetFlag, 2)
                rotations_str = argData.flagArgumentString(kAddTargetFlag, 3)
            except Exception:
                pass

        if name is None:
            raise RuntimeError("No edit flag specified")

        self._addTarget(name, joints_str, positions_str, rotations_str)

    # ------------------------------------------------------------------
    # Query methods
    def _queryListTargets(self):
        """Return list of target names from the node's morphTargets attribute."""
        target_names = []

        try:
            target_count = cmds.getAttr(f"{self._nodeName}.morphTargets", size=True)
            for i in range(target_count):
                name = cmds.getAttr(f"{self._nodeName}.morphTargets[{i}].targetName")
                if name:
                    target_names.append(name)
        except Exception as e:
            log.warning("Failed to query targets: %s", e)

        return target_names

    def _setResult(self, data):
        """Set command result (list of target names)."""
        self.setResult(data)

    def _build_bone_to_output_index_map(self, node_name):
        """
        Build bone name -> output index mapping from morphTargets attributes.

        Scans all morph targets and assigns a unique output index to each unique bone name
        in the order they are first encountered (same logic as node's internal mapping).

        Args:
            node_name: Name of the boneMorphNode

        Returns:
            dict: bone_name -> output_index mapping
        """
        bone_to_output_index = {}
        next_index = 0

        try:
            target_count = cmds.getAttr(f"{node_name}.morphTargets", size=True)

            for target_idx in range(target_count):
                bone_count = cmds.getAttr(
                    f"{node_name}.morphTargets[{target_idx}].boneNames", size=True
                )

                for bone_idx in range(bone_count):
                    bone_name = cmds.getAttr(
                        f"{node_name}.morphTargets[{target_idx}].boneNames[{bone_idx}]"
                    )
                    if bone_name and bone_name not in bone_to_output_index:
                        bone_to_output_index[bone_name] = next_index
                        next_index += 1
        except Exception as e:
            log.warning("Failed to build bone mapping: %s", e)

        return bone_to_output_index

    def _create_morph_controllers_for_joints(
        self, affected_joints: dict[str, set[int]]
    ) -> dict[str, str]:
        """
        Create MORPH_ controller transforms for joints that don't already have them.
        Only handles DAG hierarchy creation (controller + parenting).
        DG connections are handled separately by _connect_morph_controllers.

        Uses long-path-aware naming to handle edge cases:
        - Multiple joints with same short name in different branches
        - Existing controllers from previous operations
        - Namespace considerations

        Args:
            affected_joints: Maps joint name → set of outputRotate indices

        Returns:
            Dict mapping joint name → MORPH_ controller name (only newly created)
        """
        created_controllers: dict[str, str] = {}

        for joint_name in affected_joints.keys():
            # Validate joint exists
            if not cmds.objExists(joint_name):
                log.warning("Affected joint '%s' does not exist, skipping", joint_name)
                continue

            # Get long name to ensure we're working with the correct unique joint
            long_names = cmds.ls(joint_name, long=True)
            if not long_names:
                log.warning(
                    "Could not resolve long name for joint '%s', skipping", joint_name
                )
                continue

            joint_long_name = long_names[0]

            # Check if controller already exists
            existing_ctrl = _find_existing_controller(
                joint_long_name, MORPH_CONTROLLER_SUFFIX
            )
            if existing_ctrl:
                log.debug(
                    "MORPH controller '%s' already exists for joint '%s', skipping",
                    existing_ctrl,
                    joint_name,
                )
                continue

            # Generate unique controller name based on DAG path.
            # DAG paths are already unique per model (controllers sit under
            # the model's root), so the path-based hash in
            # _get_controller_name_for_joint prevents collisions.
            ctrl_name = _get_controller_name_for_joint(
                joint_long_name, MORPH_CONTROLLER_SUFFIX
            )

            # Create controller at root
            ctrl = cmds.createNode("transform", name=ctrl_name)

            # Controller stays at the ORIGIN of the parent's local space
            # (translate = 0, rotate = identity).  The joint keeps its own
            # local translate, so rest-pose capture and bind-pose reads are
            # unaffected by MORPH_ insertion.  The boneMorphNode output
            # (a parent-space delta) is then added on top of this zero
            # baseline via the outputTranslate connection below.
            joint_parent = cmds.listRelatives(
                joint_long_name, parent=True, fullPath=True
            )
            if joint_parent:
                cmds.parent(ctrl, joint_parent[0], relative=True)

            # Parent main joint under controller with relative=True so its
            # local translate is preserved exactly — jointOrient is also
            # preserved because neither the joint nor ctrl are rotated.
            cmds.parent(joint_long_name, ctrl, relative=True)

            created_controllers[joint_name] = ctrl_name
            log.debug(
                "Created MORPH controller '%s' above joint '%s'",
                ctrl_name,
                joint_long_name,
            )

        return created_controllers

    def _detect_joints_with_translation(self, joint_names: list[str]) -> set[str]:
        """
        Detect which joints have non-zero translation offsets in any morph target.

        Args:
            joint_names: List of joint names to check

        Returns:
            Set of joint names that have translation offsets
        """
        joints_with_translation = set()

        try:
            target_count = cmds.getAttr(f"{self._nodeName}.morphTargets", size=True)

            for target_idx in range(target_count):
                bone_count = cmds.getAttr(
                    f"{self._nodeName}.morphTargets[{target_idx}].boneNames", size=True
                )

                # Get position offsets for this target
                pos_count = cmds.getAttr(
                    f"{self._nodeName}.morphTargets[{target_idx}].positionOffset",
                    size=True,
                )

                for bone_idx in range(bone_count):
                    bone_name = cmds.getAttr(
                        f"{self._nodeName}.morphTargets[{target_idx}].boneNames[{bone_idx}]"
                    )

                    # Check if this bone is in our list and has position offset
                    if bone_name in joint_names and bone_idx < pos_count:
                        pos = cmds.getAttr(
                            f"{self._nodeName}.morphTargets[{target_idx}].positionOffset[{bone_idx}]"
                        )[0]

                        # Check if position offset is non-zero
                        if (
                            abs(pos[0]) > 1e-6
                            or abs(pos[1]) > 1e-6
                            or abs(pos[2]) > 1e-6
                        ):
                            joints_with_translation.add(bone_name)

        except Exception as e:
            log.warning("Failed to detect translation offsets: %s", e)

        return joints_with_translation

    def _connect_morph_controllers(
        self, affected_joints: dict[str, set[int]], controllers: dict[str, str]
    ):
        """
        Connect boneMorphNode outputs to MORPH_ controllers.
        Handles both rotation (always) and translation (when offsets exist).

        Args:
            affected_joints: Maps joint name → set of outputRotate indices
            controllers: Maps joint name → MORPH_ controller name
        """
        if not controllers:
            return

        # Detect which joints have translation offsets
        joint_names = list(controllers.keys())
        joints_with_translation = self._detect_joints_with_translation(joint_names)

        for joint_name, ctrl_name in controllers.items():
            output_idx = next(iter(affected_joints[joint_name]))

            # Connect rotation (always)
            # Touch outputRotate so the array element exists before connecting
            cmds.getAttr(f"{self._nodeName}.outputRotate[{output_idx}]")
            cmds.dgdirty(self._nodeName)
            cmds.connectAttr(
                f"{self._nodeName}.outputRotate[{output_idx}]",
                f"{ctrl_name}.rotate",
                force=True,
            )

            log.debug(
                "Connected outputRotate[%d] → %s.rotate",
                output_idx,
                ctrl_name,
            )

            # Connect translation if joint has translation offsets
            if joint_name in joints_with_translation:
                # Touch outputTranslate so the array element exists before connecting
                cmds.getAttr(f"{self._nodeName}.outputTranslate[{output_idx}]")
                cmds.dgdirty(self._nodeName)
                cmds.connectAttr(
                    f"{self._nodeName}.outputTranslate[{output_idx}]",
                    f"{ctrl_name}.translate",
                    force=True,
                )

                log.debug(
                    "Connected outputTranslate[%d] → %s.translate (has position offset)",
                    output_idx,
                    ctrl_name,
                )

    def _rewire_multiply_divide_inputs(self, helper_joints: dict[str, str]):
        """
        Rewire INHERIT_ROTATION multiplyDivide inputs to read from MORPH_
        controllers instead of the original joint.

        Uses long names for robust joint matching to handle edge cases with
        duplicate short names or namespaces.

        Args:
            helper_joints: Maps joint name → MORPH_ controller name
        """
        if not helper_joints:
            return

        log.debug(
            "Checking multiplyDivide nodes for %d joints: %s",
            len(helper_joints),
            list(helper_joints.keys()),
        )

        # Build a lookup map using long names for robust matching
        joint_long_to_ctrl: dict[str, str] = {}
        for joint_name, ctrl_name in helper_joints.items():
            if cmds.objExists(joint_name):
                long_names = cmds.ls(joint_name, long=True)
                if long_names:
                    joint_long_to_ctrl[long_names[0]] = ctrl_name

        # Iterate multiplyDivide nodes from the destination side
        for md in cmds.ls(type="multiplyDivide"):
            if not md.endswith("_RotScale"):
                continue

            for axis in ("X", "Y", "Z"):
                srcs = (
                    cmds.listConnections(f"{md}.input1{axis}", source=True, plugs=True)
                    or []
                )
                if not srcs:
                    continue

                src_plug = srcs[0]
                src_node = src_plug.split(".")[0]

                # Walk through unitConversion nodes Maya may have inserted
                while cmds.nodeType(src_node) == "unitConversion":
                    deeper = cmds.listConnections(
                        f"{src_node}.input", source=True, plugs=True
                    )
                    if deeper:
                        src_node = deeper[0].split(".")[0]
                    else:
                        break

                # Get long name for robust comparison
                src_long_names = cmds.ls(src_node, long=True)
                if not src_long_names:
                    continue

                src_long_name = src_long_names[0]

                # Check if this joint has a MORPH_ controller
                if src_long_name in joint_long_to_ctrl:
                    ctrl = joint_long_to_ctrl[src_long_name]
                    log.debug(
                        "Rewiring %s.input1%s from '%s' to '%s.rotate%s'",
                        md,
                        axis,
                        src_long_name,
                        ctrl,
                        axis,
                    )
                    cmds.disconnectAttr(src_plug, f"{md}.input1{axis}")
                    cmds.connectAttr(f"{ctrl}.rotate{axis}", f"{md}.input1{axis}")

    # ------------------------------------------------------------------
    # Edit methods
    def _addTarget(self, name, joints_str, positions_str, rotations_str):
        """Add a bone morph target.

        Writes target data to the boneMorphNode. DG connections to main joints
        and MORPH_ controller transforms are created separately by
        ``create_bone_morph_helper_joints()`` in this module.
        """
        # ---- Parse inputs ----
        joint_names = [j.strip() for j in joints_str.split(",") if j.strip()]

        pos_list = []
        for pos_str in positions_str.split(";"):
            if pos_str.strip():
                coords = [float(x.strip()) for x in pos_str.split(",")]
                if len(coords) == 3:
                    pos_list.append(coords)

        rot_list = []
        for rot_str in rotations_str.split(";"):
            if rot_str.strip():
                coords = [float(x.strip()) for x in rot_str.split(",")]
                if len(coords) == 4:
                    rot_list.append(coords)

        # ---- Resolve node ----
        sel = om.MSelectionList()
        sel.add(self._nodeName)
        node_obj = sel.getDependNode(0)

        dep_fn = om.MFnDependencyNode(node_obj)
        if dep_fn.typeName != "boneMorphNode":
            raise RuntimeError(
                f"Node '{self._nodeName}' has type '{dep_fn.typeName}', expected 'boneMorphNode'"
            )

        # ---- Calculate weight index from morphTargets attribute ----
        current_count = cmds.getAttr(f"{self._nodeName}.morphTargets", size=True)
        weight_index = current_count

        # ---- Create weight element with alias ----
        cmds.setAttr(f"{self._nodeName}.weight[{weight_index}]", 0.0)

        safe_name = name.replace(" ", "_").replace("-", "_")
        cmds.aliasAttr(safe_name, f"{self._nodeName}.weight[{weight_index}]")

        # ---- Write target data to morphTargets attributes ----
        cmds.setAttr(
            f"{self._nodeName}.morphTargets[{weight_index}].targetName",
            name,
            type="string",
        )

        for i, joint_name in enumerate(joint_names):
            cmds.setAttr(
                f"{self._nodeName}.morphTargets[{weight_index}].boneNames[{i}]",
                joint_name,
                type="string",
            )

        for i, pos in enumerate(pos_list):
            cmds.setAttr(
                f"{self._nodeName}.morphTargets[{weight_index}].positionOffset[{i}]",
                pos[0],
                pos[1],
                pos[2],
            )

        for i, rot in enumerate(rot_list):
            cmds.setAttr(
                f"{self._nodeName}.morphTargets[{weight_index}].rotationOffset[{i}]",
                rot[0],
                rot[1],
                rot[2],
                rot[3],
            )

        # Set alias for morphTargets attribute for easy lookup by name
        cmds.aliasAttr(f"PMX_{name}", f"{self._nodeName}.morphTargets[{weight_index}]")

        # ---- Build affected-joints map for this target ----
        # Scan all targets to build complete bone → output index mapping
        bone_to_output_idx = self._build_bone_to_output_index_map(self._nodeName)

        # Collect joints affected by THIS target specifically
        affected_joints: dict[str, set[int]] = {}
        for joint_name in joint_names:
            if joint_name in bone_to_output_idx:
                if joint_name not in affected_joints:
                    affected_joints[joint_name] = set()
                affected_joints[joint_name].add(bone_to_output_idx[joint_name])

        log.debug(
            "Target '%s' affects %d joints: %s",
            name,
            len(affected_joints),
            list(affected_joints.keys()),
        )

        # ---- Create MORPH_ controllers for joints in this target ----
        # Only create controllers for joints that don't already have them
        helper_joints_created = self._create_morph_controllers_for_joints(
            affected_joints
        )

        # ---- Connect MORPH_ controllers to boneMorphNode outputs ----
        # Handle both rotation and translation connections
        self._connect_morph_controllers(affected_joints, helper_joints_created)

        # ---- Rewire INHERIT_ROTATION multiplyDivide inputs ----
        # For any joints that now have MORPH_ controllers, rewire their
        # multiplyDivide inputs to read from the controller instead of the joint
        if helper_joints_created:
            self._rewire_multiply_divide_inputs(helper_joints_created)

        log.debug(
            "Added target '%s' with %d joints, created %d morph controllers",
            name,
            len(joint_names),
            len(helper_joints_created),
        )
        self._modifier = None


# Note: This command is registered via the main MayaMMD plugin (mmd/plugin.py).
# It should NOT be loaded as a standalone Maya plugin.
def maya_useNewAPI():
    """Disable standalone plugin loading - this command is registered via MayaMMD.mll"""
    pass
