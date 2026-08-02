"""
bone_morph_node.py

Custom Maya dependency graph node for bone morph (pose) interpolation.

"""

import logging
import math
import sys
from typing import Tuple
import maya.api.OpenMaya as om

log = logging.getLogger(__name__)


def _slerp_quat(q1: om.MQuaternion, q2: om.MQuaternion, t: float) -> om.MQuaternion:
    """Spherical linear interpolation between two unit quaternions.

    Always takes the shortest arc. Falls back to normalised linear interpolation
    when q1 and q2 are nearly identical to avoid division by zero.
    """
    dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    # Ensure shortest-path interpolation
    if dot < 0.0:
        q2 = om.MQuaternion(-q2.x, -q2.y, -q2.z, -q2.w)
        dot = -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        # Quaternions nearly identical — normalised linear interpolation
        r = om.MQuaternion(
            q1.x + t * (q2.x - q1.x),
            q1.y + t * (q2.y - q1.y),
            q1.z + t * (q2.z - q1.z),
            q1.w + t * (q2.w - q1.w),
        )
        r.normalizeIt()
        return r
    theta_0 = math.acos(dot)
    theta = theta_0 * t
    s0 = math.cos(theta) - dot * math.sin(theta) / math.sin(theta_0)
    s1 = math.sin(theta) / math.sin(theta_0)
    return om.MQuaternion(
        s0 * q1.x + s1 * q2.x,
        s0 * q1.y + s1 * q2.y,
        s0 * q1.z + s1 * q2.z,
        s0 * q1.w + s1 * q2.w,
    )


# Plugin information
kPluginNodeName = "boneMorphNode"
kPluginNodeId = om.MTypeId(0x39390053)  # Stable node ID (reserved for MayaMMD)
kPluginNodeClassify = "utility/general"


class BoneMorphNode(om.MPxNode):
    """
    Custom dependency graph node for bone morph interpolation.

    This node takes a weight value (0-1) and interpolates between a rest pose
    and a target pose using quaternion slerp for rotations and linear interpolation
    for positions.
    """

    # ---------------#
    # --- Inputs ---#
    # ---------------#
    aWeight = om.MObject()  # Multi attribute for weights (similar to blendShape)

    # Compound array attribute to store morph target data
    aMorphTargets = om.MObject()
    aName = om.MObject()
    aBoneNames = om.MObject()
    aPositionOffset = om.MObject()
    aRotationOffset = om.MObject()

    # ---------------#
    # --- Outputs ---#
    # ---------------#
    aOutputRotate = om.MObject()  # Output rotation array (Euler XYZ degrees) for direct connection to joint.rotate via plusMinusAverage
    aOutputTranslate = om.MObject()  # Output position array (X, Y, Z) for direct connection to joint.translate via plusMinusAverage

    def __init__(self):
        """Initialize the node."""
        om.MPxNode.__init__(self)

    def legalConnection(self, plug, otherPlug, asSrc):
        """
        Check if a connection between attributes is legal.

        This method is called by Maya when making connections to/from this node.

        Args:
            plug: The plug on this node
            otherPlug: The plug on the other node
            asSrc: True if this node's plug is the source, False if it's the destination

        Returns:
            bool: True if the connection is allowed, False otherwise
        """
        # Allow all connections for now
        # In the future, you might want to restrict certain connections
        return True

    def _read_morph_targets_from_attributes(self, dataBlock):
        """
        Read all morph targets from aMorphTargets attribute.

        Returns list of dicts with target data:
        [
            {
                "name": "Smile",
                "weight_index": 0,
                "joint_names": ["joint1", "joint2"],
                "pos_offsets": [(x,y,z), ...],
                "rot_offsets": [(x,y,z,w), ...]
            },
            ...
        ]
        """
        targets = []
        morph_targets_handle = dataBlock.inputArrayValue(BoneMorphNode.aMorphTargets)

        weight_index = 0
        while not morph_targets_handle.isDone():
            compound_handle = morph_targets_handle.inputValue()

            # Read target name
            name_child = compound_handle.child(BoneMorphNode.aName)
            target_name = name_child.asString()

            # Read bone names array
            joint_names = []
            bone_names_child = compound_handle.child(BoneMorphNode.aBoneNames)
            bone_names_array = om.MArrayDataHandle(bone_names_child)
            while not bone_names_array.isDone():
                bone_name = bone_names_array.inputValue().asString()
                if bone_name:
                    joint_names.append(bone_name)
                bone_names_array.next()

            # Read position offsets array
            pos_offsets = []
            pos_child = compound_handle.child(BoneMorphNode.aPositionOffset)
            pos_array = om.MArrayDataHandle(pos_child)
            while not pos_array.isDone():
                x, y, z = pos_array.inputValue().asDouble3()
                pos_offsets.append((x, y, z))
                pos_array.next()

            # Read rotation offsets array
            rot_offsets = []
            rot_child = compound_handle.child(BoneMorphNode.aRotationOffset)
            rot_array = om.MArrayDataHandle(rot_child)
            while not rot_array.isDone():
                x, y, z, w = rot_array.inputValue().asDouble4()
                rot_offsets.append((x, y, z, w))
                rot_array.next()

            targets.append(
                {
                    "name": target_name,
                    "weight_index": weight_index,
                    "joint_names": joint_names,
                    "pos_offsets": pos_offsets,
                    "rot_offsets": rot_offsets,
                }
            )

            weight_index += 1
            morph_targets_handle.next()

        return targets

    def _build_bone_to_output_index_map(self, dataBlock):
        """
        Build bone_name -> output_index mapping from aMorphTargets attribute.

        Scans all morph targets and assigns a unique output index to each unique bone name
        in the order they are first encountered.

        Returns tuple: (bone_to_output_index dict, output_index_to_bone dict)
        """
        bone_to_output_index = {}
        output_index_to_bone = {}
        next_index = 0

        morph_targets_handle = dataBlock.inputArrayValue(BoneMorphNode.aMorphTargets)

        while not morph_targets_handle.isDone():
            compound_handle = morph_targets_handle.inputValue()

            # Read bone names from this target
            bone_names_child = compound_handle.child(BoneMorphNode.aBoneNames)
            bone_names_array = om.MArrayDataHandle(bone_names_child)

            while not bone_names_array.isDone():
                bone_name = bone_names_array.inputValue().asString()
                if bone_name and bone_name not in bone_to_output_index:
                    # Assign new output index
                    bone_to_output_index[bone_name] = next_index
                    output_index_to_bone[next_index] = bone_name
                    next_index += 1
                bone_names_array.next()

            morph_targets_handle.next()

        return bone_to_output_index, output_index_to_bone

    def _get_parent_joint_name(self, joint_name):
        """Return the name of joint_name's parent if it is a joint, else None."""
        try:
            sel = om.MSelectionList()
            sel.add(joint_name)
            dag_path = sel.getDagPath(0)
            dag_fn = om.MFnDagNode(dag_path)
            if dag_fn.parentCount() == 0:
                return None
            parent_obj = dag_fn.parent(0)
            if parent_obj.hasFn(om.MFn.kJoint):
                parent_dag_fn = om.MFnDagNode(parent_obj)
                return parent_dag_fn.partialPathName()
            return None
        except Exception:
            return None

    def _get_joint_world_orientation_quat(self, joint_name):
        """Return the world-rest orientation of joint_name as an MQuaternion.

        Reads the world matrix via ``MDagPath.inclusiveMatrix()`` (OpenMaya 2.0),
        which is available on ``MDagPath`` directly — avoids depending on
        ``MFnTransform`` or ``MFnDagNode`` method exposure.
        """
        try:
            sel = om.MSelectionList()
            sel.add(joint_name)
            dag_path = sel.getDagPath(0)
            world_matrix = dag_path.inclusiveMatrix()
            tm = om.MTransformationMatrix(world_matrix)
            return tm.rotation(asQuaternion=True)
        except Exception as e:
            log.warning("Could not get world orientation for '%s': %s", joint_name, e)
            return om.MQuaternion()  # identity

    def compute(self, plug, dataBlock):
        # Handle both rotation and translation outputs
        if plug.attribute() == BoneMorphNode.aOutputRotate:
            # Read morph targets and build bone mapping from attributes (stateless)
            targets = self._read_morph_targets_from_attributes(dataBlock)
            bone_to_output_index, output_index_to_bone = (
                self._build_bone_to_output_index_map(dataBlock)
            )
            output_count = len(output_index_to_bone)

            # Read entire weight array once; individual elements are accessed inside the loop
            weight_array_handle = dataBlock.inputArrayValue(BoneMorphNode.aWeight)

            array_handle = dataBlock.outputArrayValue(BoneMorphNode.aOutputRotate)
            builder = array_handle.builder()

            if plug.isElement:
                # Maya requested a single element — compute only that slot
                element_index = plug.logicalIndex()
                if 0 <= element_index < output_count:
                    rx, ry, rz = self._compute_blended_rotation_for_index(
                        element_index,
                        weight_array_handle,
                        targets,
                        output_index_to_bone,
                    )
                    builder.addElement(element_index).set3Double(rx, ry, rz)
            else:
                # Maya requested the full array
                for idx in range(output_count):
                    rx, ry, rz = self._compute_blended_rotation_for_index(
                        idx, weight_array_handle, targets, output_index_to_bone
                    )
                    builder.addElement(idx).set3Double(rx, ry, rz)

            array_handle.set(builder)
            dataBlock.setClean(plug)

        elif plug.attribute() == BoneMorphNode.aOutputTranslate:
            # Read morph targets and build bone mapping from attributes (stateless)
            targets = self._read_morph_targets_from_attributes(dataBlock)
            bone_to_output_index, output_index_to_bone = (
                self._build_bone_to_output_index_map(dataBlock)
            )
            output_count = len(output_index_to_bone)

            # Read entire weight array once
            weight_array_handle = dataBlock.inputArrayValue(BoneMorphNode.aWeight)

            array_handle = dataBlock.outputArrayValue(BoneMorphNode.aOutputTranslate)
            builder = array_handle.builder()

            if plug.isElement:
                # Maya requested a single element
                element_index = plug.logicalIndex()
                if 0 <= element_index < output_count:
                    tx, ty, tz = self._compute_blended_translation_for_index(
                        element_index,
                        weight_array_handle,
                        targets,
                        output_index_to_bone,
                    )
                    builder.addElement(element_index).set3Double(tx, ty, tz)
            else:
                # Maya requested the full array
                for idx in range(output_count):
                    tx, ty, tz = self._compute_blended_translation_for_index(
                        idx, weight_array_handle, targets, output_index_to_bone
                    )
                    builder.addElement(idx).set3Double(tx, ty, tz)

            array_handle.set(builder)
            dataBlock.setClean(plug)

        else:
            # Unknown plug — nothing to compute, silently ignore (API 2.0 returns None)
            return

    def _get_output_rotate_order(self, output_index: int) -> int:
        """Read rotateOrder from the MORPH_ controller connected to ``outputRotate[output_index]``.

        Follows the DG connection from this node's ``outputRotate[i]`` plug to the
        destination MORPH_ controller node and reads its ``rotateOrder`` via the
        API 2.0 plug access pattern (avoids ``cmds.getAttr`` inside a DG compute).

        Returns:
            int: Maya rotation order (0-5), defaulting to 0 (XYZ) on error.
        """
        try:
            this_node = self.thisMObject()
            out_plug = om.MPlug(this_node, BoneMorphNode.aOutputRotate)
            elem_plug = out_plug.elementByLogicalIndex(output_index)
            destinations = elem_plug.destinations()
            if destinations:
                dest_node = destinations[0].node()
                dep_fn = om.MFnDependencyNode(dest_node)
                rot_order_plug = dep_fn.findPlug("rotateOrder", False)
                if not rot_order_plug.isNull():
                    rot_order = rot_order_plug.asInt()
                    if rot_order != 0:
                        ctrl_name = dep_fn.name()
                        log.warning(
                            "morphCtrl controller '%s' has unexpected rotateOrder %s "
                            "(expected 0/XYZ). Bone morph rotations may be incorrect.",
                            ctrl_name,
                            rot_order,
                        )
                    return rot_order
        except Exception as exc:
            log.debug(
                "Could not read rotateOrder from outputRotate[%d] connection: %s",
                output_index,
                exc,
            )
        return 0

    def _compute_blended_rotation_for_index(
        self,
        output_index: int,
        weight_array_handle: om.MArrayDataHandle,
        targets: list,
        output_index_to_bone: dict,
    ) -> Tuple[float, float, float]:
        """Blend all morph targets that affect the joint at *output_index*.

        Strategy (matches Maya blendShape additive blending):
          - Rotation:  slerp each target from identity then compose  →  ∏ slerp(I, R_i, w_i)

        Args:
            output_index: Index of the output rotation slot
            weight_array_handle: Handle to weight array
            targets: List of target dicts from _read_morph_targets_from_attributes()
            output_index_to_bone: Mapping from output index to bone name

        Returns (0, 0, 0) if the output index has no registered joint.
        Returns Euler XYZ angles in DEGREES for direct connection to joint.rotate.
        """
        joint_name = output_index_to_bone.get(output_index)
        if joint_name is None:
            return (0.0, 0.0, 0.0)

        identity_quat = om.MQuaternion()  # (0, 0, 0, 1)
        total_rot = om.MQuaternion()  # starts at identity

        for target in targets:
            joint_names = target["joint_names"]
            if joint_name not in joint_names:
                continue

            # Read weight once for this target
            w_idx = target["weight_index"]
            try:
                weight_array_handle.jumpToLogicalElement(w_idx)
                w = weight_array_handle.inputValue().asFloat()
            except Exception:
                w = 0.0

            if abs(w) < 1e-7:
                continue

            # MMD allows the same bone to appear multiple times in a single morph entry.
            # Each occurrence composes (multiplies) onto the result — iterate ALL of them.
            for joint_idx, jn in enumerate(joint_names):
                if jn != joint_name:
                    continue

                # --- Rotation: slerp from identity then compose ---
                rx, ry, rz, rw = target["rot_offsets"][joint_idx]
                target_quat = om.MQuaternion(rx, ry, rz, rw)
                partial_rot = _slerp_quat(identity_quat, target_quat, w)
                total_rot = total_rot * partial_rot

        # Convert quaternion to Euler in DEGREES using the MORPH_ controller's rotation
        # order. MORPH_ controllers are plain transform nodes created with default XYZ
        # order (0). All joints keep jointOrient = 0 (world-aligned), so the morph
        # quaternion is applied directly to .rotate with no frame conversion needed.
        # Read the actual value by following the DG connection from outputRotate[output_index]
        # to the actual MORPH_ controller node (which has a unique name), instead of using
        # the short bone name that may be ambiguous across multiple imported models.
        rot_order = self._get_output_rotate_order(output_index)

        # Map Maya rotation order (0-5) to MEulerRotation order constants
        order_map = [
            om.MEulerRotation.kXYZ,  # 0
            om.MEulerRotation.kYZX,  # 1
            om.MEulerRotation.kZXY,  # 2
            om.MEulerRotation.kXZY,  # 3
            om.MEulerRotation.kYXZ,  # 4
            om.MEulerRotation.kZYX,  # 5
        ]
        euler_order = order_map[rot_order]

        # Convert with specified rotation order
        euler = total_rot.asEulerRotation().reorder(euler_order)
        result = (math.degrees(euler.x), math.degrees(euler.y), math.degrees(euler.z))

        log.debug(
            "outputRotate[%d] for joint '%s': quat=(%.6f, %.6f, %.6f, %.6f) "
            "→ Euler deg=(%.4f, %.4f, %.4f)  rotOrder=%d",
            output_index,
            joint_name,
            total_rot.x,
            total_rot.y,
            total_rot.z,
            total_rot.w,
            result[0],
            result[1],
            result[2],
            rot_order,
        )

        return result

    def _compute_blended_translation_for_index(
        self,
        output_index: int,
        weight_array_handle: om.MArrayDataHandle,
        targets: list,
        output_index_to_bone: dict,
    ) -> Tuple[float, float, float]:
        """Blend all morph targets' position offsets for the joint at *output_index*.

        Strategy (matches Maya blendShape additive blending):
          - Position: linear weighted sum → Σ(P_i * w_i)

        Args:
            output_index: Index of the output translation slot
            weight_array_handle: Handle to weight array
            targets: List of target dicts from _read_morph_targets_from_attributes()
            output_index_to_bone: Mapping from output index to bone name

        Returns (0, 0, 0) if the output index has no registered joint.
        Returns position offset (X, Y, Z) for direct connection to joint.translate.
        """
        joint_name = output_index_to_bone.get(output_index)
        if joint_name is None:
            return (0.0, 0.0, 0.0)

        total_pos = [0.0, 0.0, 0.0]

        for target in targets:
            joint_names = target["joint_names"]
            if joint_name not in joint_names:
                continue

            # Read weight once for this target
            w_idx = target["weight_index"]
            try:
                weight_array_handle.jumpToLogicalElement(w_idx)
                w = weight_array_handle.inputValue().asFloat()
            except Exception:
                w = 0.0

            if abs(w) < 1e-7:
                continue

            # MMD allows the same bone to appear multiple times - accumulate all occurrences
            for joint_idx, jn in enumerate(joint_names):
                if jn != joint_name:
                    continue

                # Linear blend: accumulate weighted position offset
                px, py, pz = target["pos_offsets"][joint_idx]
                total_pos[0] += px * w
                total_pos[1] += py * w
                total_pos[2] += pz * w

        # Rotate the accumulated offset into the parent joint's local space.
        # joint.translate is expressed in the parent's local frame, so we must
        # transform the world-space offset by the inverse of the parent's
        # world-rest orientation.
        parent_joint = self._get_parent_joint_name(joint_name)
        if parent_joint is not None:
            parent_world_orient = self._get_joint_world_orientation_quat(parent_joint)
            inv_orient = parent_world_orient.inverse()
            v = om.MVector(total_pos[0], total_pos[1], total_pos[2])
            v = v.rotateBy(inv_orient)
            return (v.x, v.y, v.z)

        return (total_pos[0], total_pos[1], total_pos[2])

    @staticmethod
    def nodeCreator():
        """
        Creator function for the node.

        Returns:
            BoneMorphNode: New instance of the node
        """
        return BoneMorphNode()

    @staticmethod
    def nodeInitializer():
        """
        Initialize the node's attributes and set up attribute relationships.

        This function is called once when the plugin is loaded to define all
        attributes and their dependencies.
        """
        nAttr = om.MFnNumericAttribute()
        tAttr = om.MFnTypedAttribute()
        cAttr = om.MFnCompoundAttribute()

        # ===== Weight Attribute (multi/array attribute like blendShape) =====
        BoneMorphNode.aWeight = nAttr.create(
            "weight", "wt", om.MFnNumericData.kFloat, 0.0
        )
        nAttr.keyable = True
        nAttr.readable = True
        nAttr.writable = True
        nAttr.storable = True
        nAttr.array = True  # Make it an array attribute
        nAttr.setMin(0.0)
        nAttr.setMax(1.0)
        BoneMorphNode.addAttribute(BoneMorphNode.aWeight)

        # ===== Morph Target Data Attribute (compound array to store target pose data) =====
        BoneMorphNode.aName = tAttr.create("targetName", "n", om.MFnData.kString)
        tAttr.writable = True
        tAttr.readable = True
        tAttr.storable = True

        BoneMorphNode.aBoneNames = tAttr.create("boneNames", "bn2", om.MFnData.kString)
        tAttr.array = True
        tAttr.usesArrayDataBuilder = True
        tAttr.writable = True
        tAttr.readable = True
        tAttr.storable = True

        BoneMorphNode.aPositionOffset = nAttr.create(
            "positionOffset", "po", om.MFnNumericData.k3Double
        )
        nAttr.array = True
        nAttr.usesArrayDataBuilder = True
        nAttr.writable = True
        nAttr.readable = True
        nAttr.storable = True
        nAttr.default = (0.0, 0.0, 0.0)  # Default to no position offset

        BoneMorphNode.aRotationOffset = nAttr.create(
            "rotationOffset", "ro", om.MFnNumericData.k4Double
        )
        nAttr.array = True
        nAttr.usesArrayDataBuilder = True
        nAttr.writable = True
        nAttr.readable = True
        nAttr.storable = True
        nAttr.default = (0.0, 0.0, 0.0, 1.0)  # Identity quaternion as default

        BoneMorphNode.aMorphTargets = cAttr.create("morphTargets", "mt")
        cAttr.keyable = True
        cAttr.array = True
        cAttr.usesArrayDataBuilder = True
        cAttr.writable = True
        cAttr.readable = True
        cAttr.storable = True
        cAttr.addChild(BoneMorphNode.aName)
        cAttr.addChild(BoneMorphNode.aBoneNames)
        cAttr.addChild(BoneMorphNode.aPositionOffset)
        cAttr.addChild(BoneMorphNode.aRotationOffset)
        BoneMorphNode.addAttribute(BoneMorphNode.aMorphTargets)

        # ===== Output Rotate Attribute (array - one per affected bone) =====
        # outputRotate[0] corresponds to first unique bone, [1] to second, etc.
        # Output is Euler XYZ rotation in degrees for direct connection to joint.rotate
        BoneMorphNode.aOutputRotate = nAttr.create(
            "outputRotate", "outRot", om.MFnNumericData.k3Double
        )
        nAttr.keyable = False
        nAttr.readable = True
        nAttr.writable = False  # Output only - computed by compute() method
        nAttr.storable = False
        nAttr.cached = True  # Cache computed values for performance
        nAttr.array = True  # Array attribute - one element per bone
        nAttr.usesArrayDataBuilder = (
            True  # Node controls element allocation via MArrayDataBuilder
        )
        BoneMorphNode.addAttribute(BoneMorphNode.aOutputRotate)

        # ===== Output Translate Attribute (array - one per affected bone) =====
        # outputTranslate[0] corresponds to first unique bone, [1] to second, etc.
        # Output is position offset (X, Y, Z) for direct connection to joint.translate
        # Most bone morphs have zero translation, so BoneBlendShapeCmd only creates
        # connections when target has non-zero position offsets
        BoneMorphNode.aOutputTranslate = nAttr.create(
            "outputTranslate", "outTrans", om.MFnNumericData.k3Double
        )
        nAttr.keyable = False
        nAttr.readable = True
        nAttr.writable = False
        nAttr.storable = False
        nAttr.cached = True
        nAttr.array = True
        nAttr.usesArrayDataBuilder = True
        BoneMorphNode.addAttribute(BoneMorphNode.aOutputTranslate)

        # Set up attribute affects relationships
        # When weight or morphTargets change, output rotations should recompute
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aWeight, BoneMorphNode.aOutputRotate
        )
        BoneMorphNode.attributeAffects(BoneMorphNode.aName, BoneMorphNode.aOutputRotate)
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aBoneNames, BoneMorphNode.aOutputRotate
        )
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aPositionOffset, BoneMorphNode.aOutputRotate
        )
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aRotationOffset, BoneMorphNode.aOutputRotate
        )

        # When weight or morphTargets change, output translations should recompute
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aWeight, BoneMorphNode.aOutputTranslate
        )
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aName, BoneMorphNode.aOutputTranslate
        )
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aBoneNames, BoneMorphNode.aOutputTranslate
        )
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aPositionOffset, BoneMorphNode.aOutputTranslate
        )
        BoneMorphNode.attributeAffects(
            BoneMorphNode.aRotationOffset, BoneMorphNode.aOutputTranslate
        )
