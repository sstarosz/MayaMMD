import sys
import unittest


class _DummyMObject:
    pass


class _DummyMQuaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __mul__(self, other):
        return _DummyMQuaternion(
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        )

    def inverse(self):
        mag = (self.x**2 + self.y**2 + self.z**2 + self.w**2) ** 0.5
        if mag == 0:
            return _DummyMQuaternion(0, 0, 0, 1)
        # Assuming normalized for inverse (conjugate = inverse for unit quat)
        return _DummyMQuaternion(
            -self.x / mag, -self.y / mag, -self.z / mag, self.w / mag
        )


class _DummyMEulerRotation:
    kXYZ = 0
    kYZX = 1
    kZXY = 2
    kXZY = 3
    kYXZ = 4
    kZYX = 5


_mock_om = type(sys)("maya.api.OpenMaya")
_mock_om.MObject = _DummyMObject
_mock_om.MQuaternion = _DummyMQuaternion
_mock_om.MEulerRotation = _DummyMEulerRotation
_mock_om.MFnDependencyNode = type("MFnDependencyNode", (), {})
_mock_om.MPlug = type("MPlug", (), {})
_mock_om.MFn = type("MFn", (), {"kAnimCurve": 0, "kPlusMinusAverage": 0})
_mock_om.MMatrix = type("MMatrix", (), {})
_mock_om.MTransformationMatrix = type("MTransformationMatrix", (), {})
_mock_om.MVector = type("MVector", (), {})

_mock_oma = type(sys)("maya.api.OpenMayaAnim")
_mock_oma.MFnAnimCurve = type("MFnAnimCurve", (), {})
_mock_oma.MFnIkJoint = type("MFnIkJoint", (), {})

_mock_cmds = type(sys)("maya.cmds")

sys.modules.setdefault("maya", type(sys)("maya"))
sys.modules.setdefault("maya.api", type(sys)("maya.api"))
sys.modules["maya.api.OpenMaya"] = _mock_om
sys.modules["maya.api.OpenMayaAnim"] = _mock_oma
sys.modules["maya.cmds"] = _mock_cmds

from mmd.maya.vmd_scene_builder import (  # noqa: E402
    _convert_vmd_quaternion_to_maya_components,
    _ensure_quaternion_continuity,
    _normalize_quaternion_components,
)


def _quat_alignment(lhs, rhs):
    lx, ly, lz, lw = _normalize_quaternion_components(lhs)
    rx, ry, rz, rw = _normalize_quaternion_components(rhs)
    return abs(lx * rx + ly * ry + lz * rz + lw * rw)


class TestVmdSceneBuilderRotationMath(unittest.TestCase):
    def assertQuaternionAlmostEqual(self, lhs, rhs, places=6):
        self.assertAlmostEqual(_quat_alignment(lhs, rhs), 1.0, places=places)

    def test_vmd_quaternion_conversion_reflects_maya_basis(self):
        converted = _convert_vmd_quaternion_to_maya_components((0.1, -0.2, 0.3, 0.9))
        self.assertQuaternionAlmostEqual(converted, (-0.1, 0.2, 0.3, 0.9))


class TestEnsureQuaternionContinuity(unittest.TestCase):
    """Tests for _ensure_quaternion_continuity pure function."""

    def test_single_quat_returns_as_is(self):
        result = _ensure_quaternion_continuity([(0.1, 0.2, 0.3, 0.9)])
        self.assertEqual(result, [(0.1, 0.2, 0.3, 0.9)])

    def test_empty_list_returns_empty(self):
        result = _ensure_quaternion_continuity([])
        self.assertEqual(result, [])

    def test_same_hemisphere_no_change(self):
        q0 = (0.1, 0.2, 0.3, 0.9)
        q1 = (0.2, 0.3, 0.4, 0.8)
        result = _ensure_quaternion_continuity([q0, q1])
        self.assertEqual(result, [q0, q1])

    def test_opposite_hemisphere_negates(self):
        q0 = (0.1, 0.2, 0.3, 0.9)
        # q1 has negative dot product with q0 → should be negated
        q1 = (-0.1, -0.2, -0.3, -0.9)
        result = _ensure_quaternion_continuity([q0, q1])
        # q1 should be negated to (0.1, 0.2, 0.3, 0.9) to match q0's hemisphere
        self.assertEqual(result, [q0, (0.1, 0.2, 0.3, 0.9)])

    def test_multiple_flips_chain(self):
        q0 = (1.0, 0.0, 0.0, 0.0)
        q1 = (-0.9, 0.0, 0.0, 0.4)  # dot < 0 → negate
        q2 = (0.8, 0.0, 0.0, 0.6)  # dot with negated q1 > 0 → keep
        result = _ensure_quaternion_continuity([q0, q1, q2])
        expected_q1 = (0.9, 0.0, 0.0, -0.4)  # negated
        self.assertEqual(result, [q0, expected_q1, q2])

    def test_identity_rotations(self):
        q_identity = (0.0, 0.0, 0.0, 1.0)
        q_near_identity = (0.0, 0.0, 0.0, 0.9999)
        result = _ensure_quaternion_continuity([q_identity, q_near_identity])
        self.assertEqual(result, [q_identity, q_near_identity])


class TestNormalizeQuaternionComponents(unittest.TestCase):
    """Tests for _normalize_quaternion_components pure function."""

    def test_already_normalized(self):
        result = _normalize_quaternion_components((0.0, 0.0, 0.0, 1.0))
        self.assertEqual(result, (0.0, 0.0, 0.0, 1.0))

    def test_needs_normalization(self):
        result = _normalize_quaternion_components((2.0, 0.0, 0.0, 0.0))
        # magnitude = 2.0, so each component divided by 2
        self.assertEqual(result, (1.0, 0.0, 0.0, 0.0))

    def test_zero_vector_returns_identity(self):
        result = _normalize_quaternion_components((0.0, 0.0, 0.0, 0.0))
        self.assertEqual(result, (0.0, 0.0, 0.0, 1.0))

    def test_negative_magnitude_components(self):
        result = _normalize_quaternion_components((-1.0, 0.0, 0.0, 0.0))
        self.assertEqual(result, (-1.0, 0.0, 0.0, 0.0))


def _quat_dot(lhs, rhs):
    """Compute dot product of two quaternions (components only)."""
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2] + lhs[3] * rhs[3]


class TestConvertVmdQuaternionToMayaComponents(unittest.TestCase):
    """Tests for _convert_vmd_quaternion_to_maya_components pure function."""

    def test_identity_stays_identity(self):
        result = _convert_vmd_quaternion_to_maya_components((0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(_quat_dot(result, (0.0, 0.0, 0.0, 1.0)), 1.0, places=6)

    def test_x_axis_rotation(self):
        # Rotation around X in MMD: (sin(θ/2), 0, 0, cos(θ/2))
        # After conversion: X and Y negated, Z flipped sign
        mmd_quat = (0.3826834, 0.0, 0.0, 0.9238795)  # 45° around X
        result = _convert_vmd_quaternion_to_maya_components(mmd_quat)
        # Maya: (-x, -y, z, w) → (-0.3826834, 0.0, 0.0, 0.9238795)
        expected = (-0.3826834, 0.0, 0.0, 0.9238795)
        self.assertAlmostEqual(_quat_dot(result, expected), 1.0, places=4)

    def test_y_axis_rotation(self):
        mmd_quat = (0.0, 0.3826834, 0.0, 0.9238795)  # 45° around Y
        result = _convert_vmd_quaternion_to_maya_components(mmd_quat)
        expected = (0.0, -0.3826834, 0.0, 0.9238795)
        self.assertAlmostEqual(_quat_dot(result, expected), 1.0, places=4)

    def test_z_axis_rotation(self):
        mmd_quat = (0.0, 0.0, 0.3826834, 0.9238795)  # 45° around Z
        result = _convert_vmd_quaternion_to_maya_components(mmd_quat)
        # Z is kept positive
        expected = (0.0, 0.0, 0.3826834, 0.9238795)
        self.assertAlmostEqual(_quat_dot(result, expected), 1.0, places=4)


if __name__ == "__main__":
    unittest.main()
