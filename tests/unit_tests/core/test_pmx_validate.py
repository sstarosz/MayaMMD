# pylint: disable=missing-function-docstring,missing-module-docstring
from mmd.core.data_types import PmxHeader, PMXJoint, PmxModel, PMXRigidBody, Vec3
from mmd.core.pmx_validate import IssueCategory, IssueSeverity, validate_pmx_model


def _vec(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Vec3:
    return Vec3(x, y, z)


def _body(x: float, y: float, z: float = 0.0) -> PMXRigidBody:
    return PMXRigidBody(shape_position=_vec(x, y, z))


def _joint(a: int, b: int, position: Vec3, **kwargs) -> PMXJoint:
    return PMXJoint(
        name_local="joint",
        rigid_body_index_a=a,
        rigid_body_index_b=b,
        position=position,
        **kwargs,
    )


def _inert(joint: PMXJoint) -> PMXJoint:
    """Limits like a frame left at the origin by a physics converter."""
    joint.position_min = _vec(-50.0, -50.0, -50.0)
    joint.position_max = _vec(50.0, 50.0, 50.0)
    # Bullet reads an inverted angular range as free rotation.
    joint.rotation_min = _vec(0.0175, 0.0175, 0.0175)
    joint.rotation_max = _vec(0.0, 0.0, 0.0)
    return joint


def _model(
    joints: list[PMXJoint], bodies: list[PMXRigidBody] | None = None
) -> PmxModel:
    return PmxModel(
        model_name="test",
        file_path="test.pmx",
        absolute_path=".",
        header=PmxHeader(),
        rigid_bodies=bodies if bodies is not None else [_body(0, 0), _body(0, 10)],
        joints=joints,
    )


def _physics_issues(model: PmxModel):
    result = validate_pmx_model(model)
    return result.get_by_category(IssueCategory.PHYSICS)


def test_healthy_joint_reports_nothing():
    # Frame on the pivot inside body A, both axes locked.
    issues = _physics_issues(_model([_joint(0, 1, _vec(0, 0))]))
    assert issues == []


def test_detached_inert_joint_is_reported_once():
    joint = _inert(_joint(0, 1, _vec(5, 5)))
    issues = _physics_issues(_model([joint]))

    assert len(issues) == 1
    assert issues[0].severity is IssueSeverity.WARNING
    assert "placed far away from the bodies they connect" in issues[0].message
    assert "do not restrict them" in issues[0].message
    assert issues[0].affected_items == [f"Joint 0 '{joint.name_local}'"]
    assert "7.1 units" in issues[0].details  # |(5, 5) - (0, 0)|
    assert "10.0 units across" in issues[0].details  # body bounding-box diagonal
    assert "world origin" not in issues[0].details  # not this joint's problem


def test_detached_joint_at_the_origin_says_so():
    model = _model(
        [_inert(_joint(0, 1, _vec(0, 0)))],
        bodies=[_body(10, 0), _body(10, 10)],
    )
    issues = _physics_issues(model)

    assert len(issues) == 1
    assert "All of them sit at or near the world origin" in issues[0].details


def test_detached_joint_far_from_the_origin_does_not_blame_the_origin():
    issues = _physics_issues(_model([_inert(_joint(0, 1, _vec(20, 5)))]))

    assert len(issues) == 1
    assert "world origin" not in issues[0].details


def test_detached_joint_with_limits_is_reported_as_offset_pivot():
    joint = _joint(0, 1, _vec(5, 5))  # limits default to 0/0: fully locked
    issues = _physics_issues(_model([joint]))

    assert len(issues) == 1
    assert "placed far away from the bodies they connect" in issues[0].message
    assert "7.1 units" in issues[0].details
    assert "swing a long way" in issues[0].details


def test_joint_on_its_body_that_constrains_nothing_is_reported():
    issues = _physics_issues(_model([_inert(_joint(0, 1, _vec(0, 0)))]))

    assert len(issues) == 1
    assert "do not restrict their bodies" in issues[0].message


def test_spring_on_a_free_axis_still_constrains():
    joint = _inert(_joint(0, 1, _vec(0, 0)))
    joint.position_spring_constant = _vec(12.0, 0.0, 0.0)

    assert _physics_issues(_model([joint])) == []


def test_joint_with_a_missing_body_is_an_error():
    issues = _physics_issues(_model([_inert(_joint(0, 7, _vec(0, 0)))]))

    assert len(issues) == 1
    assert issues[0].severity is IssueSeverity.ERROR
    assert "rigid body that does not exist" in issues[0].message
    # A broken reference is not additionally blamed on its position.
    assert issues[0].affected_items == ["Joint 0 'joint'"]


def test_model_without_joints_reports_no_physics_issues():
    assert validate_pmx_model(_model([])).get_by_category(IssueCategory.PHYSICS) == []
