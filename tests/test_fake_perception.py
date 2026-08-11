from domain.adapters.fake.fake_perception import FakePerception
from domain.values import Point3, Pose2D


def test_detect_target_returns_domain_value_types():
    found, pose, dims = FakePerception().detect_target()

    assert found is True
    assert isinstance(pose, Point3)
    assert isinstance(dims, Point3)


def test_measure_gap_centerline_returns_pose2d():
    gap = FakePerception().measure_gap()

    assert isinstance(gap.centerline, Pose2D)


def test_detect_target_returns_none_when_not_found():
    found, pose, dims = FakePerception(found=False).detect_target()

    assert found is False
    assert pose is None
    assert dims is None
