import math

from mobile_robot_sim import DifferentialDrive, Odometry, Pose, WheelCommand


def test_straight_motion() -> None:
    robot = DifferentialDrive(wheel_base=0.3, max_wheel_speed=1.0)
    pose, left_distance, right_distance = robot.step(WheelCommand(0.5, 0.5), 2.0)
    assert math.isclose(pose.x, 1.0, abs_tol=1e-9)
    assert math.isclose(pose.y, 0.0, abs_tol=1e-9)
    assert math.isclose(pose.theta, 0.0, abs_tol=1e-9)
    assert math.isclose(left_distance, 1.0, abs_tol=1e-9)
    assert math.isclose(right_distance, 1.0, abs_tol=1e-9)


def test_in_place_rotation() -> None:
    robot = DifferentialDrive(wheel_base=0.4, max_wheel_speed=1.0)
    pose, _, _ = robot.step(WheelCommand(-0.2, 0.2), 1.0)
    assert math.isclose(pose.x, 0.0, abs_tol=1e-9)
    assert math.isclose(pose.y, 0.0, abs_tol=1e-9)
    assert math.isclose(pose.theta, 1.0, abs_tol=1e-9)


def test_odometry_quantizes_wheel_distance() -> None:
    odometry = Odometry(wheel_base=0.3, wheel_radius=0.05, ticks_per_revolution=100)
    tick = odometry.tick_distance
    estimate = odometry.update(1.4 * tick, 1.4 * tick)
    assert math.isclose(estimate.x, 1.0 * tick, abs_tol=1e-9)
    assert math.isclose(estimate.y, 0.0, abs_tol=1e-9)


def test_pose_defaults_to_origin() -> None:
    pose = Pose()
    assert (pose.x, pose.y, pose.theta) == (0.0, 0.0, 0.0)
