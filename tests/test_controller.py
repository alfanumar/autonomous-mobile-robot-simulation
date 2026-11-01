import math

from mobile_robot_sim import Pose, WaypointController


def test_controller_drives_forward_to_waypoint() -> None:
    controller = WaypointController(wheel_base=0.3)
    output = controller.compute(Pose(), (1.0, 0.0))
    assert output.command.left_mps > 0.0
    assert output.command.right_mps > 0.0
    assert math.isclose(output.heading_error, 0.0, abs_tol=1e-9)


def test_controller_turns_toward_side_waypoint() -> None:
    controller = WaypointController(wheel_base=0.3)
    output = controller.compute(Pose(), (0.0, 1.0))
    assert output.command.right_mps > output.command.left_mps
    assert output.heading_error > 0.0


def test_controller_stops_inside_tolerance() -> None:
    controller = WaypointController(waypoint_tolerance=0.1)
    output = controller.compute(Pose(x=0.05), (0.0, 0.0))
    assert output.command.left_mps == 0.0
    assert output.command.right_mps == 0.0
