from simulate import run_simulation


def test_route_reaches_all_waypoints() -> None:
    rows = run_simulation(duration=24.0, dt=0.02)
    assert rows
    assert int(rows[-1]["waypoint_index"]) == 4
    assert rows[-1]["distance_error_m"] < 0.1


def test_odometry_stays_close_to_ground_truth() -> None:
    rows = run_simulation(duration=24.0, dt=0.02)
    max_error = max(
        ((row["ground_truth_x_m"] - row["odometry_x_m"]) ** 2
         + (row["ground_truth_y_m"] - row["odometry_y_m"]) ** 2) ** 0.5
        for row in rows
    )
    assert max_error < 0.05
