# Validation Results

## Scenario

- Rectangular waypoint route: (1,0) -> (1,1) -> (0,1) -> (0,0) m.
- Differential-drive wheel base: 0.30 m.
- Encoder model: 2048 ticks/revolution and 0.05 m wheel radius.
- Simulation time step: 0.02 s.
- Feedback pose: quantized wheel odometry.

## Validation

The committed CSV and plot are generated from the same simulation command used
in the test workflow.

~~~text
Route completion: all 4 waypoints reached
Maximum ground-truth vs odometry position error: 0.0235 m
Final distance-to-waypoint error: 0.0788 m
~~~

These are software-in-the-loop results. They are not measurements from a
physical mobile robot, encoder, or motor driver.
