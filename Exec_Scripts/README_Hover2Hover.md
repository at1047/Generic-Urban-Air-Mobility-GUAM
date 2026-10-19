# Accelerate, forward flight, decelerate

Run `RUNME` from the GUAM repository root and select option **6: Hover to Hover
(Accelerate/Forward Flight/Decelerate)**. The example sets up GUAM, and `RUNME`
simulates and generates the standard plots. There is no optimization.

The fixed settings in `Exec_Scripts/exam_Hover2Hover.m` describe a level,
northbound transfer from `[0 0 0]` to `[12000 0 0]` ft:

| Phase | Time | Distance | Acceleration | Speed |
| --- | --- | --- | --- | --- |
| Accelerate | 0-50 s | 3,750 ft | +3 ft/s^2 | 0 to 150 ft/s |
| Forward flight | 50-80 s | 4,500 ft | 0 | 150 ft/s |
| Decelerate | 80-130 s | 3,750 ft | -3 ft/s^2 | 150 to 0 ft/s |
| Hover hold | 130-135 s | 0 | 0 | 0 |

Velocity increases and decreases linearly; acceleration is constant in each
phase. The resulting speed graph is a trapezoid. Position is quadratic during
acceleration/deceleration and linear during constant-speed flight. Acceleration
jumps at the phase boundaries, so the reference does not impose a jerk limit.
The speed of 150 ft/s is about 89 knots. Wing-borne flight and tracking depend
on GUAM's controller and aircraft dynamics; the planner does not select a flight
mode or verify the aircraft's physical limits.

## Generation

The inputs to `planHoverTransfer` are start position, end position, cruise speed,
and acceleration magnitude. Positions use `[north east down]` in feet, speed is
in ft/s, and acceleration is in ft/s^2. Deceleration uses the same magnitude.
For distance `D`, cruise speed `V`, and acceleration magnitude `A`:

- Acceleration time: `V/A`.
- Acceleration distance: `V^2/(2*A)`; deceleration uses the same distance/time.
- Cruise distance: `D - V^2/A`.
- Cruise time: `cruiseDistance/V`.
- Total transfer time: `2*accelerationTime + cruiseTime`.

The route must be longer than `V^2/A` to include all three phases. The function
raises an error for shorter routes or identical endpoints, rather than changing
the requested speed or acceleration.

Each phase uses elementary kinematics along the straight-line direction. During
acceleration, `p = startPosition + 0.5*a*t^2` and `v = a*t`. Constant-speed flight
starts at the acceleration endpoint. During deceleration, remaining time `r`
gives `p = endPosition - 0.5*a*r^2` and `v = a*r`. Positions and velocities match
at every boundary.

To generate the reference without running GUAM:

```matlab
addpath('Exec_Scripts');
plan = planHoverTransfer([0 0 0], [12000 0 0], 150, 3, ...
    'SampleTime', 0.05, 'HoldTime', 5);
```

`plan.time`, `position`, `velocity`, and `acceleration` include the transfer and
final hold. The grid includes every phase boundary, and sample spacing is at
most `SampleTime`. `duration` excludes the hold, while `stopTime` includes it.
The plan also reports `distance`, `peakSpeed`, `peakAcceleration`, and the time
and distance for each phase (`accelerationTime`, `cruiseTime`, `decelerationTime`,
`accelerationDistance`, `cruiseDistance`, and `decelerationDistance`). `settings`
stores sampling and hold options.

`plan.RefInput` packages position, heading-frame velocity, heading, and heading
rate into GUAM timeseries. Assign it to `target.RefInput`, choose
`userStruct.variants.refInputType = 3`, and call `simSetup`. Heading is constant
along the horizontal travel direction, including at hover. Vertical transfers
use zero heading. GUAM interpolates the samples. Acceleration is provided in
`plan` for inspection, not sent to the controller.

The earlier quintic optimizer remains available as `planMinimumTimeHover.m` for
comparison; option 6 uses only this three-phase generator.
