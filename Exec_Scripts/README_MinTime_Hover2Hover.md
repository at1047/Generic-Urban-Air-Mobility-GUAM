# Minimum-time hover-to-hover example

From the GUAM repository root, execute `RUNME` in MATLAB with Simulink and the
usual GUAM dependencies, and select option **6: Minimum-Time Hover to Hover**.
The example plans a transfer, plots its reference trajectory and motion limits,
and calls `simSetup` and `open(model)`, matching the other example scripts.
`RUNME` then runs GUAM and generates the standard simulation plots. The reference
and planner diagnostics remain in `plan`.

Keep MATLAB's current folder at the repository root. `RUNME` adds `Exec_Scripts`
to the path and invokes the example by name, avoiding the folder change caused
by `run('Exec_Scripts/exam_MinTime_Hover2Hover.m')`.

Edit `startPosition`, `endPosition`, and the limits at the top of the script.
Positions are `[north east down]` in feet: negative down means up. Limits apply
to the magnitude of the three-dimensional velocity, acceleration, and jerk.
The example limits are illustrative planning settings, not validated GUAM flight
envelope limits. Inspect tracking and actuator behavior after simulation.

## Planner

`planMinimumTimeHover` also works independently of simulation:

```matlab
addpath('Exec_Scripts');
plan = planMinimumTimeHover([0 0 0], [200 100 -50], ...
    'MaxSpeed', 20, 'MaxAcceleration', 4, 'MaxJerk', 2);
```

The planner minimizes transfer duration `T` within the straight-line quintic
family `p(t) = p0 + (pf-p0)*(10*s^3 - 15*s^4 + 6*s^5)`, with `s=t/T`.
Both endpoints have zero velocity and acceleration. For distance `D`, the exact
continuous-time peak magnitudes are:

- Speed: `15*D/(8*T)`.
- Acceleration: `10*D/(sqrt(3)*T^2)`.
- Jerk: `60*D/T^3`.

Each limit gives a lower bound on `T`. Their maximum is the exact minimum feasible
duration in this curve family, so no numerical optimizer or Optimization Toolbox
is needed. The limits are enforced analytically, not just at sampled times.
`plan.limitingConstraint` identifies an active limit. This is not the unrestricted
minimum-time trajectory or an optimization over GUAM's aircraft dynamics.

`SampleTime` (default 0.05 s) is the maximum reference sample spacing. `HoldTime`
(default 5 s, positive) adds a final stationary reference after the optimized
transfer; it is excluded from `plan.duration` but included in `plan.stopTime`.
Identical endpoints produce only a stationary hold, with zero transfer duration.

The output `plan.RefInput` can be assigned directly to `target.RefInput` with
`userStruct.variants.refInputType = 3` before calling `simSetup`. Velocity is
converted from NED into GUAM's heading frame. Heading is constant along the
transfer and during the hover endpoints, avoiding undefined track at zero speed.
Purely vertical transfers use zero heading. GUAM interpolates the sampled
reference; use a smaller `SampleTime` if necessary for tracking accuracy.

This first planner has no obstacles, wind, turn planning, attitude constraints,
power model, or separate vertical motion limits. Its acceleration is kinematic
acceleration, not thrust or specific force. Jerk may jump at the transfer/hold
boundary; position, velocity, and acceleration remain continuous.
