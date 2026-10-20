# Hover to hover with a fixed-wing segment

Run `RUNME` from the repository root and select **6**. The example generates
a straight, level reference, initializes GUAM, simulates it, saves `SimOut`,
and reports actual rotors-off flight with `analyzeHoverTransfer`.
There is no optimization or explicit flight-mode command. The reference
provides position, velocity, heading, and heading rate; GUAM's baseline
controller schedules its trim, gains, and effector allocation with velocity.

## Current trajectory

`exam_Hover2Hover.m` transfers from `[0 0 -100]` to `[20000 0 -100]` ft in
local `[north east down]` coordinates. Cruise speed is **180 ft/s** (about
**107 knots**); acceleration and deceleration are **2 ft/s²**.

| Phase | Time (s) | Distance (ft) | Speed (ft/s) |
| --- | --- | --- | --- |
| Accelerate | 0–90 | 8,100 | 0–180 |
| Constant speed | 90–111.111 | 3,800 | 180 |
| Decelerate | 111.111–201.111 | 8,100 | 180–0 |
| Hover hold | 201.111–241.111 | 0 | 0 |

The longer final hold lets the aircraft settle after deceleration. Position
and velocity are continuous; acceleration jumps at the phase boundaries.
The reference imposes no jerk limit and sends no acceleration feedforward
to the controller.

## Why 3 ft/s² failed at 73.23 s

The original run already reached fixed-wing flight: all eight lift rotors
stopped at approximately **53.45 s**, while the pusher continued running.
A growing pitch oscillation then caused large flight-path and attitude
errors. At 73.23 s the **body-axis forward airspeed** fell below 49 knots
with the lift rotors still stopped, triggering `KillifNotValidPropSpd`.
That value is a component of airspeed, not the total airspeed magnitude.

The assertion is an aerodynamic-model validity check, not simply a motor
overspeed check. In `LpC_interp_p_v2.m`, the cruise model requires forward
speed between `50*0.98` and `130*1.02` knots, lift-rotor speeds at most 1 RPM,
and pusher speed at least 550 RPM. With no matching model branch, the
validity flags remain invalid. Angle of attack and other factors also have
limited fitted ranges; disabling the assertion does not repair the earlier
loss of control.

The default trim table has a sharp rotor-assisted to rotors-off change
between approximately **160.004 and 160.342 ft/s** (near 95 knots).
Reducing acceleration to **2 ft/s²** gives this example a successful
transition in both directions while preserving the route and cruise speed.
Changing cruise speed alone to 165 or 170 ft/s at 3 ft/s² still failed.

In the verified 2 ft/s² run, all lift rotors remain at or below 1 RPM and the
pusher remains above 550 RPM from approximately **80.175 to 120.940 s**:
about **40.8 s of fixed-wing flight**, spanning cruise and parts of both ramps.
Fixed-wing angle of attack ranges from approximately **4.1 to 11.0 degrees**.
Local height ranges from approximately **66.6 to 132.3 ft** against a 100 ft
reference. The trajectory completes, but altitude tracking has a noticeable
transition transient. These settings are an empirically validated example
for the default model/controller, not a general aircraft feasibility check.
After the 40 s final hold, the verified position error is approximately
**0.12 ft** and speed is **0.011 ft/s**. The full headless run completes
at 241.11 s with the model validity assertions enabled and no figures.

`analyzeHoverTransfer(SimOut, plan)` reports measured fixed-wing intervals,
height and angle-of-attack ranges, final position error, and final speed.
It uses actual rotor speeds rather than assuming constant-speed flight means
the rotors are off.

## Headless debugging

`exam_Hover2Hover` skips figures and the Simulink editor in batch mode or
without the MATLAB desktop. Override with `userStruct.enablePlots = false`
or `true`; `RUNME` also honors this setting for its final plots.

From the repository root on the Arch host:

```sh
distrobox enter matlab -- env MATLAB_SHELL=/bin/bash \
  /home/andrew/.local/MATLAB/R2025b/bin/matlab \
  -batch "addpath('Exec_Scripts'); exam_Hover2Hover; out = sim(model, 'CaptureErrors', 'on'); disp(out.ErrorMessage); SimOut = out.logsout{1}.Values; summary = analyzeHoverTransfer(SimOut, plan); save('/tmp/hover_result.mat', 'SimOut', 'plan', 'summary', '-v7.3'); assert(isempty(out.ErrorMessage), out.ErrorMessage);" \
  -logfile /tmp/hover_result.log
```

`CaptureErrors` retains partial logs on runtime failures, so the analysis can
show the state at the assertion. Setup/compile failures can occur before logs
exist; in that case inspect `out.ErrorMessage` first. The final assertion gives
a nonzero batch exit status on simulation failure. Paths under `/tmp` above
are inside the container. `MATLAB_SHELL=/bin/bash` fixes the Perl lookup
failure encountered in `restoredefaultpath` with the inherited shell environment.

Running `exam_Hover2Hover` alone prepares the model; `sim(model)` executes it.
Run from the repository root because `simSetup` uses relative paths.

## Generator and GUAM interface

`planHoverTransfer(startPosition, endPosition, cruiseSpeed, accelerationRate)`
uses elementary straight-line kinematics. For distance `D`, cruise speed
`V`, and acceleration magnitude `A`:

- Acceleration/deceleration time each: `V/A`.
- Acceleration/deceleration distance each: `V^2/(2*A)`.
- Cruise distance: `D - V^2/A`; cruise time: `cruiseDistance/V`.
- Transfer duration: `2*V/A + cruiseTime`, excluding the final hold.

The route must exceed `V^2/A` to include a constant-speed segment; shorter
routes raise an error. Supply a positive `HoldTime` and `SampleTime`:

```matlab
addpath('Exec_Scripts');
plan = planHoverTransfer([0 0 -100], [20000 0 -100], 180, 2, ...
    'SampleTime', 0.05, 'HoldTime', 40);
```

`plan.time`, `position`, `velocity`, and `acceleration` include the final hold.
The grid includes phase boundaries and spacing is at most `SampleTime`.
`duration` excludes the hold; `stopTime` includes it. The plan also reports
phase times/distances, peak speed/acceleration, and sampling/hold settings.

`plan.RefInput` packages position, heading-frame velocity, heading, and heading
rate as GUAM timeseries. Assign it to `target.RefInput`, choose
`userStruct.variants.refInputType = 3`, and call `simSetup`. Heading is constant
along the horizontal route, including at hover; vertical routes use zero
heading. GUAM interpolates the samples.

The earlier quintic optimizer remains in `planMinimumTimeHover.m` for comparison;
option 6 uses the simple three-phase generator.
