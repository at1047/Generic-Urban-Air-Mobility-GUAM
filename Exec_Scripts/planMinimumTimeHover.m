function plan = planMinimumTimeHover(startPosition, endPosition, options)
%PLANMINIMUMTIMEHOVER Minimum-duration straight-line quintic hover transfer.
% Positions use [north east down] in ft. Limits are Euclidean magnitudes
% in ft/s, ft/s^2, and ft/s^3. Endpoint velocity and acceleration are zero.
% The optimum is restricted to p(t) = p0 + (pf-p0)*(10*s^3-15*s^4+6*s^5),
% where s = t/T. It is not a minimum-time solution of aircraft dynamics.
% No Optimization Toolbox is required.
arguments
    startPosition (1,3) double {mustBeFinite}
    endPosition (1,3) double {mustBeFinite}
    options.MaxSpeed (1,1) double {mustBeFinite, mustBePositive} = 20
    options.MaxAcceleration (1,1) double {mustBeFinite, mustBePositive} = 4
    options.MaxJerk (1,1) double {mustBeFinite, mustBePositive} = 2
    options.SampleTime (1,1) double {mustBeFinite, mustBePositive} = 0.05
    options.HoldTime (1,1) double {mustBeFinite, mustBePositive} = 5
end

displacement = endPosition - startPosition;
distance = norm(displacement);
durationBounds = [15*distance/(8*options.MaxSpeed), ...
    sqrt(10*distance/(sqrt(3)*options.MaxAcceleration)), ...
    (60*distance/options.MaxJerk)^(1/3)];
[duration, activeLimit] = max(durationBounds);
limitNames = {'speed', 'acceleration', 'jerk'};

if distance == 0
    time = 0;
    position = startPosition;
    velocity = zeros(1,3);
    acceleration = zeros(1,3);
    jerk = zeros(1,3);
    heading = 0;
    peakValues = zeros(1,3);
    limitingConstraint = 'none';
else
    time = linspace(0, duration, max(2, ceil(duration/options.SampleTime)+1))';
    phase = time/duration;
    blend = 10*phase.^3 - 15*phase.^4 + 6*phase.^5;
    blendRate = 30*phase.^2.*(1-phase).^2/duration;
    blendAcceleration = 60*phase.*(1-phase).*(1-2*phase)/duration^2;
    blendJerk = (60-360*phase+360*phase.^2)/duration^3;
    position = startPosition + blend*displacement;
    velocity = blendRate*displacement;
    acceleration = blendAcceleration*displacement;
    jerk = blendJerk*displacement;
    position(end,:) = endPosition;
    heading = atan2(displacement(2), displacement(1));
    peakValues = [15*distance/(8*duration), ...
        10*distance/(sqrt(3)*duration^2), 60*distance/duration^3];
    limitingConstraint = limitNames{activeLimit};
end

holdSamples = max(1, ceil(options.HoldTime/options.SampleTime));
holdTime = duration + (1:holdSamples)'*(options.HoldTime/holdSamples);
time = [time; holdTime];
position = [position; repmat(endPosition, holdSamples, 1)];
velocity = [velocity; zeros(holdSamples,3)];
acceleration = [acceleration; zeros(holdSamples,3)];
jerk = [jerk; zeros(holdSamples,3)];
headingVelocity = [cos(heading)*velocity(:,1)+sin(heading)*velocity(:,2), ...
    -sin(heading)*velocity(:,1)+cos(heading)*velocity(:,2), velocity(:,3)];

plan.RefInput.pos_des = timeseries(position, time);
plan.RefInput.Vel_bIc_des = timeseries(headingVelocity, time);
plan.RefInput.vel_des = timeseries(velocity, time);
plan.RefInput.chi_des = timeseries(repmat(heading, length(time), 1), time);
plan.RefInput.chi_dot_des = timeseries(zeros(length(time),1), time);
plan.time = time;
plan.position = position;
plan.velocity = velocity;
plan.acceleration = acceleration;
plan.jerk = jerk;
plan.duration = duration;
plan.stopTime = time(end);
plan.distance = distance;
plan.peakSpeed = peakValues(1);
plan.peakAcceleration = peakValues(2);
plan.peakJerk = peakValues(3);
plan.limitingConstraint = limitingConstraint;
plan.limits = options;
end
