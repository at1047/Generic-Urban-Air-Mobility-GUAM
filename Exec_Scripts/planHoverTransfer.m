function plan = planHoverTransfer(startPosition, endPosition, cruiseSpeed, accelerationRate, options)
%PLANHOVERTRANSFER Accelerate, fly at constant speed, then decelerate to hover.
% Positions use [north east down] in ft, speed ft/s, and acceleration ft/s^2.
arguments
    startPosition (1,3) double {mustBeFinite}
    endPosition (1,3) double {mustBeFinite}
    cruiseSpeed (1,1) double {mustBeFinite, mustBePositive}
    accelerationRate (1,1) double {mustBeFinite, mustBePositive}
    options.SampleTime (1,1) double {mustBeFinite, mustBePositive} = 0.05
    options.HoldTime (1,1) double {mustBeFinite, mustBePositive} = 5
end

displacement = endPosition - startPosition;
distance = norm(displacement);
accelerationTime = cruiseSpeed/accelerationRate;
accelerationDistance = 0.5*accelerationRate*accelerationTime^2;
cruiseDistance = distance-2*accelerationDistance;
if cruiseDistance <= 0
    error('planHoverTransfer:InsufficientDistance', ...
        'Route distance must exceed %.2f ft to reach %.2f ft/s and include a constant-speed phase.', ...
        2*accelerationDistance, cruiseSpeed);
end
direction = displacement/distance;
accelerationVector = accelerationRate*direction;
cruiseVelocity = cruiseSpeed*direction;
cruiseTime = cruiseDistance/cruiseSpeed;
decelerationStart = accelerationTime+cruiseTime;
duration = decelerationStart+accelerationTime;
transferSamples = max(2, ceil(duration/options.SampleTime)+1);
transferTime = linspace(0, duration, transferSamples)';
holdSamples = max(1, ceil(options.HoldTime/options.SampleTime));
holdTime = duration + (1:holdSamples)'*(options.HoldTime/holdSamples);
time = unique([transferTime; accelerationTime; decelerationStart; holdTime]);

position = zeros(length(time),3);
velocity = zeros(length(time),3);
acceleration = zeros(length(time),3);

for sample = 1:length(time)
    currentTime = time(sample);
    if currentTime < accelerationTime
        position(sample,:) = startPosition + 0.5*accelerationVector*currentTime^2;
        velocity(sample,:) = accelerationVector*currentTime;
        acceleration(sample,:) = accelerationVector;
    elseif currentTime < decelerationStart
        position(sample,:) = startPosition + direction*accelerationDistance ...
            + cruiseVelocity*(currentTime-accelerationTime);
        velocity(sample,:) = cruiseVelocity;
    elseif currentTime < duration
        remainingTime = duration-currentTime;
        position(sample,:) = endPosition - 0.5*accelerationVector*remainingTime^2;
        velocity(sample,:) = accelerationVector*remainingTime;
        acceleration(sample,:) = -accelerationVector;
    else
        position(sample,:) = endPosition;
    end
end

heading = atan2(displacement(2), displacement(1));
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
plan.duration = duration;
plan.stopTime = time(end);
plan.distance = distance;
plan.peakSpeed = cruiseSpeed;
plan.peakAcceleration = accelerationRate;
plan.accelerationTime = accelerationTime;
plan.cruiseTime = cruiseTime;
plan.decelerationTime = accelerationTime;
plan.accelerationDistance = accelerationDistance;
plan.cruiseDistance = cruiseDistance;
plan.decelerationDistance = accelerationDistance;
plan.settings = options;
end
