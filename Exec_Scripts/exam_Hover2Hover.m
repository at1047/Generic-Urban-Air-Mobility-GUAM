%% sim parameters
model = 'GUAM';
userStruct.variants.refInputType = 3;
% Override with userStruct.enablePlots = true/false before running this script.
% Batch/headless runs skip figures and the Simulink editor by default.
if ~isfield(userStruct, 'enablePlots')
    userStruct.enablePlots = usejava('desktop') && ~batchStartupOptionUsed;
end

%% Choose hover positions in ft, using [north east down].
startPosition = [0 0 -100];
endPosition = [20000 0 -100];

cruiseSpeed = 180;
% 3 ft/s^2 excites a growing pitch oscillation after rotor shutdown near
% 95 knots with the default baseline controller. 2 ft/s^2 completes both
% transitions and retains a sustained rotors-off, pusher-on cruise segment.
accelerationRate = 2;
plan = planHoverTransfer(startPosition, endPosition, cruiseSpeed, accelerationRate, ...
    'SampleTime', 0.05, 'HoldTime', 40);

fprintf('Transfer: %.2f ft in %.2f s\n', plan.distance, plan.duration);
fprintf('Peak speed: %.2f ft/s; acceleration: %.2f ft/s^2\n', ...
    plan.peakSpeed, plan.peakAcceleration);
fprintf('Accelerate: %.2f s / %.2f ft; forward flight: %.2f s / %.2f ft; decelerate: %.2f s / %.2f ft\n', ...
    plan.accelerationTime, plan.accelerationDistance, ...
    plan.cruiseTime, plan.cruiseDistance, ...
    plan.decelerationTime, plan.decelerationDistance);

if userStruct.enablePlots
    figure('Name', 'Hover transfer: accelerate, forward flight, decelerate');
    subplot(2,2,1);
    plot3(plan.position(:,1), plan.position(:,2), -plan.position(:,3));
    xlabel('North (ft)'); ylabel('East (ft)'); zlabel('Up (ft)');
    grid on; axis equal;
    subplot(2,2,2);
    plot(plan.time, sqrt(sum(plan.velocity.^2,2)));
    xlabel('Time (s)'); ylabel('Speed (ft/s)'); grid on;
    subplot(2,2,3);
    plot(plan.time, plan.position);
    xlabel('Time (s)'); ylabel('Position (ft)');
    legend('North', 'East', 'Down'); grid on;
    subplot(2,2,4);
    stairs(plan.time, plan.acceleration);
    xlabel('Time (s)'); ylabel('Acceleration (ft/s^2)');
    legend('North', 'East', 'Down'); grid on;
end

%% Prepare to run simulation
clear target
target.RefInput = plan.RefInput;
simSetup;
if userStruct.enablePlots
    open_system(model);
else
    load_system(model);
end
