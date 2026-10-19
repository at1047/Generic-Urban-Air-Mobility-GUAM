%% sim parameters
model = 'GUAM';
userStruct.variants.refInputType = 3;

%% Choose hover positions in ft, using [north east down].
startPosition = [0 0 0];
endPosition = [12000 0 0];

cruiseSpeed = 150;
accelerationRate = 3;
plan = planHoverTransfer(startPosition, endPosition, cruiseSpeed, accelerationRate, ...
    'SampleTime', 0.05, 'HoldTime', 5);

fprintf('Transfer: %.2f ft in %.2f s\n', plan.distance, plan.duration);
fprintf('Peak speed: %.2f ft/s; acceleration: %.2f ft/s^2\n', ...
    plan.peakSpeed, plan.peakAcceleration);
fprintf('Accelerate: %.2f s / %.2f ft; forward flight: %.2f s / %.2f ft; decelerate: %.2f s / %.2f ft\n', ...
    plan.accelerationTime, plan.accelerationDistance, ...
    plan.cruiseTime, plan.cruiseDistance, ...
    plan.decelerationTime, plan.decelerationDistance);

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

%% Prepare to run simulation
clear target
target.RefInput = plan.RefInput;
simSetup;
open(model);
