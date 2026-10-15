%% sim parameters
model = 'GUAM';
userStruct.variants.refInputType = 3;

%% Choose hover positions in ft, using [north east down].
startPosition = [0 0 0];
endPosition = [200 100 -50];

plan = planMinimumTimeHover(startPosition, endPosition, ...
    'MaxSpeed', 20, 'MaxAcceleration', 4, 'MaxJerk', 2, ...
    'SampleTime', 0.05, 'HoldTime', 5);

fprintf('Transfer: %.2f ft in %.2f s; limiting constraint: %s\n', ...
    plan.distance, plan.duration, plan.limitingConstraint);
fprintf('Peak speed: %.2f ft/s; acceleration: %.2f ft/s^2; jerk: %.2f ft/s^3\n', ...
    plan.peakSpeed, plan.peakAcceleration, plan.peakJerk);

figure('Name', 'Minimum-time hover transfer');
subplot(2,2,1);
plot3(plan.position(:,1), plan.position(:,2), -plan.position(:,3));
xlabel('North (ft)'); ylabel('East (ft)'); zlabel('Up (ft)');
grid on; axis equal;
subplot(2,2,2);
plot(plan.time, sqrt(sum(plan.velocity.^2,2)));
yline(plan.limits.MaxSpeed, '--');
xlabel('Time (s)'); ylabel('Speed (ft/s)'); grid on;
subplot(2,2,3);
plot(plan.time, sqrt(sum(plan.acceleration.^2,2)));
yline(plan.limits.MaxAcceleration, '--');
xlabel('Time (s)'); ylabel('Acceleration (ft/s^2)'); grid on;
subplot(2,2,4);
plot(plan.time, sqrt(sum(plan.jerk.^2,2)));
yline(plan.limits.MaxJerk, '--');
xlabel('Time (s)'); ylabel('Jerk (ft/s^3)'); grid on;

%% Prepare to run simulation
clear target
target.RefInput = plan.RefInput;
simSetup;
open(model);
