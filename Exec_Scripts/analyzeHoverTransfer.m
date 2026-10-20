function summary = analyzeHoverTransfer(SimOut, plan)
%ANALYZEHOVERTRANSFER Report actual fixed-wing flight and hover tracking.
% Works on partial logs returned by sim(model, 'CaptureErrors', 'on').
% Fixed-wing flight here means all eight lift rotors <= 1 RPM (the cruise
% model tolerance) and the pusher >= 550 RPM. Times are measured from logs,
% not inferred from the constant-speed phase of the reference trajectory.

rpm = samples(SimOut.Vehicle.PropAct.EngSpeed)*60/(2*pi);
time = SimOut.Vehicle.PropAct.EngSpeed.Time;
position = samples(SimOut.Vehicle.EOM.InertialData.Pos_bii);
velocity = samples(SimOut.Vehicle.EOM.InertialData.Vel_bIi);
bodyVelocity = samples(SimOut.Vehicle.EOM.AirRelativeData.Vel_bWb);
alpha = samples(SimOut.Vehicle.EOM.AirRelativeData.alpha)*180/pi;
fixedWing = max(abs(rpm(:,1:8)), [], 2) <= 1 & rpm(:,9) >= 550;
edges = diff([false; fixedWing; false]);
starts = find(edges == 1);
stops = find(edges == -1)-1;
summary.fixedWingIntervals = [time(starts) time(stops)];
summary.fixedWingDuration = sum(time(stops)-time(starts));
summary.finalTime = time(end);
summary.completed = abs(time(end)-plan.stopTime) < 0.01;
summary.finalPosition = position(end,:);
summary.finalPositionError = position(end,:)-plan.position(end,:);
summary.finalSpeed = norm(velocity(end,:));
summary.heightRange = [min(-position(:,3)) max(-position(:,3))];
summary.finalBodyForwardSpeedKnots = bodyVelocity(end,1)/1.687809857;
if any(fixedWing)
    summary.fixedWingAlphaRangeDeg = [min(alpha(fixedWing)) max(alpha(fixedWing))];
else
    summary.fixedWingAlphaRangeDeg = [NaN NaN];
end

fprintf('Logged %.3f / %.3f s; completed: %d\n', time(end), plan.stopTime, summary.completed);
fprintf('Fixed-wing intervals [start stop] s (%.3f s total):\n', summary.fixedWingDuration);
disp(summary.fixedWingIntervals);
fprintf('Local NED height range: %.3f to %.3f ft\n', summary.heightRange);
fprintf('Fixed-wing angle of attack: %.3f to %.3f deg\n', summary.fixedWingAlphaRangeDeg);
fprintf('Final position error [N E D]: %s ft; speed: %.4f ft/s\n', ...
    mat2str(summary.finalPositionError,6), summary.finalSpeed);
fprintf('Final body-forward airspeed: %.3f knots; rotor RPM: %s\n', ...
    summary.finalBodyForwardSpeedKnots, mat2str(rpm(end,:),6));
end

function data = samples(ts)
data = squeeze(ts.Data);
if ~ts.IsTimeFirst
    data = data';
end
% A single vector-valued sample is returned by squeeze as a column.
data = reshape(data, numel(ts.Time), []);
end
