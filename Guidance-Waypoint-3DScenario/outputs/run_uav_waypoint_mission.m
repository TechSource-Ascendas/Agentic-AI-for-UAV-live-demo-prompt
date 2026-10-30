%% Reduced-order multirotor waypoint-following mission
% Frame: NED [north east down]. Altitude above origin is -down.
% Euler order: [yaw pitch roll]. Current yaw approximates course for this
% low-speed reduced-order demonstration.

scriptDir = fileparts(mfilename('fullpath'));
mdl = 'uav_multirotor_step_response';
mdlFile = fullfile(scriptDir, [mdl '.slx']);
stopTime = 40;
waypoints = [0 0 0 0; ...
             5 0 -2 0; ...
             5 5 -2 0; ...
             0 5 -2 0; ...
             0 0 -2 0];

in = Simulink.SimulationInput(mdlFile);
in = in.setModelParameter('StopTime', num2str(stopTime));
in = in.setVariable('WaypointMode', 1, Workspace=mdl);
in = in.setVariable('WaypointsNED', waypoints, Workspace=mdl);
in = in.setVariable('WaypointMarkers', waypoints(:,1:3), Workspace=mdl);

wallClock = tic;
out = sim(in);
elapsedWallTime = toc(wallClock);

t = out.WorldPosition.Time(:);
positionNED = squeeze(out.WorldPosition.Data)';
velocityNED = squeeze(out.WorldVelocity.Data)';
eulerZYX = squeeze(out.EulerZYX.Data)';
crossTrack = squeeze(out.CrossTrackError.Data);
crossTrack = crossTrack(:);
status = squeeze(out.WaypointStatus.Data);
status = status(:);

finalPositionError = norm(positionNED(end,:) - waypoints(end,1:3));
finalSpeed = norm(velocityNED(end,:));
maxCrossTrackError = max(crossTrack);
completedIndex = find(status >= 1, 1, 'first');
if isempty(completedIndex)
    missionCompletionTime = NaN;
else
    missionCompletionTime = out.WaypointStatus.Time(completedIndex);
end

summary = table(status(end), missionCompletionTime, finalPositionError, ...
    maxCrossTrackError, finalSpeed, elapsedWallTime, ...
    'VariableNames', {'FinalWaypointStatus','MissionCompletionTime_s', ...
    'FinalPositionError_m','MaxCrossTrackError_m','FinalSpeed_mps', ...
    'ElapsedWallTime_s'});

fig = figure('Color','w','Position',[100 100 1250 760]);
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile([2 1]);
plot3(positionNED(:,1), positionNED(:,2), -positionNED(:,3), ...
    'LineWidth',1.8,'DisplayName','UAV path');
hold on;
plot3(waypoints(:,1),waypoints(:,2),-waypoints(:,3),'ko--', ...
    'MarkerFaceColor',[1 0.75 0],'DisplayName','Waypoints');
grid on; axis equal;
xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
title('Waypoint trajectory (NED plant, altitude shown positive up)');
legend('Location','best'); view(38,28);

nexttile;
plot(t,positionNED,'LineWidth',1.2);
grid on;
xlabel('Time (s)'); ylabel('Position (m)');
title('NED position');
legend('North','East','Down','Location','best');

nexttile;
yyaxis left;
plot(out.CrossTrackError.Time,crossTrack,'LineWidth',1.3);
ylabel('Cross-track error (m)');
yyaxis right;
stairs(out.WaypointStatus.Time,status,'LineWidth',1.3);
ylabel('Waypoint status'); ylim([-0.05 1.05]);
grid on; xlabel('Time (s)');
title('Tracking error and completion status');

sgtitle('Multirotor waypoint-following mission');

plotFile = fullfile(scriptDir,'uav_waypoint_mission_results.png');
matFile = fullfile(scriptDir,'uav_waypoint_mission_results.mat');
csvFile = fullfile(scriptDir,'uav_waypoint_mission_summary.csv');
exportgraphics(fig,plotFile,'Resolution',180);
save(matFile,'t','positionNED','velocityNED','eulerZYX','crossTrack', ...
    'status','waypoints','summary');
writetable(summary,csvFile);

disp(summary);
fprintf('Completed %.1f-second mission simulation in %.3f seconds wall time.\n', ...
    stopTime,elapsedWallTime);
fprintf('Saved mission plot: %s\n',plotFile);
