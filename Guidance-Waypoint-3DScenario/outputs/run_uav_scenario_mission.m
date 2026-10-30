%% Waypoint mission displayed in a UAV Scenario
% Recreate the handle-based scenario before every simulation.

scriptDir = fileparts(mfilename('fullpath'));
mdl = 'uav_multirotor_step_response';
mdlFile = fullfile(scriptDir,[mdl '.slx']);
stopTime = 40;
waypoints = [0 0 0 0; ...
             5 0 -2 0; ...
             5 5 -2 0; ...
             0 5 -2 0; ...
             0 0 -2 0];

simpleUAVScenario = create_simple_uav_scenario();

in = Simulink.SimulationInput(mdlFile);
in = in.setModelParameter('StopTime',num2str(stopTime));
in = in.setVariable('simpleUAVScenario',simpleUAVScenario,Workspace=mdl);
in = in.setVariable('WaypointMode',1,Workspace=mdl);
in = in.setVariable('WaypointsNED',waypoints,Workspace=mdl);
in = in.setVariable('WaypointMarkers',waypoints(:,1:3),Workspace=mdl);

wallClock = tic;
out = sim(in);
elapsedWallTime = toc(wallClock);

t = out.WorldPosition.Time(:);
positionNED = squeeze(out.WorldPosition.Data)';
velocityNED = squeeze(out.WorldVelocity.Data)';
crossTrack = squeeze(out.CrossTrackError.Data);
crossTrack = crossTrack(:);
status = squeeze(out.WaypointStatus.Data);
status = status(:);

finalPositionError = norm(positionNED(end,:) - waypoints(end,1:3));
finalSpeed = norm(velocityNED(end,:));
maxCrossTrackError = max(crossTrack);
completedIndex = find(status >= 1,1,'first');
if isempty(completedIndex)
    missionCompletionTime = NaN;
else
    missionCompletionTime = out.WaypointStatus.Time(completedIndex);
end

summary = table(status(end),missionCompletionTime,finalPositionError, ...
    maxCrossTrackError,finalSpeed,elapsedWallTime, ...
    'VariableNames',{'FinalWaypointStatus','MissionCompletionTime_s', ...
    'FinalPositionError_m','MaxCrossTrackError_m','FinalSpeed_mps', ...
    'ElapsedWallTime_s'});

% Static scene summary: show3D uses ENU axes, so swap north/east and negate down.
fig = figure('Color','w','Position',[100 100 1100 760]);
ax = axes(fig);
show3D(simpleUAVScenario,Parent=ax);
hold(ax,'on');
hPath = plot3(ax,positionNED(:,2),positionNED(:,1),-positionNED(:,3), ...
    'b-','LineWidth',2,'DisplayName','Guidance Model path');
hWaypoints = plot3(ax,waypoints(:,2),waypoints(:,1),-waypoints(:,3), ...
    'yo--','LineWidth',1.2,'MarkerFaceColor','y','DisplayName','Waypoints');
axis(ax,'equal'); grid(ax,'on'); view(ax,38,28);
title(ax,'Guidance Model waypoint mission in UAV Scenario');
legend(ax,[hPath hWaypoints],{'Guidance Model path','Waypoints'}, ...
    'Location','best');

imageFile = fullfile(scriptDir,'uav_scenario_mission.png');
matFile = fullfile(scriptDir,'uav_scenario_mission_results.mat');
csvFile = fullfile(scriptDir,'uav_scenario_mission_summary.csv');
exportgraphics(fig,imageFile,'Resolution',180);
save(matFile,'t','positionNED','velocityNED','crossTrack','status', ...
    'waypoints','summary');
writetable(summary,csvFile);

disp(summary);
fprintf('Completed %.1f-second UAV Scenario mission in %.3f seconds wall time.\n', ...
    stopTime,elapsedWallTime);
fprintf('Saved scene image: %s\n',imageFile);
