%% Pre-avoidance baseline: waypoint mission intersects a square building

scriptDir = fileparts(mfilename('fullpath'));
mdl = 'uav_multirotor_step_response';
mdlFile = fullfile(scriptDir,[mdl '.slx']);
stopTime = 40;
waypoints = [0 0 0 0; ...
             5 0 -2 0; ...
             5 5 -2 0; ...
             0 5 -2 0; ...
             0 0 -2 0];

[simpleUAVScenario,building] = create_obstructed_uav_scenario();

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

east = positionNED(:,2);
north = positionNED(:,1);
up = -positionNED(:,3);
insideBuilding = east >= building.EastBounds(1) & ...
    east <= building.EastBounds(2) & ...
    north >= building.NorthBounds(1) & ...
    north <= building.NorthBounds(2) & ...
    up >= building.UpBounds(1) & up <= building.UpBounds(2);

dEast = max([building.EastBounds(1)-east,zeros(size(east)), ...
    east-building.EastBounds(2)],[],2);
dNorth = max([building.NorthBounds(1)-north,zeros(size(north)), ...
    north-building.NorthBounds(2)],[],2);
dUp = max([building.UpBounds(1)-up,zeros(size(up)), ...
    up-building.UpBounds(2)],[],2);
pointClearance = sqrt(dEast.^2+dNorth.^2+dUp.^2);
minimumObstacleClearance = min(pointClearance);
pathIntersectsBuilding = any(insideBuilding);

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
    maxCrossTrackError,finalSpeed,pathIntersectsBuilding, ...
    minimumObstacleClearance,elapsedWallTime, ...
    'VariableNames',{'FinalWaypointStatus','MissionCompletionTime_s', ...
    'FinalPositionError_m','MaxCrossTrackError_m','FinalSpeed_mps', ...
    'PathIntersectsBuilding','MinimumObstacleClearance_m', ...
    'ElapsedWallTime_s'});

fig = figure('Color','w','Position',[100 100 1100 760]);
ax = axes(fig);
show3D(simpleUAVScenario,Parent=ax);
hold(ax,'on');
hPath = plot3(ax,east,north,up,'b-','LineWidth',2);
hWaypoints = plot3(ax,waypoints(:,2),waypoints(:,1),-waypoints(:,3), ...
    'yo--','LineWidth',1.2,'MarkerFaceColor','y');
hIntersection = plot3(ax,east(insideBuilding),north(insideBuilding), ...
    up(insideBuilding),'r.','MarkerSize',14);
axis(ax,'equal'); grid(ax,'on'); view(ax,38,28);
title(ax,'Pre-avoidance baseline: path intersects square building');
legend(ax,[hPath hWaypoints hIntersection], ...
    {'Guidance Model path','Waypoints','Building intersection'}, ...
    'Location','best');

imageFile = fullfile(scriptDir,'uav_scenario_obstructed_mission.png');
matFile = fullfile(scriptDir,'uav_scenario_obstructed_results.mat');
csvFile = fullfile(scriptDir,'uav_scenario_obstructed_summary.csv');
exportgraphics(fig,imageFile,'Resolution',180);
save(matFile,'t','positionNED','velocityNED','crossTrack','status', ...
    'waypoints','building','insideBuilding','pointClearance','summary');
writetable(summary,csvFile);

disp(summary);
fprintf('Completed obstructed-scene mission in %.3f seconds wall time.\n', ...
    elapsedWallTime);
fprintf('Path intersects building: %d; minimum point clearance: %.3f m.\n', ...
    pathIntersectsBuilding,minimumObstacleClearance);
