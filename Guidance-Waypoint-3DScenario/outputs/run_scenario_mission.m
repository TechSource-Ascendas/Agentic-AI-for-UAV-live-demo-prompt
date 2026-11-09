%% Waypoint mission in a UAV Scenario with an obstacle on the path
% Model: UAV_GuidanceWaypointScenario.slx (no obstacle avoidance - baseline)
% InitFcn runs setup_uav_scenario.m, which recreates uavScene and `obstacle`.
% Frames: Guidance Model / Motion Write = NED [north east down];
%         scene meshes = ENU; lidar points = sensor frame (body FRD, NED-aligned axes).
% Outputs (next to this script): stage3_scenario_clearance_results.png,
%   stage3_scenario_scope.png, stage3_scenario_results.mat (resampled to 0.05 s;
%   lidar stored as per-frame counts, not full point clouds), stage3_scenario_summary.csv

here = fileparts(mfilename("fullpath")); addpath(here);
% Long clone paths break Windows builds (260-char limit): keep Simulink cache/codegen short
if strlength(here) > 80
    Simulink.fileGenControl("set", "CacheFolder", fullfile(tempdir, "uav_slprj"), ...
        "CodeGenFolder", fullfile(tempdir, "uav_slprj"), "createDir", true);
end
mdl = "UAV_GuidanceWaypointScenario";
open_system(mdl);
mws = get_param(mdl, "ModelWorkspace");
wp  = getVariable(mws, "waypoints");

tic; scOut = sim(mdl); wallTime = toc;
setup_uav_scenario;   % reload `obstacle` geometry in this workspace for the analysis

%% Signals
y   = scOut.yout;
col = @(s) reshape(s.Values.Data, [], numel(s.Values.Time)).';
t    = y{1}.Values.Time;
pos  = col(y{1});            % [N E D]
eul  = col(y{3});            % [yaw pitch roll]
xte  = y{7}.Values.Data(:);
stat = y{8}.Values.Data(:);
lid  = scOut.logsout.get("LidarPointCloud").Values;   % N-by-3 per frame, sensor frame

%% Obstacle clearance (UAV centre to box, NED). Negative = inside the building.
bMin = [obstacle.northLimits(1) obstacle.eastLimits(1) -obstacle.upLimits(2)];
bMax = [obstacle.northLimits(2) obstacle.eastLimits(2) -obstacle.upLimits(1)];
outside = max(max(bMin - pos, pos - bMax), 0);
dOut    = vecnorm(outside, 2, 2);
inside  = all(pos > bMin & pos < bMax, 2);
depth   = min(min(pos - bMin, bMax - pos), [], 2);      % distance to nearest face when inside
clearance = dOut;  clearance(inside) = -depth(inside);
uavRadius = 0.6;                                          % half of the 1.2 m quadrotor mesh

%% Lidar: shape, valid returns, and how many land on the building
nFrames = size(lid.Data, 3);
nValid = zeros(nFrames,1); nOnObstacle = zeros(nFrames,1);
mount = [0 0 -0.4];                                      % sensor mounting, body frame
for k = 1:nFrames
    P = lid.Data(:,:,k);
    P = P(all(isfinite(P),2), :);
    nValid(k) = size(P,1);
    if isempty(P), continue, end
    [~, i] = min(abs(t - lid.Time(k)));
    R = eul2rotm(eul(i,:), "ZYX");                       % body -> NED
    W = (R * (P + mount).').' + pos(i,:);                % world NED points
    tol = 0.3;
    onBox = all(W > bMin - tol & W < bMax + tol, 2);
    nOnObstacle(k) = nnz(onBox);
end

%% Report
iDone = find(stat == 1, 1);
fprintf("\n--- Scenario mission (no avoidance) ---\n");
fprintf("Simulation completed to t = %.1f s (wall time %.1f s)\n", t(end), wallTime);
if isempty(iDone), fprintf("Waypoint status: NOT completed\n");
else, fprintf("Waypoint status: completed at t = %.1f s\n", t(iDone)); end
fprintf("Final position error to last waypoint: %.3f m\n", norm(pos(end,:) - wp(end,:)));
fprintf("Max cross-track error after take-off (t > 5 s): %.3f m\n", max(abs(xte(t > 5))));
fprintf("Lidar output per frame: %d-by-%d (unorganized), %d frames\n", size(lid.Data,1), size(lid.Data,2), nFrames);
fprintf("Lidar valid returns per frame: min %d, max %d\n", min(nValid), max(nValid));
fprintf("Lidar frames with returns on the building: %d (max %d points in one frame)\n", nnz(nOnObstacle), max(nOnObstacle));
fprintf("Minimum clearance UAV centre -> building: %.2f m\n", min(clearance));
if any(clearance < uavRadius)
    tHit = t(clearance < uavRadius);
    fprintf("COLLISION: UAV body (radius %.1f m) intersects the building from t = %.1f s to %.1f s (%.1f s inside the centre).\n", ...
        uavRadius, tHit(1), tHit(end), sum(diff(t) .* inside(1:end-1)));
else
    fprintf("No collision: clearance stays above the UAV radius (%.1f m).\n", uavRadius);
end

%% Plots
figure("Name", "Scenario mission", "Color", "w", "Position", [80 80 1400 520]);
tl = tiledlayout(1, 3, "TileSpacing", "compact");
title(tl, "Waypoint mission through the scene - no obstacle avoidance");

nexttile; hold on
fill(obstacle.eastLimits([1 2 2 1]), obstacle.northLimits([1 1 2 2]), [0.85 0.3 0.2], "FaceAlpha", 0.6);
plot(wp(:,2), wp(:,1), "ks--", "MarkerFaceColor", "y");
plot(pos(:,2), pos(:,1), "b", "LineWidth", 1.8);
plot(pos(clearance < uavRadius,2), pos(clearance < uavRadius,1), "r.", "MarkerSize", 10);
axis equal; grid on; xlabel("East [m]"); ylabel("North [m]"); title("Top view");
legend("Building", "Waypoints", "Flown path", "Collision", "Location", "southoutside", "NumColumns", 2);

nexttile; plot(t, clearance, "LineWidth", 1.5); hold on
yline(uavRadius, "r--", "UAV radius"); yline(0, "k:");
grid on; xlabel("Time [s]"); ylabel("[m]"); title("Clearance: UAV centre to building");

nexttile; plot(lid.Time, nValid, lid.Time, nOnObstacle, "LineWidth", 1.2);
grid on; xlabel("Time [s]"); ylabel("Points"); title("Lidar returns per frame");
legend("All valid returns", "Returns on building", "Location", "best");

exportgraphics(gcf, fullfile(here, "stage3_scenario_clearance_results.png"), "Resolution", 110);
fScope = findall(0, "Type", "figure", "Name", "UAV Scenario Scope");
if ~isempty(fScope)
    delete(findall(fScope(1), "Type", "datatip"));
    exportgraphics(fScope(1), fullfile(here, "stage3_scenario_scope.png"), "Resolution", 150);
end

%% Save results next to this script (signals resampled to 0.05 s to keep the file small)
hit = clearance < uavRadius; tHit = t(hit);
metrics = struct( ...
    "SimStopTime_s",          t(end), ...
    "WaypointsDone",          ~isempty(iDone), ...
    "TimeAllWaypointsDone_s", t(max([iDone; NaN])), ...
    "FinalPositionError_m",   norm(pos(end,:) - wp(end,:)), ...
    "MaxCrossTrack_after5s_m", max(abs(xte(t > 5))), ...
    "LidarPointsPerFrame",    size(lid.Data,1), ...
    "LidarFrames",            nFrames, ...
    "MinClearance_m",         min(clearance), ...
    "UAVRadius_m",            uavRadius, ...
    "Collision",              any(hit), ...
    "CollisionStart_s",       min([tHit; NaN]), ...
    "CollisionEnd_s",         max([tHit; NaN]));
[tu, iu] = unique(t);
ts = (0:0.05:t(end))';
rs = @(A) interp1(tu, A(iu,:), ts);
results = struct("t", ts, "WorldPosition", rs(pos), "EulerZYX", rs(eul), "CrossTrackError", rs(xte), ...
    "WaypointStatus", interp1(tu, double(stat(iu)), ts, "previous"), "Clearance", rs(clearance));
lidarSummary = struct("t", lid.Time(:), "ValidReturns", nValid, "ReturnsOnBuilding", nOnObstacle);
save(fullfile(here, "stage3_scenario_results.mat"), "results", "lidarSummary", "metrics", "obstacle", "wp");
writetable(struct2table(metrics), fullfile(here, "stage3_scenario_summary.csv"));
