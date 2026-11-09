%% Multirotor waypoint following - UAV_GuidanceWaypointFollowing.slx
% Waypoint Follower -> Position_Yaw_Controller -> Guidance Model (closed loop).
% Frame: NED [north east down]; altitude = -down. EulerZYX = [yaw pitch roll].
% The UAV Animation window shows the flight live during simulation.
% Outputs (next to this script): stage2_waypoint_following_results.png,
%   stage2_waypoint_results.mat (resampled to 0.05 s), stage2_waypoint_summary.csv

here = fileparts(mfilename("fullpath")); addpath(here);
% Long clone paths break Windows builds (260-char limit): keep Simulink cache/codegen short
if strlength(here) > 80
    Simulink.fileGenControl("set", "CacheFolder", fullfile(tempdir, "uav_slprj"), ...
        "CodeGenFolder", fullfile(tempdir, "uav_slprj"), "createDir", true);
end
mdl = "UAV_GuidanceWaypointFollowing";
open_system(mdl);
mws = get_param(mdl, "ModelWorkspace");
wp  = getVariable(mws, "waypoints");          % N-by-3 NED
ctrl = getVariable(mws, "ctrl");

wpOut = sim(mdl);

%% Extract logged signals (yout order = root outports 1..8)
y   = wpOut.yout;
col = @(s) reshape(s.Values.Data, [], numel(s.Values.Time)).';
t    = y{1}.Values.Time;
pos  = col(y{1});            % WorldPosition [N E D]
vel  = col(y{2});            % WorldVelocity
eul  = col(y{3});            % EulerZYX [yaw pitch roll]
lap  = col(y{6});            % LookaheadPoint
xte  = y{7}.Values.Data(:);  % CrossTrackError
stat = y{8}.Values.Data(:);  % WaypointStatus (1 = all waypoints done)

L = wpOut.logsout;
cmd.Roll    = L.get("Roll").Values;
cmd.Pitch   = L.get("Pitch").Values;
cmd.YawRate = L.get("YawRate").Values;
cmd.Thrust  = L.get("Thrust").Values;

%% Metrics
iDone = find(stat == 1, 1);
finalErr = norm(pos(end,:) - wp(end,:));
% Distance from the flown path to the planned polyline (independent check)
dPath = zeros(numel(t),1);
for k = 1:numel(t)
    d = inf;
    for s = 1:size(wp,1)-1
        a = wp(s,:); b = wp(s+1,:); ab = b - a;
        u = max(0, min(1, dot(pos(k,:) - a, ab) / max(dot(ab,ab), eps)));
        d = min(d, norm(pos(k,:) - (a + u*ab)));
    end
    dPath(k) = d;
end
afterTakeoff = t > 5;   % take-off starts at ground level, below waypoint 1
fprintf("\n--- Waypoint mission results ---\n");
fprintf("Simulation completed to t = %.1f s\n", t(end));
if isempty(iDone)
    fprintf("Waypoint status: NOT completed (status = %d)\n", stat(end));
else
    fprintf("Waypoint status: completed (status = 1) at t = %.1f s\n", t(iDone));
end
fprintf("Final position error to last waypoint: %.3f m\n", finalErr);
fprintf("Final speed: %.3f m/s (hover check)\n", norm(vel(end,:)));
fprintf("Max cross-track error (block output), whole run: %.3f m (includes start on the ground)\n", max(abs(xte)));
fprintf("Max cross-track error (block output), t > 5 s: %.3f m\n", max(abs(xte(afterTakeoff))));
fprintf("Max distance from planned path after take-off (t > 5 s): %.3f m\n", max(dPath(afterTakeoff)));
fprintf("Max horizontal speed: %.2f m/s (limit %.1f)\n", max(vecnorm(vel(:,1:2),2,2)), ctrl.vMaxXY);
fprintf("Max tilt command: roll %.3f / pitch %.3f rad (limit %.2f)\n", ...
    max(abs(cmd.Roll.Data)), max(abs(cmd.Pitch.Data)), ctrl.tiltMax);

%% Plots
figure("Name", "Waypoint following", "Color", "w", "Position", [80 80 1500 800]);
tl = tiledlayout(3, 4, "TileSpacing", "compact");
title(tl, "Multirotor Guidance Model - waypoint following (NED, altitude = -down)");

nexttile([2 2]);   % top view: east on x-axis, north on y-axis
plot(wp(:,2), wp(:,1), "ks--", "MarkerSize", 10, "MarkerFaceColor", "y"); hold on
plot(pos(:,2), pos(:,1), "b", "LineWidth", 1.8);
plot(pos(1,2), pos(1,1), "go", "MarkerFaceColor", "g");
plot(pos(end,2), pos(end,1), "rp", "MarkerSize", 14, "MarkerFaceColor", "r");
text(wp(:,2)+0.6, wp(:,1)+0.6, "WP" + string(1:size(wp,1))');
axis equal; grid on; xlabel("East [m]"); ylabel("North [m]");
legend("Planned waypoints", "Flown path", "Start", "End", "Location", "best");
title("Top view");

nexttile([2 2]);
plot3(wp(:,2), wp(:,1), -wp(:,3), "ks--", "MarkerFaceColor", "y"); hold on
plot3(pos(:,2), pos(:,1), -pos(:,3), "b", "LineWidth", 1.8);
grid on; axis equal; view(-35, 25);
xlabel("East [m]"); ylabel("North [m]"); zlabel("Altitude [m]"); title("3-D path");

lg = {"UAV", "Lookahead point"};
nexttile; plot(t, pos(:,1), t, lap(:,1), "--"); grid on; title("North"); legend(lg); xlabel("Time [s]"); ylabel("[m]");
nexttile; plot(t, pos(:,2), t, lap(:,2), "--"); grid on; title("East");  legend(lg); xlabel("Time [s]"); ylabel("[m]");
nexttile; plot(t, -pos(:,3), t, -lap(:,3), "--"); grid on; title("Altitude"); legend(lg, "Location", "southeast"); xlabel("Time [s]"); ylabel("[m]");
nexttile; yyaxis left; plot(t, xte); ylabel("Cross-track error [m]");
yyaxis right; stairs(t, double(stat)); ylim([-0.1 1.2]); ylabel("Status");
grid on; xlabel("Time [s]"); title("Tracking");

exportgraphics(gcf, fullfile(here, "stage2_waypoint_following_results.png"), "Resolution", 110);

%% Save results next to this script (signals resampled to 0.05 s to keep the file small)
metrics = struct( ...
    "SimStopTime_s",           t(end), ...
    "WaypointsDone",           ~isempty(iDone), ...
    "TimeAllWaypointsDone_s",  t(max([iDone; NaN])), ...
    "FinalPositionError_m",    finalErr, ...
    "FinalSpeed_mps",          norm(vel(end,:)), ...
    "MaxCrossTrack_after5s_m", max(abs(xte(afterTakeoff))), ...
    "MaxPathDistance_after5s_m", max(dPath(afterTakeoff)), ...
    "MaxHorizontalSpeed_mps",  max(vecnorm(vel(:,1:2),2,2)), ...
    "MaxRollCmd_rad",          max(abs(cmd.Roll.Data)), ...
    "MaxPitchCmd_rad",         max(abs(cmd.Pitch.Data)));
[tu, iu] = unique(t);
ts = (0:0.05:t(end))';
rs = @(A) interp1(tu, A(iu,:), ts);
results = struct("t", ts, "WorldPosition", rs(pos), "WorldVelocity", rs(vel), "EulerZYX", rs(eul), ...
    "LookaheadPoint", rs(lap), "CrossTrackError", rs(xte), "WaypointStatus", interp1(tu, double(stat(iu)), ts, "previous"), ...
    "PathDistance", rs(dPath));
save(fullfile(here, "stage2_waypoint_results.mat"), "results", "metrics", "wp", "ctrl");
writetable(struct2table(metrics), fullfile(here, "stage2_waypoint_summary.csv"));
