%% analyze_sample_log.m
% Stage 1 of Demo 1: flight overview and anomaly check of outputs/Sample_Log.tlog
% (PX4 quadrotor, MAVLink telemetry log) with UAV Toolbox.
%
% Reproduces what the agent did for the prompt
%   "Please help me analyze this log: what happened to the UAV, as an overview?"
%
% Outputs (in ../outputs, relative to this script):
%   Sample_Log_overview.png          ground track, altitude + events, velocity, motor outputs
%   Sample_Log_gps_glitch.png        raw GPS vs EKF track, EKF test ratio and GPS error
%   Sample_Log_timeline.csv          flight events (status text, modes, arm, landed state)
%   Sample_Log_analysis_results.mat  summary struct, timeline table, key time series
%
% Requires: UAV Toolbox (mavlinkdialect, mavlinktlog, readmsg)

%% Read log
here    = fileparts(mfilename('fullpath'));
outDir  = fullfile(here, '..', 'outputs');
logFile = fullfile(outDir, 'Sample_Log.tlog');

dialect = mavlinkdialect('ardupilotmega.xml', 2);
tl      = mavlinktlog(logFile, dialect);

hb   = readAll(tl, 'HEARTBEAT', 1);
stxt = readAll(tl, 'STATUSTEXT', 1);
ess  = readAll(tl, 'EXTENDED_SYS_STATE', 1);
lpos = readAll(tl, 'LOCAL_POSITION_NED', 1);
gpos = readAll(tl, 'GLOBAL_POSITION_INT', 1);
graw = readAll(tl, 'GPS_RAW_INT', 1);
sys  = readAll(tl, 'SYS_STATUS', 1);
est  = readAll(tl, 'ESTIMATOR_STATUS', 1);
vib  = readAll(tl, 'VIBRATION', 1);
srv  = readAll(tl, 'SERVO_OUTPUT_RAW', 1);

t0  = min([hb.Time(1) stxt.Time(1) lpos.Time(1)]);
sec = @(T) seconds(T.Time - t0);

%% Flight timeline: status text, arming / PX4 main mode changes, landed state
modeNames = containers.Map(num2cell(1:8), ...
    {'MANUAL','ALTCTL','POSCTL','AUTO','ACRO','OFFBOARD','STABILIZED','RATTITUDE'});
landedNames = {'UNDEFINED','ON_GROUND','IN_AIR','TAKEOFF','LANDING'};

tH    = sec(hb);
armed = bitand(double(hb.base_mode), 128) > 0;
mainM = bitand(bitshift(double(hb.custom_mode), -16), 255);   % PX4 main mode in bits 16-23

ev = table('Size', [0 3], 'VariableTypes', {'double','string','string'}, ...
    'VariableNames', {'Time_s','Source','Event'});
for k = 1:height(stxt)
    ev(end+1, :) = {sec(stxt(k, :)), "STATUSTEXT", strtrim(string(stxt.text{k}))}; %#ok<SAGROW>
end
chg = [1; find(diff(mainM) ~= 0 | diff(armed) ~= 0) + 1];
for k = chg'
    if isKey(modeNames, mainM(k)), m = modeNames(mainM(k)); else, m = sprintf('MODE %d', mainM(k)); end
    a = "disarmed"; if armed(k), a = "armed"; end
    ev(end+1, :) = {tH(k), "HEARTBEAT", sprintf("Mode %s, %s", m, a)}; %#ok<SAGROW>
end
ls  = double(ess.landed_state);
chg = [1; find(diff(ls) ~= 0) + 1];
for k = chg'
    ev(end+1, :) = {sec(ess(k, :)), "EXTENDED_SYS_STATE", "Landed state " + landedNames{ls(k) + 1}}; %#ok<SAGROW>
end
timeline = sortrows(ev, 'Time_s');
writetable(timeline, fullfile(outDir, 'Sample_Log_timeline.csv'));

%% Trajectory, GPS and estimator health
tP  = sec(lpos);
gnd = median(-double(lpos.z(tP < 20)));                 % ground level before takeoff
pN  = double(lpos.x);  pE = double(lpos.y);  pU = -double(lpos.z) - gnd;
vH  = hypot(double(lpos.vx), double(lpos.vy));

lat0 = double(gpos.lat(1)) / 1e7;  lon0 = double(gpos.lon(1)) / 1e7;
[gN, gE] = geo2ne(double(graw.lat) / 1e7, double(graw.lon) / 1e7, lat0, lon0);
gN = gN + pN(1);  gE = gE + pE(1);                       % align to the local origin
tG = sec(graw);
gpsErr = hypot(gN - interp1(tP, pN, tG, 'linear', 'extrap'), ...
               gE - interp1(tP, pE, tG, 'linear', 'extrap'));

tE    = sec(est);
ratio = double(est.pos_horiz_ratio);
bad   = tE(ratio > 1);

tS   = sec(sys);
volt = double(sys.voltage_battery) / 1000;
batt = double(sys.battery_remaining);

tV   = sec(srv);
pwm  = double([srv.servo1_raw srv.servo2_raw srv.servo3_raw srv.servo4_raw]);

isKill = contains(timeline.Event, "Kill-switch engaged");
summary = struct( ...
    'LogDuration_s',        seconds(tl.EndTime - tl.StartTime), ...
    'NumMessages',          tl.NumMessages, ...
    'NumPacketsLost',       tl.NumPacketsLost, ...
    'MaxAltitude_m',        max(pU), ...
    'MaxHorizontalRange_m', max(hypot(pN, pE)), ...
    'MaxGroundSpeed_mps',   max(vH), ...
    'BatteryStart_pct',     batt(1), ...
    'BatteryEnd_pct',       batt(end), ...
    'VoltageMin_V',         min(volt(volt > 0)), ...
    'GpsSatellitesMin',     min(double(graw.satellites_visible)), ...
    'EkfPosHorizRatioMax',  max(ratio), ...
    'GpsGlitchWindow_s',    [min(bad) max(bad)], ...
    'GpsEkfErrorMax_m',     max(gpsErr), ...
    'VibrationMax',         max([vib.vibration_x; vib.vibration_y; vib.vibration_z]), ...
    'KillSwitchInFlight_s', max(timeline.Time_s(isKill)));
disp(summary)
disp(timeline)

series = struct('tP', tP, 'pN', pN, 'pE', pE, 'pU', pU, 'vH', vH, 'tG', tG, 'gN', gN, 'gE', gE, ...
    'gpsErr', gpsErr, 'tE', tE, 'ekfPosHorizRatio', ratio, 'tS', tS, 'voltage', volt, ...
    'battery', batt, 'tV', tV, 'pwm', pwm, 'tH', tH, 'armed', armed, 'mainMode', mainM);
save(fullfile(outDir, 'Sample_Log_analysis_results.mat'), 'summary', 'timeline', 'series');

%% Figure 1: overview
evT = [21.0 23.0 21.7 127.7 167.3 170.7];
evL = {'arm', 'takeoff', 'OFFBOARD', 'POSCTL', 'kill', 'disarm'};
fig = figure('Name', 'Sample_Log overview', 'Color', 'w', 'Visible', 'off', 'Position', [100 100 1100 750]);
tiledlayout(fig, 2, 2);
ax = nexttile;  plot(ax, pE, pN, 'b');  hold(ax, 'on');  plot(ax, 0, 0, 'gp', 'MarkerSize', 12);
axis(ax, 'equal');  grid(ax, 'on');  xlabel(ax, 'East [m]');  ylabel(ax, 'North [m]');  title(ax, 'Ground track (local NED)');
ax = nexttile;  plot(ax, tP, pU);  grid(ax, 'on');  xline(ax, evT, ':', evL);
xlabel(ax, 't [s]');  ylabel(ax, 'Alt [m]');  title(ax, 'Altitude');
ax = nexttile;  plot(ax, tP, vH);  hold(ax, 'on');  plot(ax, tP, -double(lpos.vz));  grid(ax, 'on');
legend(ax, 'ground speed', 'climb rate');  xlabel(ax, 't [s]');  ylabel(ax, 'm/s');  title(ax, 'Velocity');
ax = nexttile;  plot(ax, tV, pwm);  grid(ax, 'on');  xlabel(ax, 't [s]');  ylabel(ax, 'PWM');
title(ax, 'Motor outputs');  legend(ax, 'M1', 'M2', 'M3', 'M4');
exportgraphics(fig, fullfile(outDir, 'Sample_Log_overview.png'), 'Resolution', 120);
close(fig);

%% Figure 2: GPS glitch evidence
fig = figure('Name', 'Sample_Log GPS glitch', 'Color', 'w', 'Visible', 'off', 'Position', [100 100 1200 480]);
tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
ax = nexttile;  hold(ax, 'on');  grid(ax, 'on');  axis(ax, 'equal');
inG = tG >= summary.GpsGlitchWindow_s(1) & tG <= summary.GpsGlitchWindow_s(2);
plot(ax, pE, pN, '-', 'Color', [.1 .35 .8], 'LineWidth', 1.5);
scatter(ax, gE(~inG & tG > 20), gN(~inG & tG > 20), 12, [.5 .5 .5], 'filled');
scatter(ax, gE(inG), gN(inG), 22, [.85 .2 .15], 'filled');
plot(ax, 0, 0, 'p', 'MarkerSize', 14, 'MarkerFaceColor', [1 .8 0], 'MarkerEdgeColor', 'k');
legend(ax, 'EKF track', 'Raw GPS (normal)', 'Raw GPS (glitch window)', 'Home', 'Location', 'southoutside', 'Orientation', 'horizontal');
xlabel(ax, 'East [m]');  ylabel(ax, 'North [m]');  title(ax, 'EKF track vs raw GPS fixes');
ax = nexttile;  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
xregion(ax, summary.GpsGlitchWindow_s(1), summary.GpsGlitchWindow_s(2), 'FaceColor', [1 .85 .85]);
yyaxis(ax, 'left');   plot(ax, tE, ratio, '-o', 'Color', [.7 .1 .1], 'MarkerSize', 3, 'LineWidth', 1.2);
yline(ax, 1, '--', 'reject threshold', 'Color', [.7 .1 .1]);  ylabel(ax, 'EKF pos-H test ratio');  ax.YColor = [.7 .1 .1];
yyaxis(ax, 'right');  plot(ax, tG, gpsErr, '-', 'Color', [.2 .4 .8]);  ylabel(ax, '|GPS - EKF| [m]');  ax.YColor = [.2 .4 .8];
xlim(ax, [15 180]);  xlabel(ax, 'Log time [s]');
title(ax, sprintf('GPS rejected %.0f-%.0f s (ratio up to %.1f)', summary.GpsGlitchWindow_s, summary.EkfPosHorizRatioMax));
exportgraphics(fig, fullfile(outDir, 'Sample_Log_gps_glitch.png'), 'Resolution', 120);
close(fig);
fprintf('Saved overview, GPS-glitch figure, timeline CSV and results MAT to %s\n', outDir);

%% ---------- helpers
function T = readAll(tl, name, sysID)
% Read one message type, merging the MAVLink v1 and v2 tables readmsg returns.
    r = readmsg(tl, 'MessageName', name, 'SystemID', sysID);
    parts = r.Messages;
    common = parts{1}.Properties.VariableNames;
    for i = 2:numel(parts)
        common = intersect(common, parts{i}.Properties.VariableNames, 'stable');
    end
    parts = cellfun(@(p) p(:, common), parts, 'UniformOutput', false);
    T = sortrows(vertcat(parts{:}));
    [~, iu] = unique(T.Time, 'stable');          % drop duplicate timestamps
    T = T(iu, :);
end

function [dN, dE] = geo2ne(lat, lon, lat0, lon0)
% Small-area flat-earth conversion to north/east metres.
    R  = 6378137;
    dN = deg2rad(lat - lat0) * R;
    dE = deg2rad(lon - lon0) * R .* cosd(lat0);
end
