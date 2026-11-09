%% create_sample_log_playback.m
% Playback animation of outputs/Sample_Log.tlog (PX4 quadrotor, MAVLink).
%
% Shows, synchronised in time:
%   - 3D view: EKF position track, UAV attitude (scaled mesh), raw GPS fixes
%     (to expose the ~79-137 s GPS glitch), home and landing points
%   - Altitude, ground speed and GPS-health panels with a moving cursor and
%     flight-mode bands
%   - Banner with time, mode, arm state, altitude, speed, battery and the
%     latest autopilot STATUSTEXT messages
%
% Output: outputs/Sample_Log_playback.mp4 and outputs/Sample_Log_playback_preview.png
% Requires: UAV Toolbox (mavlinkdialect, mavlinktlog, readmsg)

%% Settings
here      = fileparts(mfilename('fullpath'));
logFile   = fullfile(here, '..', 'outputs', 'Sample_Log.tlog');
videoFile = fullfile(here, '..', 'outputs', 'Sample_Log_playback.mp4');
prevFile  = fullfile(here, '..', 'outputs', 'Sample_Log_playback_preview.png');

speedUp    = 4;          % playback speed (x real time)
fps        = 30;         % video frame rate
tStart     = 18;         % [s] log time to start playback (just before arming)
tEnd       = 178;        % [s] log time to end playback
previewT   = 110;        % [s] log time used for the preview still
meshSpan   = 11;         % [m] displayed UAV size (not to scale, for visibility)
trailGps   = true;       % show raw GPS fixes

%% Read log
dialect = mavlinkdialect('ardupilotmega.xml', 2);
tl = mavlinktlog(logFile, dialect);

hb   = readAll(tl, 'HEARTBEAT', 1);
lpos = readAll(tl, 'LOCAL_POSITION_NED', 1);
att  = readAll(tl, 'ATTITUDE', 1);
gpos = readAll(tl, 'GLOBAL_POSITION_INT', 1);
graw = readAll(tl, 'GPS_RAW_INT', 1);
sys  = readAll(tl, 'SYS_STATUS', 1);
est  = readAll(tl, 'ESTIMATOR_STATUS', 1);
stxt = readAll(tl, 'STATUSTEXT', 1);

% Common time base: seconds since the start of the log
t0 = min([hb.Time(1) lpos.Time(1) att.Time(1) gpos.Time(1) stxt.Time(1)]);
sec = @(T) seconds(T.Time - t0);

% Position in ENU relative to the EKF origin (x = East, y = North, z = Up)
tP  = sec(lpos);
gnd = median(-double(lpos.z(tP < 20)));                 % on-ground level before takeoff
pE  = double(lpos.y);  pN = double(lpos.x);  pU = -double(lpos.z) - gnd;
vH  = hypot(double(lpos.vx), double(lpos.vy));

% Attitude
tA  = sec(att);
eulA = unwrapEul([double(att.yaw) double(att.pitch) double(att.roll)]);

% Raw GPS in the same local frame (origin = first EKF global position)
lat0 = double(gpos.lat(1))/1e7;  lon0 = double(gpos.lon(1))/1e7;
[gN0, gE0] = geo2ne(double(gpos.lat(1))/1e7, double(gpos.lon(1))/1e7, lat0, lon0);
eN = pN(1) - gN0;  eE = pE(1) - gE0;                     % align EKF global to local origin
[gN, gE] = geo2ne(double(graw.lat)/1e7, double(graw.lon)/1e7, lat0, lon0);
gN = gN + eN;  gE = gE + eE;
tG = sec(graw);
gU = interp1(tP, pU, sec(graw), 'linear', 'extrap');   % draw GPS fixes at flight altitude
gpsErr = hypot(gN - interp1(tP, pN, tG, 'linear', 'extrap'), ...
               gE - interp1(tP, pE, tG, 'linear', 'extrap'));

% Flight mode / armed state (PX4 custom_mode main mode in bits 16-23)
tH    = sec(hb);
armed = bitand(double(hb.base_mode), 128) > 0;
mainM = bitand(bitshift(double(hb.custom_mode), -16), 255);
modeNames = containers.Map(num2cell(1:8), ...
    {'MANUAL','ALTCTL','POSCTL','AUTO','ACRO','OFFBOARD','STABILIZED','RATTITUDE'});
modeColors = containers.Map(num2cell(1:8), ...
    {[.6 .6 .6],[.95 .75 .3],[.35 .7 .4],[.4 .55 .9],[.8 .4 .8],[.3 .6 .95],[.75 .75 .75],[.6 .6 .6]});

% Battery
tS   = sec(sys);
batt = double(sys.battery_remaining);
volt = double(sys.voltage_battery)/1000;

% EKF horizontal position test ratio
tE    = sec(est);
ratio = double(est.pos_horiz_ratio);

% Status text
tX   = sec(stxt);
txt  = strings(height(stxt), 1);
for k = 1:height(stxt)
    txt(k) = strtrim(string(stxt.text{k}));
end
sev  = double(stxt.severity);

%% Figure layout
% Hidden figure: frames are captured with print, and nobody can close it mid-render.
fig = figure('Name', 'Sample_Log playback', 'Color', 'w', 'Visible', 'off', ...
    'Units', 'pixels', 'Position', [40 40 1600 900], 'Resize', 'off');
ax3 = axes(fig, 'Units', 'normalized', 'Position', [0.04 0.07 0.55 0.78]);
axA = axes(fig, 'Units', 'normalized', 'Position', [0.65 0.62 0.32 0.22]);
axV = axes(fig, 'Units', 'normalized', 'Position', [0.65 0.35 0.32 0.20]);
axG = axes(fig, 'Units', 'normalized', 'Position', [0.65 0.08 0.32 0.20]);

banner = annotation(fig, 'textbox', [0.02 0.90 0.96 0.08], 'EdgeColor', 'none', ...
    'FontName', 'Consolas', 'FontSize', 15, 'FontWeight', 'bold', ...
    'VerticalAlignment', 'middle', 'Interpreter', 'none');
evBox = annotation(fig, 'textbox', [0.02 0.74 0.30 0.12], 'EdgeColor', [.8 .8 .8], ...
    'BackgroundColor', [1 1 1 0.85], 'FontName', 'Consolas', 'FontSize', 10, ...
    'VerticalAlignment', 'top', 'Interpreter', 'none');

% --- 3D scene
hold(ax3, 'on'); grid(ax3, 'on'); axis(ax3, 'equal');
xlabel(ax3, 'East [m]'); ylabel(ax3, 'North [m]'); zlabel(ax3, 'Up [m]');
pad = 10;
xl = [min([pE; gE(tG > tStart & tG < tEnd)]) - pad, max([pE; gE(tG > tStart & tG < tEnd)]) + pad];
yl = [min([pN; gN(tG > tStart & tG < tEnd)]) - pad, max([pN; gN(tG > tStart & tG < tEnd)]) + pad];
zl = [0, max(pU) + 5];
xlim(ax3, xl); ylim(ax3, yl); zlim(ax3, zl);
view(ax3, -35, 28);
patch(ax3, xl([1 2 2 1]), yl([1 1 2 2]), [0 0 0 0], [.93 .95 .9], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');
plot3(ax3, pE, pN, pU, '-', 'Color', [.85 .85 .85], 'LineWidth', 1, 'DisplayName', 'Full EKF track');
hTrail  = plot3(ax3, nan, nan, nan, '-', 'Color', [.1 .35 .8], 'LineWidth', 2.2, 'DisplayName', 'EKF track (flown)');
hShadow = plot3(ax3, nan, nan, nan, '-', 'Color', [.4 .4 .4], 'LineWidth', 1, 'DisplayName', 'Ground shadow');
hDrop   = plot3(ax3, nan, nan, nan, ':', 'Color', [.4 .4 .4], 'HandleVisibility', 'off');
hGps    = scatter3(ax3, nan, nan, nan, 14, [.85 .2 .15], 'filled', ...
    'MarkerFaceAlpha', 0.45, 'DisplayName', 'Raw GPS fixes');
hGpsNow = plot3(ax3, nan, nan, nan, 'o', 'MarkerSize', 10, 'LineWidth', 2, ...
    'Color', [.85 .2 .15], 'HandleVisibility', 'off');
hGpsLnk = plot3(ax3, nan, nan, nan, '--', 'Color', [.85 .2 .15], 'LineWidth', 1.2, 'HandleVisibility', 'off');
plot3(ax3, 0, 0, 0, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [1 .8 0], ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Home / takeoff');
[~, kLand] = min(abs(tP - 168.7));
plot3(ax3, pE(kLand), pN(kLand), 0, 'v', 'MarkerSize', 11, 'MarkerFaceColor', [.2 .2 .2], ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Landing point');
lgd = legend(ax3, 'Location', 'northeast', 'FontSize', 9);
lgd.Title.String = sprintf('3D playback, %dx speed (UAV not to scale)', speedUp);

% UAV mesh in an hgtransform
tr = stlread(which('multirotor.stl'));
V  = tr.Points;  V(:,3) = V(:,3) - mean(V(:,3));
V  = V * (meshSpan / (max(V(:,1)) - min(V(:,1))));
hgUav = hgtransform('Parent', ax3);
patch('Parent', hgUav, 'Faces', tr.ConnectivityList, 'Vertices', V, ...
    'FaceColor', [.15 .15 .2], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.5, 'HandleVisibility', 'off');
% heading arrow (body forward) for readability
plot3(hgUav, [0 meshSpan*0.9], [0 0], [0 0], '-', 'Color', [.95 .3 .1], 'LineWidth', 3, 'HandleVisibility', 'off');
light(ax3, 'Position', [0 0 1], 'Style', 'infinite');

% --- Time-series panels
modeBands(axA, tH, mainM, modeColors, [-1 max(pU) + 3]);
plot(axA, tP, pU, 'k', 'LineWidth', 1.2);
ylabel(axA, 'Altitude [m]'); title(axA, 'Altitude (bands = flight mode)');
iMode = find(diff(mainM) ~= 0 & tH(2:end) > tStart) + 1;
evT = [tH(iMode)' 167.3];
evL = [arrayfun(@(m) modeNames(m), mainM(iMode)', 'UniformOutput', false), {'KILL SW'}];
xline(axA, evT, ':', evL, 'FontSize', 8, 'LabelOrientation', 'aligned', 'Color', [.3 .3 .3]);

modeBands(axV, tH, mainM, modeColors, [0 max(vH) + 1]);
plot(axV, tP, vH, 'k', 'LineWidth', 1.2);
ylabel(axV, 'Ground speed [m/s]'); title(axV, 'Horizontal speed');

hold(axG, 'on'); grid(axG, 'on'); box(axG, 'on');
xregion(axG, 79, 137, 'FaceColor', [1 .85 .85], 'FaceAlpha', 0.6);
yyaxis(axG, 'left');
plot(axG, tE, ratio, '-o', 'Color', [.7 .1 .1], 'MarkerSize', 3, 'LineWidth', 1.2);
yline(axG, 1, '--', 'reject threshold', 'Color', [.7 .1 .1], 'FontSize', 8);
ylabel(axG, 'EKF pos-H test ratio'); axG.YColor = [.7 .1 .1];
yyaxis(axG, 'right');
plot(axG, tG, gpsErr, '-', 'Color', [.2 .4 .8], 'LineWidth', 1);
ylabel(axG, '|GPS - EKF| [m]'); axG.YColor = [.2 .4 .8];
xlabel(axG, 'Log time [s]'); title(axG, 'GPS health  (shaded = GPS glitch)');

linkaxes([axA axV axG], 'x');
xlim(axA, [tStart tEnd]);
cur = gobjects(3, 1);
for k = 1:3
    a = [axA axV axG];
    cur(k) = xline(a(k), tStart, '-', 'Color', [0 0 0], 'LineWidth', 1.8, 'HandleVisibility', 'off');
end

%% Render
S = struct('tP', tP, 'pE', pE, 'pN', pN, 'pU', pU, 'vH', vH, 'tA', tA, 'eulA', eulA, ...
    'tG', tG, 'gE', gE, 'gN', gN, 'gU', gU, 'gpsErr', gpsErr, 'tH', tH, 'armed', armed, ...
    'mainM', mainM, 'tS', tS, 'batt', batt, 'volt', volt, 'tX', tX, 'sev', sev, ...
    'tStart', tStart, 'trailGps', trailGps);
S.txt = txt;  S.modeNames = modeNames;
H = struct('hTrail', hTrail, 'hShadow', hShadow, 'hDrop', hDrop, 'hGps', hGps, ...
    'hGpsNow', hGpsNow, 'hGpsLnk', hGpsLnk, 'hgUav', hgUav, 'banner', banner, 'evBox', evBox);

tFrames = tStart : speedUp/fps : tEnd;
if exist('previewOnly', 'var') && previewOnly
    % Quick check: render just the preview still, no video.
    drawState(previewT, S, H, cur);  drawnow;
    imwrite(print(fig, '-RGBImage', '-r0'), prevFile);
    fprintf('Preview saved to %s\n', prevFile);
    return
end
vw = VideoWriter(videoFile, 'MPEG-4');
vw.FrameRate = fps;  vw.Quality = 92;
open(vw);
fprintf('Rendering %d frames...\n', numel(tFrames));
gpsLatched = false;
for f = 1:numel(tFrames)
    t = tFrames(f);
    drawState(t, S, H, cur);
    drawnow;
    img = print(fig, '-RGBImage', '-r0');
    if f == 1, frameSz = 2*floor([size(img, 1) size(img, 2)]/2); end
    img = fitFrame(img, frameSz);                % print size can wobble by a pixel
    writeVideo(vw, img);
    if ~gpsLatched && t >= previewT
        imwrite(img, prevFile);  gpsLatched = true;
    end
    if mod(f, 150) == 0, fprintf('  %d / %d\n', f, numel(tFrames)); end
end
close(vw);
fprintf('Saved %s (%.1f s of video)\n', videoFile, numel(tFrames)/fps);

%% ---------- frame drawing
function drawState(t, S, H, cur)
% Update every animated graphics object for log time t.
    k  = find(S.tP <= t, 1, 'last');  if isempty(k), k = 1; end
    k0 = find(S.tP >= S.tStart, 1);
    e = S.pE(k); n = S.pN(k); u = S.pU(k);
    set(H.hTrail,  'XData', S.pE(k0:k), 'YData', S.pN(k0:k), 'ZData', S.pU(k0:k));
    set(H.hShadow, 'XData', S.pE(k0:k), 'YData', S.pN(k0:k), 'ZData', zeros(k-k0+1, 1));
    set(H.hDrop,   'XData', [e e], 'YData', [n n], 'ZData', [0 u]);

    % Attitude: NED yaw/pitch/roll -> ENU rotation of an FLU body
    ea  = interp1(S.tA, S.eulA, t, 'linear', 'extrap');
    Rned = eul2rotm(ea, 'ZYX');                      % body(FRD) -> NED
    T   = [0 1 0; 1 0 0; 0 0 -1];                    % NED <-> ENU
    B   = diag([1 -1 -1]);                           % FLU <-> FRD
    R   = T * Rned * B;
    M   = eye(4);  M(1:3, 1:3) = R;  M(1:3, 4) = [e; n; u];
    H.hgUav.Matrix = M;

    % Raw GPS
    gi = find(S.tG <= t & S.tG >= S.tStart);
    if S.trailGps && ~isempty(gi)
        set(H.hGps, 'XData', S.gE(gi), 'YData', S.gN(gi), 'ZData', S.gU(gi));
        g = gi(end);
        set(H.hGpsNow, 'XData', S.gE(g), 'YData', S.gN(g), 'ZData', u);
        if S.gpsErr(g) > 5
            set(H.hGpsLnk, 'XData', [e S.gE(g)], 'YData', [n S.gN(g)], 'ZData', [u u]);
        else
            set(H.hGpsLnk, 'XData', nan, 'YData', nan, 'ZData', nan);
        end
        err = S.gpsErr(g);
    else
        err = NaN;
    end

    % Mode / armed / battery
    h  = find(S.tH <= t, 1, 'last');  if isempty(h), h = 1; end
    mm = S.mainM(h);
    if isKey(S.modeNames, mm), mName = S.modeNames(mm); else, mName = sprintf('MODE %d', mm); end
    if S.armed(h), aStr = 'ARMED'; else, aStr = 'DISARMED'; end
    b  = find(S.tS <= t, 1, 'last');  if isempty(b), b = 1; end
    gpsFlag = '';
    if err > 5, gpsFlag = sprintf('   GPS GLITCH: %4.1f m off', err); end
    H.banner.String = sprintf('t = %6.1f s   %-10s %-9s  Alt %5.1f m   GS %4.1f m/s   Batt %3d%% %5.2f V%s', ...
        t, mName, aStr, u, S.vH(k), S.batt(b), S.volt(b), gpsFlag);
    if err > 5, H.banner.Color = [.75 0 0]; else, H.banner.Color = [0 0 0]; end

    % Latest autopilot messages
    xi = find(S.tX <= t);
    xi = xi(max(1, end-3):end);
    lines = strings(numel(xi), 1);
    for i = 1:numel(xi)
        tag = '';  if S.sev(xi(i)) <= 3, tag = '!! '; end
        lines(i) = sprintf('%6.1f s  %s%s', S.tX(xi(i)), tag, S.txt(xi(i)));
    end
    H.evBox.String = ["Autopilot messages:"; lines];

    for c = 1:numel(cur), cur(c).Value = t; end
end

%% ---------- helpers
function T = readAll(tl, name, sysID)
% Read one message type, merging the MAVLink v1 and v2 rows readmsg returns.
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

function out = fitFrame(img, sz)
% Crop or white-pad an RGB image to exactly sz = [rows cols].
    out = 255 * ones(sz(1), sz(2), 3, 'uint8');
    r = min(sz(1), size(img, 1));  c = min(sz(2), size(img, 2));
    out(1:r, 1:c, :) = img(1:r, 1:c, :);
end

function [dN, dE] = geo2ne(lat, lon, lat0, lon0)
% Small-area flat-earth conversion to north/east metres.
    R = 6378137;
    dN = deg2rad(lat - lat0) * R;
    dE = deg2rad(lon - lon0) * R .* cosd(lat0);
end

function e = unwrapEul(e)
% Unwrap angle columns so interpolation does not jump across +/-pi.
    e = unwrap(e);
end

function modeBands(ax, tH, mainM, colors, yl)
% Background colour bands for each flight-mode segment.
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    edges = [1; find(diff(mainM) ~= 0) + 1; numel(mainM) + 1];
    for i = 1:numel(edges) - 1
        a = tH(edges(i));
        if edges(i+1) <= numel(tH), b = tH(edges(i+1)); else, b = tH(end); end
        m = mainM(edges(i));
        if isKey(colors, m), c = colors(m); else, c = [.8 .8 .8]; end
        patch(ax, [a b b a], yl([1 1 2 2]), c, 'EdgeColor', 'none', 'FaceAlpha', 0.25, ...
            'HandleVisibility', 'off');
    end
    ylim(ax, yl);
end
