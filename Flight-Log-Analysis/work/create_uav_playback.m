%% Create UAV telemetry playback from MAVLink TLOG
% Produces a 4x-speed MP4 containing a 3-D trajectory, vehicle marker,
% telemetry timeline, flight mode, vehicle state, battery, and status events.

clearvars;
close all force;

% Paths are relative to this script so the demo runs from any clone location:
%   Flight-Log-Analysis/work/create_uav_playback.m   (this script)
%   Flight-Log-Analysis/outputs/Sample_Log.tlog      (input)
%   Flight-Log-Analysis/outputs/uav_tlog_playback.mp4 (output)
scriptDir = fileparts(mfilename("fullpath"));
outputDir = fullfile(scriptDir, "..", "outputs");
if ~isfolder(outputDir)
    mkdir(outputDir);
end
logFile = fullfile(outputDir, "Sample_Log.tlog");
outputFile = fullfile(outputDir, "uav_tlog_playback.mp4");

assert(isfile(logFile), "TLOG file was not found: %s", logFile);

dialect = mavlinkdialect("ardupilotmega.xml", 2);
reader = mavlinktlog(logFile, dialect);
messageNames = ["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "ATTITUDE", ...
    "VFR_HUD", "HEARTBEAT", "EXTENDED_SYS_STATE", "STATUSTEXT", ...
    "SYS_STATUS"];
messages = readmsg(reader, MessageName=messageNames, SystemID=1);

localPosition = largestStream(messages, "LOCAL_POSITION_NED");
globalPosition = largestStream(messages, "GLOBAL_POSITION_INT");
attitude = largestStream(messages, "ATTITUDE");
hud = largestStream(messages, "VFR_HUD");
heartbeat = largestStream(messages, "HEARTBEAT");
extendedState = largestStream(messages, "EXTENDED_SYS_STATE");
statusText = largestStream(messages, "STATUSTEXT");
systemStatus = largestStream(messages, "SYS_STATUS");

startTime = reader.StartTime;
endTime = reader.EndTime;
durationSeconds = seconds(endTime - startTime);

% Render at 4x real-time speed. At 15 video frames per second this gives
% one frame for every 0.2667 seconds of source telemetry.
playbackSpeed = 4;
frameRate = 15;
sourceStep = playbackSpeed / frameRate;
playTime = (0:sourceStep:durationSeconds).';

tLocal = seconds(localPosition.Time - startTime);
tGlobal = seconds(globalPosition.Time - startTime);
tAttitude = seconds(attitude.Time - startTime);
tHud = seconds(hud.Time - startTime);
tHeartbeat = seconds(heartbeat.Time - startTime);
tExtended = seconds(extendedState.Time - startTime);
tSystem = seconds(systemStatus.Time - startTime);
tStatus = seconds(statusText.Time - startTime);

% TLOG timestamps are stored at millisecond resolution, so high-rate streams
% can contain multiple packets at the same recorded instant. Keep the latest
% packet at each instant to provide unique interpolation sample points.
[tLocal, localPosition] = uniqueSamples(tLocal, localPosition);
[tGlobal, globalPosition] = uniqueSamples(tGlobal, globalPosition);
[tAttitude, attitude] = uniqueSamples(tAttitude, attitude);
[tHud, hud] = uniqueSamples(tHud, hud);
[tHeartbeat, heartbeat] = uniqueSamples(tHeartbeat, heartbeat);
[tExtended, extendedState] = uniqueSamples(tExtended, extendedState);
[tSystem, systemStatus] = uniqueSamples(tSystem, systemStatus);
[tStatus, statusText] = uniqueSamples(tStatus, statusText);

% Local position is NED. Display altitude upward by negating Down.
north = interp1(tLocal, double(localPosition.x), playTime, "linear", "extrap");
east = interp1(tLocal, double(localPosition.y), playTime, "linear", "extrap");
up = interp1(tLocal, -double(localPosition.z), playTime, "linear", "extrap");
yaw = interp1(tAttitude, unwrap(double(attitude.yaw)), playTime, "linear", "extrap");
relativeAltitude = interp1(tGlobal, double(globalPosition.relative_alt)/1000, ...
    playTime, "linear", "extrap");
groundSpeed = interp1(tHud, double(hud.groundspeed), playTime, "linear", "extrap");
throttle = interp1(tHud, double(hud.throttle), playTime, "previous", "extrap");
voltage = interp1(tSystem, double(systemStatus.voltage_battery)/1000, ...
    playTime, "linear", "extrap");
batteryRemaining = interp1(tSystem, double(systemStatus.battery_remaining), ...
    playTime, "previous", "extrap");

mainModeSamples = bitand(bitshift(uint32(heartbeat.custom_mode), -16), uint32(255));
mainMode = interp1(tHeartbeat, double(mainModeSamples), playTime, "previous", "extrap");
armedSamples = bitand(uint8(heartbeat.base_mode), uint8(128)) ~= 0;
armed = interp1(tHeartbeat, double(armedSamples), playTime, "previous", "extrap") ~= 0;
landedState = interp1(tExtended, double(extendedState.landed_state), ...
    playTime, "previous", "extrap");

% The first vehicle-state packet arrives shortly after the first position
% packet. Extend the nearest valid state across that brief leading interval.
throttle = fillmissing(throttle, "nearest");
batteryRemaining = fillmissing(batteryRemaining, "nearest");
mainMode = fillmissing(mainMode, "nearest");
armed = fillmissing(armed, "nearest");
landedState = fillmissing(landedState, "nearest");

distanceFromStart = hypot(north - north(1), east - east(1));

% Dark presentation-style layout.
background = [0.035 0.055 0.085];
panel = [0.055 0.085 0.125];
gridColor = [0.25 0.34 0.42];
cyan = [0.10 0.82 0.94];
orange = [1.00 0.60 0.16];
green = [0.25 0.90 0.50];
red = [1.00 0.25 0.28];
white = [0.92 0.96 1.00];
muted = [0.60 0.70 0.78];

fig = figure("Color", background, "Position", [50 50 1600 900], ...
    "Visible", "on", "Renderer", "opengl");
layout = tiledlayout(fig, 2, 3, "TileSpacing", "compact", "Padding", "compact");

ax3 = nexttile(layout, [2 2]);
set(ax3, "Color", panel, "XColor", muted, "YColor", muted, "ZColor", muted, ...
    "GridColor", gridColor, "MinorGridColor", gridColor, "FontSize", 11);
hold(ax3, "on");
grid(ax3, "on");
view(ax3, 36, 28);
axis(ax3, "equal");
xlabel(ax3, "North (m)", "Color", white);
ylabel(ax3, "East (m)", "Color", white);
zlabel(ax3, "Up (m)", "Color", white);
title(ax3, "3-D Flight Track", "Color", white, "FontWeight", "bold");

% Full route is shown faintly; completed route is highlighted during playback.
plot3(ax3, north, east, up, "--", "Color", [0.28 0.38 0.45], "LineWidth", 1.1);
completedLine = plot3(ax3, north(1), east(1), up(1), "-", ...
    "Color", cyan, "LineWidth", 2.7);
plot3(ax3, north(1), east(1), up(1), "s", "MarkerSize", 10, ...
    "MarkerFaceColor", green, "MarkerEdgeColor", white);
plot3(ax3, north(end), east(end), up(end), "p", "MarkerSize", 12, ...
    "MarkerFaceColor", orange, "MarkerEdgeColor", white);

% Exaggerated quadrotor arms make orientation visible at this map scale.
armLength = 2.2;
arm1 = plot3(ax3, nan, nan, nan, "-", "Color", white, "LineWidth", 3.5);
arm2 = plot3(ax3, nan, nan, nan, "-", "Color", white, "LineWidth", 3.5);
vehicle = plot3(ax3, north(1), east(1), up(1), "o", "MarkerSize", 8, ...
    "MarkerFaceColor", cyan, "MarkerEdgeColor", white, "LineWidth", 1.2);
headingArrow = quiver3(ax3, north(1), east(1), up(1), 1, 0, 0, 0, ...
    "Color", orange, "LineWidth", 2.4, "MaxHeadSize", 1.3);

padXY = 8;
xlim(ax3, [min(north)-padXY max(north)+padXY]);
ylim(ax3, [min(east)-padXY max(east)+padXY]);
zlim(ax3, [min(-1, min(up)-1) max(up)+4]);

axTelemetry = nexttile(layout, 3);
set(axTelemetry, "Color", panel, "XColor", muted, "YColor", muted, ...
    "GridColor", gridColor, "FontSize", 10);
hold(axTelemetry, "on");
grid(axTelemetry, "on");
yyaxis(axTelemetry, "left");
altitudeLine = plot(axTelemetry, playTime, relativeAltitude, "Color", cyan, "LineWidth", 1.7);
altitudeMarker = plot(axTelemetry, playTime(1), relativeAltitude(1), "o", ...
    "MarkerFaceColor", cyan, "MarkerEdgeColor", white, "MarkerSize", 7);
ylabel(axTelemetry, "Altitude AGL (m)", "Color", cyan);
axTelemetry.YAxis(1).Color = cyan;
yyaxis(axTelemetry, "right");
speedLine = plot(axTelemetry, playTime, groundSpeed, "Color", orange, "LineWidth", 1.5);
speedMarker = plot(axTelemetry, playTime(1), groundSpeed(1), "o", ...
    "MarkerFaceColor", orange, "MarkerEdgeColor", white, "MarkerSize", 7);
ylabel(axTelemetry, "Groundspeed (m/s)", "Color", orange);
axTelemetry.YAxis(2).Color = orange;
xlabel(axTelemetry, "Elapsed source time (s)", "Color", white);
title(axTelemetry, "Flight Telemetry", "Color", white, "FontWeight", "bold");
xlim(axTelemetry, [0 durationSeconds]);
cursor = xline(axTelemetry, 0, "-", "Color", white, "LineWidth", 1.2);
legend(axTelemetry, [altitudeLine speedLine], {"Altitude", "Groundspeed"}, ...
    "TextColor", white, "Color", panel, "EdgeColor", gridColor, "Location", "northwest");

axInfo = nexttile(layout, 6);
set(axInfo, "Color", panel, "XColor", panel, "YColor", panel);
axis(axInfo, [0 1 0 1]);
axis(axInfo, "off");
title(axInfo, "Vehicle State", "Color", white, "FontWeight", "bold");
stateText = text(axInfo, 0.05, 0.93, "", "Color", white, "FontName", "Consolas", ...
    "FontSize", 13, "VerticalAlignment", "top", "Interpreter", "none");
eventText = text(axInfo, 0.05, 0.16, "", "Color", muted, "FontSize", 12, ...
    "FontWeight", "bold", "VerticalAlignment", "top", "Interpreter", "none");

sgtitle(layout, "PX4 UAV TELEMETRY PLAYBACK  |  Sample_Log.tlog", ...
    "Color", white, "FontSize", 18, "FontWeight", "bold");

video = VideoWriter(outputFile, "MPEG-4");
video.FrameRate = frameRate;
video.Quality = 95;
open(video);
cleanupVideo = onCleanup(@() closeVideoSafely(video));

for index = 1:numel(playTime)
    currentNorth = north(index);
    currentEast = east(index);
    currentUp = up(index);
    currentYaw = yaw(index);

    set(completedLine, "XData", north(1:index), "YData", east(1:index), "ZData", up(1:index));
    set(vehicle, "XData", currentNorth, "YData", currentEast, "ZData", currentUp);

    % PX4 yaw is clockwise from North in NED coordinates.
    northDirection = cos(currentYaw);
    eastDirection = sin(currentYaw);
    crossNorth = cos(currentYaw + pi/2);
    crossEast = sin(currentYaw + pi/2);
    set(arm1, "XData", currentNorth + armLength*[-northDirection northDirection], ...
        "YData", currentEast + armLength*[-eastDirection eastDirection], ...
        "ZData", currentUp + [0 0]);
    set(arm2, "XData", currentNorth + armLength*[-crossNorth crossNorth], ...
        "YData", currentEast + armLength*[-crossEast crossEast], ...
        "ZData", currentUp + [0 0]);
    set(headingArrow, "XData", currentNorth, "YData", currentEast, "ZData", currentUp, ...
        "UData", 4*northDirection, "VData", 4*eastDirection, "WData", 0);

    set(altitudeMarker, "XData", playTime(index), "YData", relativeAltitude(index));
    set(speedMarker, "XData", playTime(index), "YData", groundSpeed(index));
    cursor.Value = playTime(index);

    modeName = px4ModeName(mainMode(index));
    armedName = string(ternary(armed(index), "ARMED", "DISARMED"));
    landedName = mavLandedStateName(landedState(index));
    currentClock = startTime + seconds(playTime(index));
    stateText.String = sprintf([ ...
        'Recorded time  %s\n' ...
        'Playback       %.0fx\n\n' ...
        'Mode           %s\n' ...
        'State          %s\n' ...
        'Land state     %s\n\n' ...
        'Altitude       %6.2f m\n' ...
        'Groundspeed    %6.2f m/s\n' ...
        'Distance       %6.1f m\n' ...
        'Throttle       %6.0f %%\n' ...
        'Battery        %6.2f V  (%3.0f%%)'], ...
        char(string(currentClock, "HH:mm:ss.SSS")), playbackSpeed, char(modeName), char(armedName), char(landedName), ...
        relativeAltitude(index), groundSpeed(index), distanceFromStart(index), ...
        throttle(index), voltage(index), batteryRemaining(index));

    statusIndex = find(tStatus <= playTime(index), 1, "last");
    if isempty(statusIndex)
        eventText.String = "STATUS: waiting for vehicle messages";
        eventText.Color = muted;
    else
        statusMessage = string(statusText.text{statusIndex});
        age = playTime(index) - tStatus(statusIndex);
        eventText.String = "STATUS: " + statusMessage;
        if age <= 4 && (contains(lower(statusMessage), "kill") || contains(lower(statusMessage), "failure"))
            eventText.Color = red;
        elseif age <= 4
            eventText.Color = green;
        else
            eventText.Color = muted;
        end
    end

    drawnow;
    capturedFrame = getframe(fig);
    capturedFrame.cdata = imresize(capturedFrame.cdata, [900 1600]);
    writeVideo(video, capturedFrame);
end

close(video);
clear cleanupVideo;
close(fig);

fprintf("Created playback video: %s\n", outputFile);
fprintf("Source duration: %.3f s | Video duration: %.3f s | Frames: %d\n", ...
    durationSeconds, numel(playTime)/frameRate, numel(playTime));

function stream = largestStream(messages, messageName)
rows = find(messages.MessageName == messageName);
assert(~isempty(rows), "Message stream %s is not present in the TLOG.", messageName);
heights = cellfun(@height, messages.Messages(rows));
[~, largestIndex] = max(heights);
stream = sortrows(messages.Messages{rows(largestIndex)});
end

function [uniqueTime, uniqueStream] = uniqueSamples(time, stream)
[uniqueTime, reverseIndices] = unique(flipud(time), "stable");
reverseIndices = height(stream) - reverseIndices + 1;
uniqueTime = flipud(uniqueTime);
reverseIndices = flipud(reverseIndices);
uniqueStream = stream(reverseIndices, :);
end

function name = px4ModeName(mainMode)
names = ["UNKNOWN", "MANUAL", "ALTITUDE", "POSITION", "AUTO", ...
    "ACRO", "OFFBOARD", "STABILIZED", "RATTITUDE"];
index = round(mainMode) + 1;
if index >= 1 && index <= numel(names)
    name = names(index);
else
    name = "PX4 MODE " + string(round(mainMode));
end
end

function name = mavLandedStateName(state)
names = ["UNDEFINED", "ON GROUND", "IN AIR", "TAKEOFF", "LANDING"];
index = round(state) + 1;
if index >= 1 && index <= numel(names)
    name = names(index);
else
    name = "STATE " + string(round(state));
end
end

function value = ternary(condition, trueValue, falseValue)
if condition
    value = trueValue;
else
    value = falseValue;
end
end

function closeVideoSafely(video)
try
    close(video);
catch
end
end
