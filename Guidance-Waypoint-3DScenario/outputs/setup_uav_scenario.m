%% Scene for UAV_GuidanceWaypointScenario.slx
% Rerun before EVERY simulation: uavScenario is a handle object and keeps
% its state between runs. The model's InitFcn calls this script.
%
% Frames
%   Scenario / scene meshes : ENU. addMesh polygon vertices are [east north],
%                             height limits are [bottom top] in metres UP.
%   UAV platform and Motion Write input : NED [north east down].
%
% Mission (NED, from the model workspace): climb to 5 m, square 20 m x 20 m
%   WP1 [0 0 -5] -> WP2 [20 0 -5] -> WP3 [20 20 -5] -> WP4 [0 20 -5] -> WP5 [0 0 -5]
% The obstacle sits in the middle of the WP2 -> WP3 leg (north = 20 m, east 0 -> 20 m).

scenarioSampleTime = 0.1;            % s, = Scenario Configuration sample time
lidarSampleTime    = 0.1;            % s, integer multiple of scenarioSampleTime

uavScene = uavScenario("UpdateRate", 1/scenarioSampleTime, "StopTime", 60, ...
    "ReferenceLocation", [0 0 0]);

% Ground plane (ENU), 60 m x 60 m centred on the mission
addMesh(uavScene, "polygon", {[-20 -20; 40 -20; 40 40; -20 40], [-0.5 0]}, [0.6 0.6 0.6]);

% Obstacle: a 4 m x 4 m x 10 m building on the WP2 -> WP3 leg.
% NED footprint: north 18..22 m, east 8..12 m, from ground (0 m) up to 10 m.
obstacle.northLimits = [18 22];
obstacle.eastLimits  = [8 12];
obstacle.upLimits    = [0 10];
obstacleENU = [obstacle.eastLimits(1) obstacle.northLimits(1);   % [east north]
               obstacle.eastLimits(2) obstacle.northLimits(1);
               obstacle.eastLimits(2) obstacle.northLimits(2);
               obstacle.eastLimits(1) obstacle.northLimits(2)];
addMesh(uavScene, "polygon", {obstacleENU, obstacle.upLimits}, [0.85 0.3 0.2]);

% UAV platform, driven by the Guidance Model through Motion Write (NED)
uavPlat = uavPlatform("UAV", uavScene, "ReferenceFrame", "NED", ...
    "InitialPosition", [0 0 0], "InitialOrientation", [1 0 0 0]);
updateMesh(uavPlat, "quadrotor", {1.2}, [0 0 1], eye(4));

% Lidar (unorganized N-by-3 output, ready for an Obstacle Avoidance block later)
lidarModel = uavLidarPointCloudGenerator("UpdateRate", 1/lidarSampleTime, ...
    "MaxRange", 30, "RangeAccuracy", 0.02, ...
    "AzimuthLimits", [-180 180], "AzimuthResolution", 2, ...
    "ElevationLimits", [-20 20], "ElevationResolution", 2, ...
    "HasOrganizedOutput", false, "HasNoise", false);
uavSensor("Lidar", uavPlat, lidarModel, "MountingLocation", [0 0 -0.4]);
% Do not call setup(uavScene): the UAV Scenario Configuration block does it.
