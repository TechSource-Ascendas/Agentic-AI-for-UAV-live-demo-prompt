function scene = create_simple_uav_scenario()
%CREATE_SIMPLE_UAV_SCENARIO Recreate the handle-based scene for each run.
% Geometry inputs to addMesh use ENU horizontal coordinates [east north].
% Platform motion is supplied later in NED [north east down] by Simulink.

scene = uavScenario(UpdateRate=10, StopTime=40, ...
    ReferenceLocation=[0 0 0]);

% Flat ground covering the 5 m square waypoint mission.
groundENU = [-5 -5; 10 -5; 10 10; -5 10];
addMesh(scene,"polygon",{groundENU,[-0.1 0]},[0.35 0.35 0.35]);

% A small decorative cylinder outside the flight path.
addMesh(scene,"cylinder",{[8 8 0.5],[0 3]},[0.2 0.65 0.25]);

uav = uavPlatform("UAV",scene,ReferenceFrame="NED", ...
    InitialPosition=[0 0 0],InitialOrientation=[1 0 0 0]);
updateMesh(uav,"quadrotor",{0.6},[0 0.4470 0.7410], ...
    [0 0 0],[1 0 0 0]);
end
