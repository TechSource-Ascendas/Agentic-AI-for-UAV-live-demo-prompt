function [scene,building] = create_obstructed_uav_scenario()
%CREATE_OBSTRUCTED_UAV_SCENARIO Add a square building across the first leg.
% Building geometry uses ENU coordinates [east north up].

scene = create_simple_uav_scenario();

building.EastBounds = [-0.75 0.75];
building.NorthBounds = [1.75 3.25];
building.UpBounds = [0 4];

footprintENU = [building.EastBounds(1) building.NorthBounds(1); ...
                building.EastBounds(2) building.NorthBounds(1); ...
                building.EastBounds(2) building.NorthBounds(2); ...
                building.EastBounds(1) building.NorthBounds(2)];
addMesh(scene,"polygon",{footprintENU,building.UpBounds},[0.8 0.2 0.1]);
end
