function [Roll, Pitch, YawRate, Thrust] = Position_Yaw_Controller(LookaheadPoint, WorldPosition, WorldVelocity, EulerZYX, DesiredYaw, ctrl)
%#codegen
% Waypoint tracking controller for the multirotor Guidance Model.
% Source of the MATLAB Function block in UAV_GuidanceWaypointFollowing.slx.
% Frame: NED [north east down]. EulerZYX = [yaw pitch roll].
% Small-angle Guidance Model kinematics (consistent with the step tests:
% +roll -> +east, +pitch -> -north):
%   aN = -g*(pitch*cos(yaw) + roll*sin(yaw))
%   aE = -g*(pitch*sin(yaw) - roll*cos(yaw))
%   aD =  g - Thrust*cos(roll)*cos(pitch)/mass

yaw = EulerZYX(1);

% Outer loop: position error -> velocity command (with limits)
vCmd = ctrl.Kp_pos * (LookaheadPoint(:) - WorldPosition(:));
speedXY = norm(vCmd(1:2));
if speedXY > ctrl.vMaxXY
    vCmd(1:2) = vCmd(1:2) * (ctrl.vMaxXY / speedXY);
end
vCmd(3) = min(max(vCmd(3), -ctrl.vMaxZ), ctrl.vMaxZ);

% Inner loop: velocity error -> acceleration command (NED)
aCmd = ctrl.Kv * (vCmd - WorldVelocity(:));

% Rotate horizontal acceleration into the heading frame
aFwd   =  cos(yaw)*aCmd(1) + sin(yaw)*aCmd(2);
aRight = -sin(yaw)*aCmd(1) + cos(yaw)*aCmd(2);

% Acceleration -> attitude and thrust
Pitch = min(max(-aFwd   / ctrl.g, -ctrl.tiltMax), ctrl.tiltMax);
Roll  = min(max( aRight / ctrl.g, -ctrl.tiltMax), ctrl.tiltMax);
Thrust = ctrl.mass * (ctrl.g - aCmd(3)) / (cos(Roll) * cos(Pitch));
Thrust = min(max(Thrust, 0), ctrl.thrustMax);

% Yaw: wrapped heading error -> yaw-rate command
yawErr = atan2(sin(DesiredYaw - yaw), cos(DesiredYaw - yaw));
YawRate = min(max(ctrl.Kyaw * yawErr, -ctrl.yawRateMax), ctrl.yawRateMax);
end
