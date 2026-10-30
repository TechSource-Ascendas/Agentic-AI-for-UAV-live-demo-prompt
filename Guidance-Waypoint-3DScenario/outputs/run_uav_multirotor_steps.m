%% UAV multirotor Guidance Model: individual control-input step responses
% Coordinate frame: NED [north east down].
% Attitude order: EulerZYX = [yaw pitch roll].

scriptDir = fileparts(mfilename('fullpath'));
mdl = 'uav_multirotor_step_response';
mdlFile = fullfile(scriptDir, [mdl '.slx']);

caseNames = ["Roll", "Pitch", "YawRate", "Thrust"];
stepTime = 1.0;
stopTime = 8.0;
hoverThrust = 9.81;

rollFinal = [deg2rad(5), 0, 0, 0];
pitchFinal = [0, deg2rad(5), 0, 0];
yawRateFinal = [0, 0, deg2rad(15), 0];
thrustFinal = [hoverThrust, hoverThrust, hoverThrust, hoverThrust + 1.0];

in(4,1) = Simulink.SimulationInput(mdlFile);
for k = 1:4
    in(k) = Simulink.SimulationInput(mdlFile);
    in(k) = in(k).setModelParameter('StopTime', num2str(stopTime));
    in(k) = in(k).setVariable('StepTime', stepTime, Workspace=mdl);
    in(k) = in(k).setVariable('RollBefore', 0, Workspace=mdl);
    in(k) = in(k).setVariable('PitchBefore', 0, Workspace=mdl);
    in(k) = in(k).setVariable('YawRateBefore', 0, Workspace=mdl);
    in(k) = in(k).setVariable('ThrustBefore', hoverThrust, Workspace=mdl);
    in(k) = in(k).setVariable('RollAfter', rollFinal(k), Workspace=mdl);
    in(k) = in(k).setVariable('PitchAfter', pitchFinal(k), Workspace=mdl);
    in(k) = in(k).setVariable('YawRateAfter', yawRateFinal(k), Workspace=mdl);
    in(k) = in(k).setVariable('ThrustAfter', thrustFinal(k), Workspace=mdl);
end

wallClock = tic;
out = sim(in, 'UseFastRestart', 'on');
elapsedWallTime = toc(wallClock);

results = struct([]);
finalValue = zeros(4,1);
peakValue = zeros(4,1);
riseTime = zeros(4,1);
settlingTime = zeros(4,1);
overshoot = zeros(4,1);

for k = 1:4
    results(k).Case = caseNames(k);
    results(k).WorldPosition = out(k).WorldPosition;
    results(k).WorldVelocity = out(k).WorldVelocity;
    results(k).EulerZYX = out(k).EulerZYX;
    results(k).BodyAngularRateRPY = out(k).BodyAngularRateRPY;
    results(k).Thrust = out(k).ThrustResponse;

    switch k
        case 1
            primaryTs = out(k).EulerZYX;
            primary = rad2deg(squeeze(primaryTs.Data(3,1,:)));
            target = 5;
            baseline = 0;
            results(k).PrimarySignal = "Roll angle (deg)";
        case 2
            primaryTs = out(k).EulerZYX;
            primary = rad2deg(squeeze(primaryTs.Data(2,1,:)));
            target = 5;
            baseline = 0;
            results(k).PrimarySignal = "Pitch angle (deg)";
        case 3
            primaryTs = out(k).BodyAngularRateRPY;
            primary = rad2deg(squeeze(primaryTs.Data(3,1,:)));
            target = 15;
            baseline = 0;
            results(k).PrimarySignal = "Yaw rate (deg/s)";
        otherwise
            primaryTs = out(k).ThrustResponse;
            primary = squeeze(primaryTs.Data(1,1,:));
            target = hoverThrust + 1;
            baseline = hoverThrust;
            results(k).PrimarySignal = "Thrust (N)";
    end

    t = primaryTs.Time(:);
    primary = squeeze(primary);
    primary = primary(:);
    info = stepinfo(primary - baseline, t, target - baseline);
    finalValue(k) = primary(end);
    peakValue(k) = max(primary);
    riseTime(k) = info.RiseTime;
    settlingTime(k) = info.SettlingTime;
    overshoot(k) = info.Overshoot;
    results(k).Time = t;
    results(k).PrimaryResponse = primary;
end

summary = table(caseNames', finalValue, peakValue, riseTime, settlingTime, overshoot, ...
    'VariableNames', {'Case','FinalValue','PeakValue','RiseTime_s','SettlingTime_s','Overshoot_pct'});

fig = figure('Color','w','Position',[100 100 1200 900]);
tiledlayout(4,2,'TileSpacing','compact','Padding','compact');
for k = 1:4
    nexttile;
    plot(results(k).Time, results(k).PrimaryResponse, 'LineWidth', 1.5);
    grid on;
    xline(stepTime, '--k', 'Step');
    title(caseNames(k) + " input: primary response");
    ylabel(results(k).PrimarySignal);
    xlabel('Time (s)');

    nexttile;
    pos = results(k).WorldPosition;
    plot(pos.Time, squeeze(pos.Data)', 'LineWidth', 1.2);
    grid on;
    title(caseNames(k) + " input: NED position");
    ylabel('Position (m)');
    xlabel('Time (s)');
    legend('North','East','Down','Location','best');
end
sgtitle('Reduced-order multirotor Guidance Model step responses');

plotFile = fullfile(scriptDir, 'uav_multirotor_step_responses.png');
matFile = fullfile(scriptDir, 'uav_multirotor_step_results.mat');
csvFile = fullfile(scriptDir, 'uav_multirotor_step_summary.csv');
exportgraphics(fig, plotFile, 'Resolution', 180);
save(matFile, 'results', 'summary', 'elapsedWallTime', 'stepTime', 'stopTime');
writetable(summary, csvFile);

disp(summary);
fprintf('Completed four %.1f-second simulations in %.3f seconds wall time.\n', stopTime, elapsedWallTime);
fprintf('Saved plot: %s\n', plotFile);
fprintf('Saved results: %s\n', matFile);
