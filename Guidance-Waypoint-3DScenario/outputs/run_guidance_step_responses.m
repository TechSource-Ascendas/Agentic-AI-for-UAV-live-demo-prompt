%% Multirotor Guidance Model - step response per control input
% Runs UAV_GuidanceStepResponse.slx once per control-bus element.
% Each run steps ONE element at t = tStep while the others stay at trim
% (Roll = Pitch = YawRate = 0, Thrust = hover thrust = m*g).
% Frame: NED. WorldPosition = [north east down], altitude = -down.
% EulerZYX = [yaw pitch roll].
% Outputs (next to this script): stage1_step_responses.png,
%   stage1_step_results.mat, stage1_step_summary.csv

here = fileparts(mfilename("fullpath")); addpath(here);
% Long clone paths break Windows builds (260-char limit): keep Simulink cache/codegen short
if strlength(here) > 80
    Simulink.fileGenControl("set", "CacheFolder", fullfile(tempdir, "uav_slprj"), ...
        "CodeGenFolder", fullfile(tempdir, "uav_slprj"), "createDir", true);
end
mdl = "UAV_GuidanceStepResponse";
open_system(mdl);

% Step amplitudes (added on top of trim)
cases = struct( ...
    "name",  {"Hover",  "Roll",  "Pitch", "YawRate", "Thrust"}, ...
    "var",   {"",       "dRoll", "dPitch","dYawRate","dThrust"}, ...
    "amp",   {0,        0.1,     0.1,     0.5,       0.1}, ...    % rad, rad, rad/s, N
    "unit",  {"",       "rad",   "rad",   "rad/s",   "N"});

stepVars = ["dRoll" "dPitch" "dYawRate" "dThrust"];
in = Simulink.SimulationInput.empty;
for k = 1:numel(cases)
    in(k) = Simulink.SimulationInput(mdl);
    for v = stepVars
        in(k) = in(k).setVariable(v, 0, "Workspace", mdl);
    end
    if cases(k).var ~= ""
        in(k) = in(k).setVariable(cases(k).var, cases(k).amp, "Workspace", mdl);
    end
end
out = sim(in, "ShowProgress", "off");

%% Extract results
res = struct();
for k = 1:numel(cases)
    % yout order follows root outports 1..5 (logged names carry <...>)
    y = out(k).yout;
    r.t     = y{1}.Values.Time;
    r.pos   = squeeze(y{1}.Values.Data);                       % WorldPosition [N E D]
    r.vel   = squeeze(y{2}.Values.Data);                       % WorldVelocity
    r.eul   = squeeze(y{3}.Values.Data);                       % EulerZYX [yaw pitch roll]
    r.rate  = squeeze(y{4}.Values.Data);                       % BodyAngularRateRPY
    r.thr   = squeeze(y{5}.Values.Data);                       % Thrust
    fix = @(A) reshape(A, [], numel(r.t)).';                 % -> N-by-k
    r.pos = fix(r.pos); r.vel = fix(r.vel); r.eul = fix(r.eul); r.rate = fix(r.rate); r.thr = r.thr(:);
    % Commanded inputs (logged on the Control bus input lines)
    L = out(k).logsout;
    for nm = ["Roll" "Pitch" "YawRate" "Thrust"]
        s = L.get(nm).Values;
        r.cmd.(nm) = struct("t", s.Time, "u", s.Data(:));
    end
    res.(cases(k).name) = r;
end

%% Plot: commanded input vs. vehicle response for each channel
% Top row: full run. Bottom row: zoom around the step to show the lag.
tStep = getVariable(get_param(mdl, "ModelWorkspace"), "tStep");
figure("Name", "Guidance Model step responses", "Color", "w", "Position", [100 100 1500 650]);
tl = tiledlayout(2, 4, "TileSpacing", "compact");
title(tl, "Multirotor Guidance Model - command vs. response (step at t = " + tStep + " s)");

plotSpec = { ...   % case, response, label, unit, zoom window after step [s]
    "Roll",    @(r) r.eul(:,3),  "Roll angle",  "rad",   0.3;
    "Pitch",   @(r) r.eul(:,2),  "Pitch angle", "rad",   0.3;
    "YawRate", @(r) r.rate(:,3), "Yaw rate r",  "rad/s", 0.01;
    "Thrust",  @(r) r.thr,       "Thrust",      "N",     0.005};

ax = gobjects(2, 4);
for row = 1:2
    for k = 1:4
        [nm, f, lbl, unit, win] = plotSpec{k,:};
        r = res.(nm); c = r.cmd.(nm);
        ax(row,k) = nexttile((row-1)*4 + k);
        stairs(c.t, c.u, "k--", "LineWidth", 1.2); hold on
        plot(r.t, f(r), "LineWidth", 1.5); hold off; grid on
        ylabel(lbl + " [" + unit + "]"); xlabel("Time [s]");
        if row == 1
            title(nm + " step");
            legend("Command", "Response", "Location", "southeast");
        else
            xlim(tStep + [-0.2 1] * win);
            title(nm + " (zoom)");
        end
    end
end

%% Step metrics for the commanded channel
fprintf("\n%-8s %-10s %-10s %-10s %-10s %-10s\n", "Input", "Final", "Target", "Rise[s]", "Settle[s]", "OS[%]");
chan = {"Roll", @(r) r.eul(:,3); "Pitch", @(r) r.eul(:,2); "YawRate", @(r) r.rate(:,3); "Thrust", @(r) r.thr};
summary = table('Size', [4 7], 'VariableTypes', ["string" "string" "double" "double" "double" "double" "double"], ...
    'VariableNames', ["Input" "Unit" "StepAmplitude" "FinalValue" "RiseTime_s" "SettlingTime_s" "Overshoot_pct"]);
for k = 1:size(chan,1)
    nm = chan{k,1}; r = res.(nm); sig = chan{k,2}(r);
    idx = r.t >= tStep;
    c = cases(strcmp([cases.name], nm));
    y0 = sig(find(idx,1)); target = y0 + c.amp;
    S = stepinfo(sig(idx) - y0, r.t(idx) - tStep, c.amp);
    fprintf("%-8s %-10.4f %-10.4f %-10.4f %-10.4f %-10.2f\n", nm, sig(end), target, S.RiseTime, S.SettlingTime, S.Overshoot);
    summary(k,:) = {nm, c.unit, c.amp, sig(end), S.RiseTime, S.SettlingTime, S.Overshoot};
end
r = res.Hover;
fprintf("\nHover baseline: max |position| = %.2e m, max |velocity| = %.2e m/s\n", max(abs(r.pos), [], "all"), max(abs(r.vel), [], "all"));

%% Save results next to this script
exportgraphics(gcf, fullfile(here, "stage1_step_responses.png"), "Resolution", 110);
save(fullfile(here, "stage1_step_results.mat"), "res", "cases", "summary", "tStep");
writetable(summary, fullfile(here, "stage1_step_summary.csv"));
