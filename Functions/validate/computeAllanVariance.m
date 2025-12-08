% 
% % Select a .csv file
% [filename, pathname] = uigetfile('*.csv', 'Select a CSV file to import');
% if isequal(filename,0)
%     disp('User canceled file selection.');
%     return;
% end
% filepath = fullfile(pathname, filename);
% 
% % Parse FIFO log (fast path)
% [rawGyroData, Fs, stats] = parseFifoLog(filepath);
% 
% % Sanity check (retain previous behaviour for convenience)
% pd_x = stats.pd_x;
% pd_y = stats.pd_y;
% pd_z = stats.pd_z;
% 

%% Use a .mat
rawGyroData = lowRateData;
Fs = 250;

% if size(rawdata, 2) > 3
%     rawAccelData = rawdata(:,2:4);
% else
%     rawAccelData = rawdata;
% end
%Fs = 16; %frequency of accel data


%% Do Allan variance / deviation

% allanvar returns variance; Allan deviation is often easier to interpret.
[avar, tau] = allanvar(rawGyroData, 'octave', Fs);
adev = sqrt(avar); % Allan deviation

% Compute local log-log slopes for diagnostic purposes (used for both
% plotting guides and parameter extraction).
logTau = log10(tau(:));
logAdev_x = log10(adev(:,1));
logAdev_y = log10(adev(:,2));
logAdev_z = log10(adev(:,3));

slope_x = diff(logAdev_x) ./ diff(logTau);
slope_y = diff(logAdev_y) ./ diff(logTau);
slope_z = diff(logAdev_z) ./ diff(logTau);

% Plot Allan deviation for each axis
figObj = figure();
loglog(tau, adev(:,1),  'LineWidth', 1.5, Color='#FFB14E');
hold on;
loglog(tau, adev(:,2),  'LineWidth', 1.5, Color='#EA5F94');
hold on;
loglog(tau, adev(:,3), 'LineWidth', 1.5, Color='#0000E3');
xlabel('$\mathbf{\tau [s]}$')
ylabel('$\mathbf{\sigma(\tau)}$')
title('Allan Deviation (IMU)')
grid on

lgdEntries = {'x', 'y', 'z'};

hold on;

% Add reference slope lines only where appropriate regions exist
% Use simple slope-based detection similar to the parameter extraction.
target_rw  = -0.5;
target_bi  = 0.0;
target_rrw = 0.5;
tol_plot   = 0.15;

has_rw  = any(abs(slope_x - target_rw)  <= tol_plot) || ...
          any(abs(slope_y - target_rw)  <= tol_plot) || ...
          any(abs(slope_z - target_rw)  <= tol_plot);
has_bi  = any(abs(slope_x - target_bi)  <= tol_plot) || ...
          any(abs(slope_y - target_bi)  <= tol_plot) || ...
          any(abs(slope_z - target_bi)  <= tol_plot);
has_rrw = any(abs(slope_x - target_rrw) <= tol_plot) || ...
          any(abs(slope_y - target_rrw) <= tol_plot) || ...
          any(abs(slope_z - target_rrw) <= tol_plot);

% Pick a reference point near the middle of tau range
midIdx = round(numel(tau)/2);
tau0   = tau(midIdx);
refVal = adev(midIdx,1);

if has_rw
    ref_rw = refVal * (tau./tau0).^(-0.5);
    loglog(tau, ref_rw, 'k--', 'LineWidth', 1);
    lgdEntries{end+1} = '-0.5 slope';
end

if has_bi
    ref_bi = refVal * ones(size(tau));
    loglog(tau, ref_bi, 'k-.', 'LineWidth', 1);
    lgdEntries{end+1} = '0 slope';
end

if has_rrw
    ref_rrw = refVal * (tau./tau0).^(0.5);
    loglog(tau, ref_rrw, 'k:', 'LineWidth', 1);
    lgdEntries{end+1} = '+0.5 slope';
end

legend(lgdEntries, 'Location', 'southwest');

formatFigForLatex_v2(figObj);

hold off;

%% Extract Noise Parameters

% (1) Velocity/Angle Random Walk (VRW/ARW) -> evaluate near tau = 1 s
tau_index = find(tau >= 1, 1); % first tau >= 1s
if isempty(tau_index)
    tau_index = numel(tau); % fall back to largest tau available
end

VRW_x = adev(tau_index, 1) * sqrt(2);
VRW_y = adev(tau_index, 2) * sqrt(2);
VRW_z = adev(tau_index, 3) * sqrt(2);

VRW_tau = tau(tau_index);

% approximate slope at VRW point
idx_slope = max(tau_index-1, 1);
VRW_slope_x = slope_x(min(idx_slope, numel(slope_x)));
VRW_slope_y = slope_y(min(idx_slope, numel(slope_y)));
VRW_slope_z = slope_z(min(idx_slope, numel(slope_z)));

% (2) Bias Instability (BI) -> minimum Allan deviation
[BI_x, bi_x_idx] = min(adev(:,1));
BI_x_tau = tau(bi_x_idx);
BI_x_hz = BI_x/(0.664*sqrt(BI_x_tau));

[BI_y, bi_y_idx] = min(adev(:,2));
BI_y_tau = tau(bi_y_idx);
BI_y_hz = BI_y/(0.664*sqrt(BI_y_tau));

[BI_z, bi_z_idx] = min(adev(:,3));
BI_z_tau = tau(bi_z_idx);
BI_z_hz = BI_z/(0.664*sqrt(BI_z_tau));

% slope at BI points (should be ~0)
BI_slope_x = slope_x(min(bi_x_idx, numel(slope_x)));
BI_slope_y = slope_y(min(bi_y_idx, numel(slope_y)));
BI_slope_z = slope_z(min(bi_z_idx, numel(slope_z)));

% (3) Rate Random Walk (RRW) -> look for +0.5 slope region
targetSlope = 0.5;
tolerance   = 0.1;

% for x
idx_rrw_x = find(abs(slope_x - targetSlope) <= tolerance, 1, 'last');
if ~isempty(idx_rrw_x)
    RRW_x = adev(idx_rrw_x+1,1) / sqrt(3);
    RRW_x_tau = tau(idx_rrw_x+1);
    RRW_x_hz = RRW_x/sqrt(3*RRW_x_tau);
    RRW_slope_x = slope_x(idx_rrw_x);
else
    RRW_x = NaN;
    RRW_x_tau = NaN;
    RRW_x_hz = NaN;
    RRW_slope_x = NaN;
end

% for y
idx_rrw_y = find(abs(slope_y - targetSlope) <= tolerance, 1, 'last');
if ~isempty(idx_rrw_y)
    RRW_y = adev(idx_rrw_y+1,2) / sqrt(3);
    RRW_y_tau = tau(idx_rrw_y+1);
    RRW_y_hz = RRW_y/sqrt(3*RRW_y_tau);
    RRW_slope_y = slope_y(idx_rrw_y);
else
    RRW_y = NaN;
    RRW_y_tau = NaN;
    RRW_y_hz = NaN;
    RRW_slope_y = NaN;
end

% for z
idx_rrw_z = find(abs(slope_z - targetSlope) <= tolerance, 1, 'last');
if ~isempty(idx_rrw_z)
    RRW_z = adev(idx_rrw_z+1,3) / sqrt(3);
    RRW_z_tau = tau(idx_rrw_z+1);
    RRW_z_hz = RRW_z/sqrt(3*RRW_z_tau);
    RRW_slope_z = slope_z(idx_rrw_z);
else
    RRW_z = NaN;
    RRW_z_tau = NaN;
    RRW_z_hz = NaN;
    RRW_slope_z = NaN;
end

% Package outputs in a struct for downstream use
allanPar = struct();
allanPar.tau  = tau;
allanPar.adev = adev;

allanPar.VRW.value = [VRW_x; VRW_y; VRW_z];
allanPar.VRW.tau   = VRW_tau;
allanPar.VRW.slope = [VRW_slope_x; VRW_slope_y; VRW_slope_z];

allanPar.BI.value      = [BI_x; BI_y; BI_z];
allanPar.BI.tau        = [BI_x_tau; BI_y_tau; BI_z_tau];
allanPar.BI.equiv_hz   = [BI_x_hz; BI_y_hz; BI_z_hz];
allanPar.BI.slope      = [BI_slope_x; BI_slope_y; BI_slope_z];

allanPar.RRW.value     = [RRW_x; RRW_y; RRW_z];
allanPar.RRW.tau       = [RRW_x_tau; RRW_y_tau; RRW_z_tau];
allanPar.RRW.equiv_hz  = [RRW_x_hz; RRW_y_hz; RRW_z_hz];
allanPar.RRW.slope     = [RRW_slope_x; RRW_slope_y; RRW_slope_z];


%% Display Results
fprintf('IMU Noise Parameters (per-axis: x, y, z):\n');
fprintf('-----------------------------------------\n');
fprintf('VRW [units/sqrt(Hz)] at tau = %.3g s:   %.3e  %.3e  %.3e\n', ...
        VRW_tau, VRW_x, VRW_y, VRW_z);
fprintf('  local slope near VRW (expect -0.5):   %.2f   %.2f   %.2f\n', ...
        VRW_slope_x, VRW_slope_y, VRW_slope_z);

fprintf('BI Allan deviation (min sigma):         %.3e  %.3e  %.3e\n', ...
        BI_x, BI_y, BI_z);
fprintf('  BI occurs at tau [s]:                 %.3g   %.3g   %.3g\n', ...
        BI_x_tau, BI_y_tau, BI_z_tau);
fprintf('  BI equiv [units/sqrt(Hz)]:            %.3e  %.3e  %.3e\n', ...
        BI_x_hz, BI_y_hz, BI_z_hz);
fprintf('  local slope at BI (expect ~0):        %.2f   %.2f   %.2f\n', ...
        BI_slope_x, BI_slope_y, BI_slope_z);

fprintf('RRW [units/s/sqrt(Hz)]:                 %.3e  %.3e  %.3e\n', ...
        RRW_x_hz, RRW_y_hz, RRW_z_hz);
fprintf('  RRW identified at tau [s]:            %.3g   %.3g   %.3g\n', ...
        RRW_x_tau, RRW_y_tau, RRW_z_tau);
fprintf('  local slope at RRW (expect +0.5):     %.2f   %.2f   %.2f\n', ...
        RRW_slope_x, RRW_slope_y, RRW_slope_z);

% Optional: add numeric summary to the current plot
% Uncomment the line below if you want the text box on every run.
%addAllanResultsToPlot(allanPar, 'Location', 'northwest');

% function [allanPar, figHandle] = getAllanVar(rawData)
% 
%     allanPar.vrw = 
% 
% 
% end