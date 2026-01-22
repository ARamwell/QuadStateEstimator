% poly = [-0.09 5.82 66.12 0;
%         0.02 -3.19 -5.70 0;
%         1.09 -8.17 -318.91 0] *10e-05; %accelerometer

poly = [-0.09 -1.32 2.46 0;
        0.12 2.54 -11.65 0;
        0.18 0.29 -17.51 0] *10e-06; %gyroscope



%% Import highrate data
% Select a .csv file
[filename, pathname] = uigetfile('*.csv', 'Select the high-rate CSV file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

% Parse FIFO log (fast path)
[rawImuData, Fs, ts_high, stats] = parseFifoLog(filepath);
%throw away the first few measurements
%rawGyroData = rawGyroData(Fs:end, :);
%Fs=2000;% for gyro

% Sanity check (retain previous behaviour for convenience)
pd_x = stats.pd_x;
pd_y = stats.pd_y;
pd_z = stats.pd_z;


 %% Use a .mat
 % rawGyroData = imuData;
 % Fs = 2000;
% 
% % if size(rawdata, 2) > 3
% %     rawAccelData = rawdata(:,2:4);
% % else
% %     rawAccelData = rawdata;
% % end
% %Fs = 16; %frequency of accel data

%% Do temperature compensation
% Get low rate data
% Get sensor accel file
[filename, pathname] = uigetfile('*.csv', 'Select sensor_accel_0 file');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
lowratefilepath = fullfile(pathname, filename);

px4LowRateData = readtable(lowratefilepath); 
temp_low = px4LowRateData.temperature;
idx = ~isnan(temp_low);
temp_low = temp_low(idx);
ts_low = px4LowRateData.timestamp_sample;
ts_low = ts_low(idx);

temp_highrate = interp1(double(ts_low), double(temp_low), double(ts_high), 'linear', 'extrap');
% tempOffset_highrate = temp_highrate-25;
% 
% biasCorr_x = polyval(poly(1,:), tempOffset_highrate);
% biasCorr_y = polyval(poly(2,:), tempOffset_highrate);
% biasCorr_z = polyval(poly(3,:), tempOffset_highrate);
% 
% biasCorr = [biasCorr_x; biasCorr_y; biasCorr_z]';
% %rawImuData = 
%% or, instead, only consider the portion that has a stable temperature of 40
idx_stable = find(temp_highrate==40, 1, 'first');
imuData_stable = rawImuData(idx_stable:end, :);

%% Custom bins
num_bins = 50; % Desired number of bins
% Define the range for 'm' (from 1 sample up to a reasonable maximum, 
% typically half the total number of samples)
m_min = 1;
m_max = floor((size(rawImuData, 1) - 1) / 4);
m = floor(logspace(log10(m_min), log10(m_max), num_bins));
% Ensure 'm' values are unique and ascending integers
m = unique(m); 

%% Do Allan variance / deviation

% allanvar returns variance; Allan deviation is often easier to interpret.
[avar, tau] = allanvar(rawImuData, 'octave', Fs);
%[avar, tau] = allanvar(rawImuData, m, Fs);
%[avar, tau] = allanvar(imuData_stable, 'octave', Fs);
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

% % Add reference slope lines only where appropriate regions exist.
% % Anchor each guide at the best-matching slope point so they appear in the
% % correct region of tau.
% target_rw  = -0.5;
% target_bi  = 0.0;
% target_rrw = 0.5;
% tol_plot   = 0.15;
% 
% % Helper to pick the best anchor (tau, sigma) across all axes for a target slope
% pickAnchor = @(target) deal([], [], []);
% bestDiff = inf; bestIdx = []; bestAxis = 0;
% % VRW / BI / RRW handled below via target-specific loops
% 
% % Build an inline function to select anchor for a given target slope
% function [tau_anchor, sig_anchor, found] = selectAnchor(target, tol, ...
%     slope_x, slope_y, slope_z, tau, adev)
%     candidates = [
%         min(abs(slope_x - target)), ...
%         min(abs(slope_y - target)), ...
%         min(abs(slope_z - target))];
%     [bestDiff, bestAxis] = min(candidates);
%     if isempty(bestDiff) || bestDiff > tol
%         tau_anchor = [];
%         sig_anchor = [];
%         found = false;
%         return;
%     end
%     switch bestAxis
%         case 1
%             idx = find(abs(slope_x - target) == min(abs(slope_x - target)), 1, 'last');
%             sig_anchor = adev(idx+1, 1);
%         case 2
%             idx = find(abs(slope_y - target) == min(abs(slope_y - target)), 1, 'last');
%             sig_anchor = adev(idx+1, 2);
%         case 3
%             idx = find(abs(slope_z - target) == min(abs(slope_z - target)), 1, 'last');
%             sig_anchor = adev(idx+1, 3);
%     end
%     tau_anchor = tau(idx+1);
%     found = true;
% end
% 
% % Plot each guide only if a suitable anchor exists
% [tau_rw, sig_rw, has_rw]   = selectAnchor(target_rw,  tol_plot, slope_x, slope_y, slope_z, tau, adev);
% [tau_bi, sig_bi, has_bi]   = selectAnchor(target_bi,  tol_plot, slope_x, slope_y, slope_z, tau, adev);
% [tau_rrw, sig_rrw, has_rrw] = selectAnchor(target_rrw, tol_plot, slope_x, slope_y, slope_z, tau, adev);
% 
% if has_rw
%     ref_rw = sig_rw * (tau./tau_rw).^(target_rw);
%     loglog(tau, ref_rw, 'k--', 'LineWidth', 1);
%     lgdEntries{end+1} = '-0.5 slope';
% end
% 
% if has_bi
%     ref_bi = sig_bi * ones(size(tau));
%     loglog(tau, ref_bi, 'k-.', 'LineWidth', 1);
%     lgdEntries{end+1} = '0 slope';
% end
% 
% if has_rrw
%     ref_rrw = sig_rrw * (tau./tau_rrw).^(target_rrw);
%     loglog(tau, ref_rrw, 'k:', 'LineWidth', 1);
%     lgdEntries{end+1} = '+0.5 slope';
% end
% 
% legend(lgdEntries, 'Location', 'southwest');
% 
% formatFigForLatex_v2(figObj);
% 
% hold off;

%% Extract Noise Parameters

% (1) Velocity/Angle Random Walk (VRW/ARW) -> evaluate near tau = 1 s
tau_index = find(tau >= 1, 1); % first tau >= 1s
if isempty(tau_index)
    tau_index = numel(tau); % fall back to largest tau available
end

VRW_x = adev(tau_index, 1);% * sqrt(2);
VRW_y = adev(tau_index, 2);% * sqrt(2);
VRW_z = adev(tau_index, 3);% * sqrt(2);

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
idx_rrw_x = find(abs(slope_x - targetSlope) <= tolerance, 1, 'first');
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
idx_rrw_y = find(abs(slope_y - targetSlope) <= tolerance, 1, 'first');
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
idx_rrw_z = find(abs(slope_z - targetSlope) <= tolerance, 1, 'first');
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
fprintf('IMU Noise Parameters (per-axis: x, y, z) at %.2f Hz:\n', Fs);
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