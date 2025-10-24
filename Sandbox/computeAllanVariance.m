
% % %Select a .csv file
[filename, pathname] = uigetfile('*.csv', 'Select a CSV file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

% Import the data
% filepath = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\imuHist_imu2_static_raw_20Hz_20250317_0909.csv';
% filepath ='C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\imuHist_imu2_static_raw_20Hz_20250317_0951-1014-1029.csv';
T = readtable(filepath); % Use readtable(filepath) if the CSV has headers

% Drop columns that are entirely zero
zeroCols = all(table2array(T) == 0, 1);   % logical mask of zero-only columns
T(:, zeroCols) = [];

% Now extract acceleration samples
ax = [];
ay = [];
az = [];
all_dts = [];

for k = 1:height(T)
    sc = T.scale(k);
    
    % Pick remaining columns by name
    xvals = table2array(T(k, startsWith(T.Properties.VariableNames,'x')));
    yvals = table2array(T(k, startsWith(T.Properties.VariableNames,'y')));
    zvals = table2array(T(k, startsWith(T.Properties.VariableNames,'z')));

    % Determine how many samples are valid (non-NaN or non-empty)
    nValid = nnz(~isnan(xvals));  % assumes zeros are possible real data

    % Keep only valid portion
    xvals = xvals(1:nValid);
    yvals = yvals(1:nValid);
    zvals = zvals(1:nValid);
    
    % Convert to SI units
    ax = [ax; double(xvals(:)) * sc];
    ay = [ay; double(yvals(:)) * sc];
    az = [az; double(zvals(:)) * sc];

    % Store per-sample dt from this packet (us / nValid, in seconds)
    all_dts = [all_dts; (T.dt(k)/nValid) * 1e-6 * ones(nValid,1)];

end

rawGyroData = [ax ay az];

%sanity check
pd_x = fitdist(ax,'Normal');
pd_y = fitdist(ay,'Normal');
pd_z = fitdist(az,'Normal');

%Get sample rate
avg_dt = mean(all_dts);
Fs = 1 / avg_dt;

%accelData = rawdata()

%from MATLAB
% imu_calibData = load('C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\imuReadings_static_74000.mat');
% rawdata = imu_calibData.imuMsgLog;
% rawAccelData = imu_calibData.imuMsgLog(:, 4:6);


% if size(rawdata, 2) > 3
%     rawAccelData = rawdata(:,2:4);
% else
%     rawAccelData = rawdata;
% end
%Fs = 16; %frequency of accel data


%% Do allan variance for acceleration

%calAccelData = (imuCorrect(rawAccelData'))';
[avar,tau] = allanvar(rawGyroData,'octave',Fs);
adev = sqrt(avar); % Allan deviation

figure();
loglog(tau,avar)
xlabel('\tau')
ylabel('\sigma^2(\tau)')
title('Allan Variance')
grid on
%legend('x', 'y', 'z');

%% Extract Noise Parameters

% (1) Velocity/Angle Random Walk (VRW/ARW) -> Find Slope -0.5 Region
tau_index = find(tau >= 1, 1); % Locate τ = 1s
VRW_x = adev(tau_index, 1) * sqrt(2); % Convert to standard unit
VRW_y = adev(tau_index, 2) * sqrt(2); % Convert to standard unit
VRW_z = adev(tau_index, 3) * sqrt(2); % Convert to standard unit

% (2) Bias Instability (BI) -> Find the Minimum Deviation
% in x
[BI_x, bi_x_idx] = min(adev(:,1));
BI_x_tau = tau(bi_x_idx); % Time at which Bias Instability occurs
BI_x_hz = BI_x/(0.664*sqrt(BI_x_tau));
%in y
[BI_y, bi_y_idx] = min(adev(:,2));
BI_y_tau = tau(bi_y_idx); % Time at which Bias Instability occurs
BI_y_hz = BI_y/(0.664*sqrt(BI_y_tau));
%in z
[BI_z, bi_z_idx] = min(adev(:,3));
BI_z_tau = tau(bi_z_idx); % Time at which Bias Instability occurs
BI_z_hz = BI_z/(0.664*sqrt(BI_z_tau));

% (3) Rate Random Walk (RRW) -> Find Slope +0.5 Region

%for x
slope05_x_idx = find(diff(log10(adev(:,1))) ./ diff(log10(tau)) >= 0.5, 1);
if ~isempty(slope05_x_idx)
    RRW_x = adev(slope05_x_idx,1) / sqrt(3); % Convert to standard unit
    RRW_x_tau = tau(slope05_x_idx);
    RRW_x_hz = RRW_x/sqrt(3*RRW_x_tau);
else
    RRW_x = NaN; % No +0.5 slope detected
    RRW_x_tau = NaN;
    RRW_x_hz = NaN;
end

%for y
slope05_y_idx = find(diff(log10(adev(:,2))) ./ diff(log10(tau)) >= 0.5, 1);
if ~isempty(slope05_y_idx)
    RRW_y = adev(slope05_y_idx,2) / sqrt(3); % Convert to standard unit
    RRW_y_tau = tau(slope05_y_idx);
    RRW_y_hz = RRW_y/sqrt(3*RRW_y_tau);
else
    RRW_y = NaN; % No +0.5 slope detected
    RRW_y_tau = NaN;
    RRW_y_hz = NaN;
end

%for z
slope05_z_idx = find(diff(log10(adev(:,3))) ./ diff(log10(tau)) >= 0.5, 1);
if ~isempty(slope05_z_idx)
    RRW_z = adev(slope05_z_idx, 3) / sqrt(3); % Convert to standard unit
    RRW_z_tau = tau(slope05_z_idx);
    RRW_z_hz = RRW_z/sqrt(3*RRW_z_tau);
else
    RRW_z = NaN; % No +0.5 slope detected
    RRW_z_tau = NaN;
    RRW_z_hz = NaN;
end


%% Display Results
fprintf('IMU Noise Parameters:\n');
fprintf('----------------------\n');
fprintf('Velocity Random Walk (VRW): %.3e %.3f %.3g units/sqrt(Hz)\n', VRW_x, VRW_y, VRW_z);
fprintf('Bias Instability (BI) occurs at τ = %.3e , %.3f , %.3g , sec: %.3e , %.3f , %.3g units\n', BI_x_tau, BI_y_tau, BI_z_tau, BI_x, BI_y, BI_z);
fprintf('Bias Instability in m/s^3/sqrt(Hz): %.3e , %.3f , %.3g \n', BI_x_hz, BI_y_hz, BI_z_hz);
fprintf('Rate Random Walk (RRW) occurs at τ = %.3e , %.3f , %.3g , sec: %.3e , %.3f , %.3g units\n', RRW_x_tau, RRW_y_tau, RRW_z_tau, RRW_x, RRW_y, RRW_z);
fprintf('Rate Random Walk in m/s^3 / sqrt(Hz): %.3e , %.3f , %.3g \n', RRW_x_hz, RRW_y_hz, RRW_z_hz);

% function [allanPar, figHandle] = getAllanVar(rawData)
% 
%     allanPar.vrw = 
% 
% 
% end