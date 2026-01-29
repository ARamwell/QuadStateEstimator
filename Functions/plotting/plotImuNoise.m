% Script to plot IMU readings over time and calculate Gaussian fit

%% %Select a .mat file
[filename, pathname] = uigetfile('*.mat, *.csv', 'Select a fifo file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

%% Import data
%imuData = load(filepath);
[imuData , Fs, ts_high, stats] = parseFifoLog(filepath);

if isstruct(imuData)
    fields = fieldnames(imuData);
    imuData = getfield(imuData, fields{1});
end

if (size(imuData, 1) > 6)
    imuData = imuData';
end

 %% Do temperature compensation

poly = [-0.09 5.82 66.12 0;
        0.02 -3.19 -5.70 0;
        1.09 -8.17 -318.91 0] *10e-05; %accelerometer

% poly = [-0.09 -1.32 2.46 0;
%         0.12 2.54 -11.65 0;
%         0.18 0.29 -17.51 0] *10e-06; %gyroscope

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
tempOffset_highrate = temp_highrate-25;

biasCorr_x = polyval(poly(1,:), tempOffset_highrate);
biasCorr_y = polyval(poly(2,:), tempOffset_highrate);
biasCorr_z = polyval(poly(3,:), tempOffset_highrate);

biasCorr = [biasCorr_x; biasCorr_y; biasCorr_z];

%% or, instead, only consider the portion that has a stable temperature of 40
idx_stable = find(temp_highrate==40, 1, 'first');
imuData_stable = imuData(:, idx_stable:end);

imuData = imuData_stable;

%% Optionally, correct reference values using scale factor
accelParamFile = './Resources/Calibrations/accel_accel0_20251211_allan_stable.mat';
gyroParamFile = './Resources/Calibrations/gyro_gyro0_20251211_allan.mat';

accelCalib = load(accelParamFile);
gyroCalib = load(gyroParamFile);

invScale = inv(accelCalib.scale);


%% Calc error
ref = invScale*[0 0 -9.795]';

for i = 1:size(imuData, 2)
    imuErr(:,i) = imuData(:,i)-ref; 
end

%% Init plot
figObj = figure();
hold on;
tileLayoutObj = tiledlayout(figObj, 3, 1, "TileSpacing", "tight");
xlabel(tileLayoutObj, '$\mathbf{sample\ number}$', 'fontweight', 'bold',  'Interpreter', 'latex');
ylabel(tileLayoutObj,'$\mathbf{raw\ measurement\ (rad/s)}$', 'fontweight', 'bold', 'Interpreter', 'latex');


%% Do plotting
titles = {'$x$-channel', '$y$-channel', '$z$-channel'};
for i = 1:3 %for each channel 
    
    channelData = imuData(i,:);%imuData(i, :) - biasCorr(i,:);
    channelData_clean = rmoutliers(channelData, 'mean', 'ThresholdFactor', 6);  

    nexttile;
    plotGaussNoise(channelData_clean, figObj, '', '', titles(i));
    set(gca, 'YMinorTick', 'on', 'TickDir', 'out');
    
end

formatFigForLatex(figObj);




