%% get highrate data

%Find folder holding ulg CSVs
[filename, pathname] = uigetfile('*.csv', 'Select the high rate acceleration file (sensor_accel_fifo_0');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
highratefilepath = fullfile(pathname, filename);

%Parse FIFO log (fast path)
[rawImuData_highrate, Fs, ts_highrate, stats] = parseFifoLog(highratefilepath);

%% Get low rate data
% Get sensor accel file
[filename, pathname] = uigetfile('*.csv', 'Select sensor_accel_0 file');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
lowratefilepath = fullfile(pathname, filename);

px4LowRateData = readtable(lowratefilepath); 

%% Clean the temperature data
imuT = px4LowRateData.temperature';
imuts_lowrate =px4LowRateData.timestamp_sample';
idx_Tclean = ~isoutlier(imuT, 'mean');
imuRead = [px4LowRateData.x';  px4LowRateData.y'; px4LowRateData.z'];
imuData = [imuT; imuts_lowrate; imuRead];
g = [0 0 -9.7952]; 

%imuData_clean = rmoutliers(imuData, 2,'ThresholdFactor', 5);
px4LowRateData_clean = imuData(:, idx_Tclean);


%% Interpolate temperatures for highrate data
temp_highrate = getHighRateTemp(ts_highrate, px4LowRateData_clean(2,:), px4LowRateData_clean(1,:));

%% Clean the data
%now clean dependent data
px4HighRateData = [temp_highrate; rawImuData_highrate];

idx_clean = ~isoutlier(px4HighRateData(2:4,:)', 'mean','ThresholdFactor', 6)';
%idx_measClean = ~isoutlier(px4LowRateData_clean(2:4,:)', 'mean','ThresholdFactor', 5)';
idx_clean = ~any(idx_clean==0, 1);
%imuData_measClean = rmoutliers(imuData_clean(2:4,:)', 'mean','ThresholdFactor', 5)';
imuData_clean = px4HighRateData(:,idx_clean);
imuT = imuData_clean(1,:);
imuData = imuData_clean(2:4,:);

%% Find polynomial fit for each
[xp, xS] = polyfit(imuT, imuData(1,:), 3);
[yp, yS] = polyfit(imuT, imuData(2,:), 3);
[zp, zS] = polyfit(imuT, imuData(3,:), 3);

%% Plot it
imuTempPoly = [xp; yp; zp];
imuTempPolyStats = [xS; yS; zS];
figObj_imuTemp = plotImuTempFit(imuT, imuData, imuTempPoly);

%% Assume no temperature bias at 25 deg
Tref = 25;

%get 'reference biases'
xb_ref = polyval(xp, 25);
yb_ref = polyval(yp, 25);
zb_ref =polyval(zp, 25);

%% Recalculate polynomial fit
[xp_rel, xS] = polyfit((imuT-Tref), (imuData(1,:)-xb_ref), 3);
[yp_rel, yS] = polyfit((imuT-Tref), (imuData(2,:)-yb_ref), 3);
[zp_rel, zS] = polyfit((imuT-Tref), (imuData(3,:)-zb_ref), 3);

%% Plot results
relT = imuT-Tref;
relImu = imuData - [xb_ref; yb_ref; zb_ref];
relPoly = [xp_rel; yp_rel; zp_rel];

newFigObj = plotImuTempFit(relT, relImu, relPoly);


%% functions

function figObj = plotImuTempFit(imuIndData, imuDepData, imuPolynomials)

figObj = figure();
hold on;
tileLayoutObj = tiledlayout(figObj, 3, 1, "TileSpacing", "tight");
xlabel(tileLayoutObj, '$\mathbf{temperature (^{\circ}C)}$', 'fontweight', 'bold',  'Interpreter', 'latex');
ylabel(tileLayoutObj,'$\mathbf{measurement (m/s^2)}$', 'fontweight', 'bold', 'Interpreter', 'latex');


%% Do plotting
titles = {'x-channel', 'y-channel', 'z-channel'};
for i = 1:3 %for each channel    
    nexttile; 

    %clean and plot raw readings
    channelData = imuDepData(i, :);
    plot(imuIndData, channelData, Color='#FFB14E'); 
    hold on;

    %plot polynomial
    channelPoly = imuPolynomials(i, :);
    depMin = min(imuIndData);
    depMax = max(imuIndData);
    T = depMin:0.5:depMax;
    plot(T, polyval(channelPoly, T), Color= '#0000e3');
    title(titles{i});
   
    
end

formatFigForLatex(figObj);

end

function temp_highrate = getHighRateTemp(ts_highrate, ts_lowrate, temp_lowrate)

    temp_highrate = interp1(double(ts_lowrate), double(temp_lowrate), double(ts_highrate), 'linear', 'extrap');
    

end