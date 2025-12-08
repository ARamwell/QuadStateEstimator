% Script to plot IMU readings over time and calculate Gaussian fit

% % %Select a .mat file
[filename, pathname] = uigetfile('*.mat', 'Select a MAT file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

imuData = load(filepath);

if (size(imuData, 2) > 6)
    imuData = imuData';
end

meanGyro = mean(imuData, 1);
noiseGyro = std(imuData, 1);
%noiseGyro = var(gyroData, 1);

figure();
plot(imuData);


%accelData = imuData.imuMsgLog(:,4:6);
%meanAccel = mean(accelData, 1)-[0 0 -9.7952];
%noiseAccel = std(accelData, 1);
