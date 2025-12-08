% % %Select a .csv file
[filename, pathname] = uigetfile('*.mat', 'Select a MAT file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

imuData = load(fullfile('C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3', '\imuReadings_static_15000.mat'));

gyroData = imuData.imuMsgLog(:,1:3);
meanGyro = mean(gyroData, 1);
noiseGyro = std(gyroData, 1);
noiseGyro = var(gyroData, 1);

accelData = imuData.imuMsgLog(:,4:6);
meanAccel = mean(accelData, 1)-[0 0 -9.7952];
noiseAccel = std(accelData, 1);





