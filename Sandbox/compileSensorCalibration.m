%POPULATE MAT FILE WITH SENSOR CALIBRATION PARAMETERS

%specify save destination
accelName = 'accel0_20250808_gauss_nobias';
gyroName = 'gyro0_nobias';
camName = 'esp32cam_320p';
targetFolder = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Resources\Calibrations';


%% ACCELEROMETER
accel.description = 'Pixhawk 6C Accel0 - full rate, only gaussian fit';
%accel.turnOnBias = [-0.1550; 0.0886; -0.0226] + [-0.173; -0.164; 0.329];
accel.turnOnBias = [0 0 0];
accel.scale = [1 0 0; 0 1 0; 0 0 1];
accel.vrw = [0.015893  0.0315858  0.017725]; %white noise on acceleration signal
accel.bi = [0 0 0]; %bias isntability
accel.rrw =[0 0 0]; %rate random walk
accel.max = 78.5;
accel.res =0.001;
accel.skew = inv(accel.scale)*100;
accel.simBias =  ((accel.skew /100) * accel.turnOnBias')';

fullFileName = strcat(fullfile(targetFolder), '/', 'accel_', accelName, '.mat');
save(fullFileName, '-struct', "accel")

%% GYROSCOPE
%gyro.turnOnBias = [0.0016; 0.0028; 0.0015] + [0; 0; 0];
gyro.turnOnBias =[0 0 0];
gyro.scale = [1 0 0; 0 1 0; 0 0 1];
gyro.arw = [1.037e-03 1e-03 1.29e-03]; %white noise on gyro signal
gyro.bi = [0 0 0]; %bias isntability
gyro.rrw =[0 0 0]; %rate random walk
gyro.max = 34.9;
gyro.res =0.0001;
gyro.skew = inv(gyro.scale)*100;
gyro.simBias =  ((gyro.skew /100) * gyro.turnOnBias')';

fullFileName = strcat(fullfile(targetFolder), '/', 'gyro_', gyroName, '.mat');
save(fullFileName, '-struct', "gyro")


%% SET CAMERA PARAMETERS
camPin.K = [458.8803 0 235.5162; 0 458.9989 167.8936; 0 0 1];
    %K = [458.944297528687 0 249.584321044399; 0 459.543641247509 172.625660963493; 0 0 1];%esp32 svga
    %K = [462.0327 0 241.8730; 0 462.4926 166.7321; 0 0 1];
camPin.radialDistortion = [-0.0571 0.1245];
camPin.tangentialDistortion =[-0.0035 -0.0064];
camPin.imageSize = [320 480];

fullFileName = strcat(fullfile(targetFolder), '/', 'camPin_', camName, '.mat');
save(fullFileName, '-struct', "camPin")


%calib.camFish = cameraParams;
camFish.MappingCoefficients = [-410.004498270331	0.000595185438226373	-6.40563444907148e-06	1.41682024130720e-08];
camFish.ImageSize =  [320 480];
camFish.DistortionCenter = [234.349743656515	221.809375095898];
camFish.StretchMatrix = [0.991340278392322	-0.258367605394120; 0.255792105017201	1];

fullFileName = strcat(fullfile(targetFolder), '/', 'camFish_', camName, '.mat');
save(fullFileName, '-struct', "camFish")
