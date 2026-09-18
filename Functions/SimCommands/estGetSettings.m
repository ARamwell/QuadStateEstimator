function [estset] = estGetSettings()
%ESTGETSETTINGS Summary of this function goes here
%   Detailed explanation goes here

    accelParamFile = './Resources/Calibrations/accel_accel0_20251211_allan_stable.mat';
    gyroParamFile = './Resources/Calibrations/gyro_gyro0_20251211_allan.mat';
    accelCalib = load(accelParamFile);
    gyroCalib = load(gyroParamFile);

    estset.runP3P = true;
    estset.runEKF = true;
    estset.trackBias = true;
    estset.trapInteg = false;
    estset.adaptive = false;
    estset.alpha = 0.0;
    estset.accelBias = accelCalib.turnOnBias;%zeros(3,1);
    estset.accelK = accelCalib.scale;%eye(3);
    estset.gyroBias = gyroCalib.turnOnBias;%zeros(3,1);
    estset.gyroK = gyroCalib.scale;%eye(3);
    %estset.Q0=;
    %estset.W0=;
    %estset.x0 = zeros
    

end

