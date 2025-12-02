function [estset] = estGetSettings()
%ESTGETSETTINGS Summary of this function goes here
%   Detailed explanation goes here

    estset.runP3P = true;
    estset.trackBias = false;
    estset.trapInteg = true;
    estset.adaptive = false;
    estset.alpha = 0.95;
    estset.accelBias = zeros(3,1);
    estset.accelK = eye(3);
    estset.gyroBias = zeros(3,1);
    estset.gyroK = eye(3);
    %estset.Q0=;
    %estset.W0=;
    %estset.x0 = zeros
    

end

