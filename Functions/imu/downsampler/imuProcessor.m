function [outAccel, outGyro] = imuProcessor(rawAccel, rawGyro, dt, calibTrue, downsample, coning, sculling, resetFlag, aBias, aK, gBias, gK)
%IMUPROCESSOR Summary of this function goes here
%   Detailed explanation goes here


if calibTrue == 1
    corrAccel = applyCalib(rawAccel, aBias, aK);
    corrGyro = applyCalib(rawGyro, gBias, gK);
else
    corrAccel=rawAccel;
    corrGyro=rawGyro;
end

if downsample == 1 
    [outAccel, outGyro] = accumImu(corrAccel, corrGyro, dt, coning, sculling, resetFlag);
else
    outAccel = corrAccel;
    outGyro = corrGyro;
end


end

