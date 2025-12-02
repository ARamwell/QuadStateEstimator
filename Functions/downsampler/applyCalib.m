function [calibImuRead] = applyCalib(rawImuRead, turnOnBias, KMatrix)
%APPLYCALIB_ACCEL Summary of this function goes here
%   Expects column vectors

calibImuRead = KMatrix * (rawImuRead - turnOnBias);

end

