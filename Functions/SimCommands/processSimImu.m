function [imu] = processSimImu(imuOut)
%PROCESSSIMIMU Summary of this function goes here
%   Detailed explanation goes here
    imu.rawdata = imuOut.signals.values';
    imu.time = imuOut.time';
end

