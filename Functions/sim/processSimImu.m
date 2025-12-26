function [imu] = processSimImu(out)
%PROCESSSIMIMU Summary of this function goes here
%   Detailed explanation goes here
    imu.rawdata = out.IMU.signals.values';
    imu.time = out.IMU.time';
end

