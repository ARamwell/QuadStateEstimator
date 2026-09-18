function [temp] = getPx4SensorTemperature(inputArg1,inputArg2)
%GETPX4SENSORTEMPERATURE Summary of this function goes here
%   Detailed explanation goes here

px4Data = readtable('C:\Users\Alyssa\Documents\QGroundControl\Logs\testLog_temperatureCalib\log_22_2025-12-11-02-43-36_sensor_accel_0');

temp = px4Data.temperature;

end

