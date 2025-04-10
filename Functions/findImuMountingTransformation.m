%Select a .csv file
[filename, pathname] = uigetfile('*.csv', 'Select a CSV file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);
% filepath = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\imuHist_imu2_static_raw_20Hz_20250317_0909.csv';

% Import the data
raw_accel = readmatrix(filepath); % Use readtable(filepath) if the CSV has headers

% calibrate
for row=1:size(raw_accel, 1)
    raw_accel_column = raw_accel(row, :)';
    calib_accel_column = imuCorrect(raw_accel_column);
    calib_accel(row, :) = calib_accel_column';
end
% calib_accel = raw_accel;

means = mean(calib_accel, 1);

%Find rotation

%Begin with axis-angle representation
g_actual = g;
g_meas = means';

g_a_hat = g_actual/norm(g_actual);
g_m_hat = g_meas/norm(g_meas);

rotAngle = acos(dot(g_m_hat, g_a_hat));
rotAxis = (1/sin(rotAngle)) * cross(g_m_hat, g_a_hat);

rotAxis_SS = [0 -rotAxis(3) rotAxis(2);
              rotAxis(3) 0 -rotAxis(1);
              -rotAxis(2) rotAxis(1) 0];

R_imu2rq = eye(3) + sin(rotAngle) * rotAxis_SS + (1-cos(rotAngle)) * (rotAxis_SS)^2;

%Rotate measurements
for row=1:size(calib_accel, 1)
    accel_corr(row, :) = transpose( R_imu2rq * (calib_accel(row, :))' );
end 
