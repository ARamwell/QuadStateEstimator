
%Import .csv data from uncalibrated accelerometer (should be ellipsoid) and
%fit to a sphere.

% Select a .csv file
[filename, pathname] = uigetfile('*.csv', 'Select a CSV file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end

% Import the data
filepath = fullfile(pathname, filename);
data_imu = readmatrix(filepath); % Use readtable(filepath) if the CSV has headers

% Isolate accelerometer values
x = data_imu(:, 4);
y = data_imu(:, 5);
z = data_imu(:, 6);
accel_uncalib = [x, y, z];

%Plot uncalibrated values
figure(1)
plot3(x(:),y(:),z(:),"LineStyle","none","Marker","X","MarkerSize",8)

% Calibrate
[A,b,expmfs] = magcal(accel_uncalib, 'sym'); % Calibration coefficients
expmfs % Display the expected magnetic field strength in uT

% Calibrate the raw magnetometer data
accel_calib = (accel_uncalib - b) * A;

% Scale to desired field strength beta
beta = 9.81; % Example value in µT
for row=1:size(accel_calib,1)
    accel_calib_norms(row) = norm(accel_calib(row, :));
end
accel_calib_mean = mean(accel_calib_norms);
accel_scaled = accel_calib * beta / accel_calib_mean;