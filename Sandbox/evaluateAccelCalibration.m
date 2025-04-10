
%% Import test set - raw accel data

%Select a .csv file
[filename, pathname] = uigetfile('*.csv', 'Select a CSV file to import as test data');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

% Import the data
%filepath = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Testers\imuCalib\imuHist_20250310_1058.csv';
accel_raw = readmatrix(filepath); % Use readtable(filepath) if the CSV has headers


%% Import validation - ground truth accelerations
accel_true = repmat(g', size(accel_raw, 1), 1);


%% Calibrate the raw data
accel_corr = accel_raw;
for row=1:size(accel_raw, 1)
    accel_corr(row, :) = imuCorrect(accel_raw(row, :)');
end

%% Evaluate errors

stdDev = std(accel_corr); %Standard deviation

error_sum = 0;
normError_sum = 0;

for i=1:size(accel_corr, 1)

    accel_true_i = accel_true(i, :);
    accel_corr_i = accel_corr(i, :);

    error_i = norm(accel_true_i - accel_corr_i); %error
    error_sum = error_sum + error_i;

    normError_i = norm(g) - norm(accel_corr_i); %error in the reading norm
    normError_sum = normError_sum + normError_i;

end

error_mean = ( (1/size(accel_corr, 1)) * error_sum );

error_rmse = sqrt( (1/size(accel_corr, 1)) * error_sum ); %root mean square error

normError_rmse = sqrt( (1/size(accel_corr, 1)) * normError_sum ); %root mean square norm error

errorStruct = struct('Std_dev', stdDev, 'RMSE', error_rmse, 'Raw_data', accel_raw, 'Calibrated_data', accel_corr, 'True_data', accel_true);

%% Print out results 
fprintf('  Mean Error (m/s^2): %f\n', error_mean);
fprintf('  Noise Std Dev (m/s^2): [%f, %f, %f]\n', stdDev);
fprintf('  RMSE (m/s^2): %f\n', error_rmse);
fprintf('  Total Norm Error RMSE (m/s^2): %f\n', normError_rmse);

%% Save evaluation

% Ask user if they want to save the file
choice = input('Do you want to save the data? (y/n): ', 's');

if lower(choice) == 'y'
            
    newfilename = input('Enter filename (without .mat extension): ', 's');
    if isempty(newfilename)
        newfilename = strcat(filename, '_errorEvaluation'); % Default name if none provided
    end

    newfilepath = fullfile('.', '/Testers/imuCalib/', newfilename); % Default to current folder

    save(newfilepath, 'errorStruct');
    fprintf('Data saved as %s\n', newfilepath);
else
    fprintf('Data not saved.\n');
end
