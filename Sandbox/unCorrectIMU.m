function [outputArg1,outputArg2] = unCorrectIMU(inputArg1,inputArg2)
%UNCORRECTIMU Summary of this function goes here
%   Detailed explanation goes here

%% Define variables


S_a = diag([0.992 0.989 0.975]);
b_a = [-0.193, -0.115, 0.339]';
b_g = [0 0 0]';
R_imu2q = eul2rotm(deg2rad([-0.441 -3.177 0]), 'XYZ');



%% Find all subfolders
parentFolder = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_2\';
allSubFolders = genpath(parentFolder);
% Parse into a cell array.
remain = allSubFolders;
listOfFolderNames = {};
while true
	[singleSubFolder, remain] = strtok(remain, ';');
	if isempty(singleSubFolder)
		break;
	end
	listOfFolderNames = [listOfFolderNames singleSubFolder];
end
numberOfSubFolders = length(listOfFolderNames);

%% Loop through subfolders
for k=2:numberOfSubFolders %Ignore 1st entry (parent folder)

    %% Actually do correction
    currentFolder = string(listOfFolderNames(k));
    clear imuMsgLog timestamps_imu;

    if ~contains(currentFolder, 'QVGA')
        %currentFolder = "C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_2\StaticLow1";
        imuReadings_calib = load(fullfile(strcat(currentFolder, '\imuReadings.mat')));
        imuMsgLog_calib = imuReadings_calib.imuMsgLog;
        numReadings = size(imuMsgLog_calib, 1); 
        timestamps_imu = imuReadings_calib.timestamps_imu(1:numReadings, :);
        numReadings = size(timestamps_imu, 1); 
        
    
        for t=1:numReadings
            accel_calib_t = imuMsgLog_calib(t,4:6)';
            gyro_calib_t = imuMsgLog_calib(t,1:3)';
    
            accel_raw_t = R_imu2q' * accel_calib_t; %cancel board offsets
            accel_raw_t = inv(S_a) * (accel_raw_t + b_a); %uncalibrate accel
            gyro_raw_t = R_imu2q' * gyro_calib_t; %cancel board offsets
            gyro_raw_t = gyro_raw_t + b_g; %uncalibrate gyro
    
            %build new log
            imuMsgLog(t,:) = [gyro_raw_t' accel_raw_t'];
        end
    
        %% Save to folder
        save(strcat(currentFolder, '/imuReadings_raw'), 'timestamps_imu', 'imuMsgLog');
    end

end

end

