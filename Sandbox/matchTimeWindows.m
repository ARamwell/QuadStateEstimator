
%% Find all subfolders
parentFolder = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\';
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
%for k=2:numberOfSubFolders %Ignore 1st entry (parent folder)

    %% Actually do correction
    %currentFolder = string(listOfFolderNames(k));
    currentFolder = fullfile('C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\arb_3');
    
    clear imuMsgLog timestamps_imu mocapMsgLog timestamps_mocap imageStream;
  
    

    if ~contains(currentFolder, 'calib')
        imgFolder = fullfile(currentFolder);
        imuLogFolder = fullfile(currentFolder, '/imuReadings.mat');
        mocapLogFolder = fullfile(currentFolder, '/mocapLog.mat');


        %% import images
        
        imageStream = struct();
        imageStream.img = [];
        imageStream.time = createArray(1, 0, 'datetime');
        imageStream.time = datetime(imageStream.time, 'Format', 'yyyyMMdd_HHmmss_SSS');
        [imageStream.img, imageStream.time] = imgFuncs.importImageSeq(imgFolder, 1);
        startTime_img = imageStream.time(1,1);
        endTime_img = imageStream.time(1,end);
    
        %% import imu data
        imuLog = load(imuLogFolder);
        [closestDiff, ind_start] = min(abs(   milliseconds(imuLog.timestamps_imu(:,1) - startTime_img)    ));
        [closestDiff, ind_end] = min(abs(   milliseconds(imuLog.timestamps_imu(:,1) - endTime_img)    ));
        imuMsgLog = imuLog.imuMsgLog(ind_start:ind_end,:);
        timestamps_imu = imuLog.timestamps_imu(ind_start:ind_end,:);
        save(fullfile(currentFolder, '/imuReadings_sync'), 'timestamps_imu', 'imuMsgLog'); %save chopped data

        %% import mocap data
        mocapData = load(mocapLogFolder);
        [closestDiff, ind_start] = min(abs(   milliseconds(mocapData.timestamps_mocap(:,1) - startTime_img)    ));
        [closestDiff, ind_end] = min(abs(   milliseconds(mocapData.timestamps_mocap(:,1) - endTime_img)    ));
        mocapMsgLog = mocapData.mocapMsgLog(ind_start:ind_end,:);
        timestamps_mocap = mocapData.timestamps_mocap(ind_start:ind_end, :);
        save(fullfile(currentFolder, '/mocapLog_sync'), 'timestamps_mocap', 'mocapMsgLog'); %save chopped data
    
    end

%end