delete(gcp('nocreate')); %close parallel pool if open

%Set up variables
saveFolder = fullfile('.', '/Tests/RobMech/Dynamic/TestSeries_2');
numFrames = 10; %Approx 10 Hz
numImu = numFrames*2.5; %Approx 20 Hz
numMocap = numFrames*3; %?
url = 'http://192.168.0.106/capture';

%set up nodes
mocapNode = ros2node("mocap_node", 11);
imuNode = ros2node("ekf_node", 1);


%set up parallel threads
parpool(1);
%f1 = parfeval(@run, 0, 'streamCamSimple_WithTimestamp', url, numFrames, saveFolder);
f1 = parfeval(@run, 2, 'imuRosListener', imuNode, numImu, saveFolder, 'imuReadings');
%f2 = parfeval(@run, 0, 'streamMocap', mocapNode, numMocap, saveFolder);

pause(10);

streamCamSimple_WithTimestamp(url, numFrames, saveFolder);
%imuRosListener(numImu, saveFolder, 'imuReadings')

