%IMU LISTENER

%% LOG DESTINATION
clear imuMsgLog timestamp_imu mocapMsgLog timestamp_mocap
saveFolder = fullfile('.', '/Tests/RobMech/Dynamic');

%% ROS2 INITIALISATIONS
ekfNode = ros2node("ekf_node", 1);
mocapNode = ros2node("mocap_node", 11);

% create subscriber
imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', Reliability="besteffort");
mocapSub = ros2subscriber(mocapNode, '/fakeDrone/pose_stamped', Reliability="besteffort");

%Initialise IMU log
imuMsgLog = [];
timestamp_imu = [];

%Initialise mocap log
mocapMsgLog = [];
timestamp_mocap = [];

%% RECORDING LOOP
disp('Recording IMU and mocap data...');
for i=1:2000
    
    %receive IMU message
    newImuMsg = receive(imuSub, 1);
    timestamp_imu = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');

    %receive mocap message
    newMocapMsg = receive(mocapSub, 1);
    timestamp_mocap = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');   

    % Update IMU log   
    imuMsgLog(i,:) = [(newImuMsg .gyro_rad)' (newImuMsg .accelerometer_m_s2)'];
    timestamps_imu(i,:) = timestamp_imu;

    % Update mocap log
    mocap_t = [newMocapMsg.pose.position.x newMocapMsg.pose.position.y newMocapMsg.pose.position.z];
    mocap_q = [newMocapMsg.pose.orientation.w newMocapMsg.pose.orientation.x  newMocapMsg.pose.orientation.y newMocapMsg.pose.orientation.z]; 
    mocapMsgLog(i,:) = [mocap_t mocap_q];
    timestamps_mocap(i,:) = timestamp_mocap;

end


save(strcat(saveFolder, '/imuReadings'), 'timestamps_imu', 'imuMsgLog');
save(strcat(saveFolder, '/mocapLog'), 'timestamps_mocap', 'mocapMsgLog');

disp('IMU and mocap log saved!');