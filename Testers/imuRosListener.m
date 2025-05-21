%IMU LISTENER
%function [imuHist, imuTs] = imuRosListener(node, numMsgs, saveFolder, fileName)
    clear imuMsgLog timestamps_imu

    saveFolder = fullfile('.', '/Tests/RobMech/Dynamic/TestSeries_3');
    numMsgs = 100000;
    fileName = '/imuReadings';

    %% ROS2 INITIALISATIONS
    imuNode = ros2node("ekf_node", 1);
    
    % create subscriber
    %imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', @imuReceiveCallback, Reliability="besteffort");
    imuSub = ros2subscriber(imuNode, '/fmu/out/sensor_combined', Reliability="besteffort");
    %p3pSub = ros2subscriber(ekfNode, '/p3p', @p3pReceiveCallback, Reliability="besteffort");
      
    imuHist = [];
    
    for i=1:numMsgs
        
        newImuMsg = receive(imuSub, 2);
        timestamp_pc = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');
    
        newImuData_gyro = newImuMsg .gyro_rad;
        newImuData_accel = newImuMsg .accelerometer_m_s2;
    
        %cal_accel = transpose(transpose(newImuData_accel) * A + b); 

        imuMsgLog(i,:) = [newImuMsg.gyro_rad' newImuMsg.accelerometer_m_s2'];
        timestamps_imu(i,:) = timestamp_pc;
    
    end
  
    %saveFile = fullfile('.', '/Tests/RobMech/dynamic/imuReadings_ellipsoid');
    save(strcat(saveFolder, fileName), 'timestamps_imu', 'imuMsgLog');
