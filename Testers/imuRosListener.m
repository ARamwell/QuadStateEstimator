%IMU LISTENER

%% ROS2 INITIALISATIONS
ekfNode = ros2node("ekf_node");

% create subscriber
%imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', @imuReceiveCallback, Reliability="besteffort");
imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', Reliability="besteffort");
%p3pSub = ros2subscriber(ekfNode, '/p3p', @p3pReceiveCallback, Reliability="besteffort");

%imuParams = [0 0 0 0 0 0;       % CAL_GYRO0_XOFF CAL_GYRO0_YOFF CAL_GYRO0_ZOFF CAL_GYRO0_XSCALE CAL_GYRO0_YSCALE CAL_GYRO0_ZSCALE
%             0 0 0 0 0 0;       % CAL_ACC0_XOFF.... CAL_ACC0_XSCALE...(X/Y/Z)
%             0 0 0 0 0 0];      % CAL_MAG0_XOFF CAL_MAG0_XSCALE ...(X/Y/Z) 

imuHist = [];

%record at a fixed frequency of 20Hz
%rateCtrlr = rateControl(20);
%reset(rateCtrlr); 
%tic

for i=1:2000
    
    newImuMsg = receive(imuSub, 2);
    timestamp_pc = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');

    %t = rateCtrlr.TotalElapsedTime;
    newImuData_gyro = newImuMsg .gyro_rad;
    newImuData_accel = newImuMsg .accelerometer_m_s2;

    %cal_accel = transpose(transpose(newImuData_accel) * A + b); 
    %newImuData = [newImuData_gyro; newImuData_accel];

    

    %TRY CORRECT

    %newImuData_accel_corr = transpose( imuCorrect(newImuData_accel));
    %t = toc;
    %imuHist(i,:) = [rateCtrlr.TotalElapsedTime, newImuData_accel'];
    imuHist(i,:) = [timestamp_pc, newImuData_accel'];


    %imuHist(i,:) = newImuData;

    %disp(newImuData_accel');  
 
    %disp([newImuData_accel_corr, newImuData_accel']);
    
    %pause(0.05);
    %waitfor(rateCtrlr);
end

%imuHist = [imuParams; imuHist];
%writematrix(imuHist, fullfile('.', '/Testers/imuCalib/Px6C_imuHist_static_zdown_room2_raw_20250404_1102.csv'))