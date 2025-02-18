%IMU LISTENER

%% ROS2 INITIALISATIONS
ekfNode = ros2node("ekf_node");

% create subscriber
imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', Reliability="besteffort");


imuHist = [];
orientations = {'XUP', 'XDOWN', 'YUP', 'YDOWN', 'ZUP', 'ZDOWN'};

for j=1:length(orientations)
    fprintf('Please position your accelerometer in the following orientation: %s \n', orientations{j});
    pause('on');
    pause;
    for i=1:1000
        newImuMsg = receive(imuSub, 2);
        newImuData = [newImuMsg .accelerometer_m_s2];
    
        imuHist(i,:, j) = newImuData;
        
        disp(transpose(newImuData));
        
        pause(0.05);
    end
end


[A,b] = accelcal(imuHist(:,:,1), imuHist(:,:,2), imuHist(:,:,3), imuHist(:,:,4), imuHist(:,:,5), imuHist(:,:,6)); 

%writematrix(imuHist, fullfile('.', '/Testers/imuCalib/imuHist_.csv'))
%save(fullfile('.', '/Testers/imuCalib/imuHist_.csv'), "imuHist");