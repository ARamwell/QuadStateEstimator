%IMU LISTENER

%% ROS2 INITIALISATIONS
ekfNode = ros2node("ekf_node");

% create subscriber
imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', Reliability="besteffort");


imuRaw = [];
orientations = {'XUP', 'XDOWN', 'YUP', 'YDOWN', 'ZUP', 'ZDOWN'};
orientations = {'ALL'};

for j=1:length(orientations)
    fprintf('Please position your accelerometer in the following orientation: %s \n', orientations{j});
    pause('on');
    pause;
    for i=1:1000
        newImuMsg = receive(imuSub, 2);
        newImuData = [newImuMsg .accelerometer_m_s2];
    
        imuRaw(i,:, j) = newImuData;
        
        disp(transpose(newImuData));
        
        pause(0.05);
    end
end

figure();
scatter3(imuRaw(:,1,1), imuRaw(:,2,1), imuRaw(:,3,1));

[A,b] = accelcal(imuRaw(:,:,1), imuRaw(:,:,2), imuRaw(:,:,3), imuRaw(:,:,4), imuRaw(:,:,5), imuRaw(:,:,6)); 

imuMeans_raw = [];
for l=1:6
    for c=1:3
        imuMeans_raw(l, c) = mean(imuRaw(:,c,l));
    end
end

meanoffset_x = mean([(9.81 - imuMeans_raw(1,1)); (imuMeans_raw(2,1) + 9.81); - imuMeans_raw(3,1); - imuMeans_raw(4,1); imuMeans_raw(5,1); -imuMeans_raw(6,1)]);
meanoffset_y = mean([imuMeans_raw(1,2); -imuMeans_raw(2,2); (9.81 - imuMeans_raw(3,2)); (imuMeans_raw(4,2) + 9.81); imuMeans_raw(5,2); -imuMeans_raw(6,2)]);
meanoffset_z = mean([imuMeans_raw(1,3); imuMeans_raw(2,3); imuMeans_raw(3,3); imuMeans_raw(4,3); (imuMeans_raw(5,3) - 9.81); (imuMeans_raw(6,3) + 9.81)]);

imuCorrected = [];
for l=1:6
    for i=1:size(imuRaw, 1)
        imuCorrected(i, 1, l) = imuRaw(i, 1, l) + meanoffset_x;
        imuCorrected(i, 2, l) = imuRaw(i, 2, l) - meanoffset_y;
        imuCorrected(i, 3, l) = imuRaw(i, 3, l) - meanoffset_z;
        imuCorrected(i, 4, l) = norm(imuCorrected(i, 1:3, l));
    end
end

imuAutoCorrected = [];
for l=1:6
    for i=1:size(imuRaw, 1)
        imuAutoCorrected(i, 1:3, l) = imuRaw(i, 1:3, l)*A + b;
    end
end

imuMeans_corr = [];
for l=1:6
    for c=1:4
        imuMeans_corr(l, c) = mean(imuCorrected(:,c,l));
    end
end
