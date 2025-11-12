%MOCAP LISTENER - TEST ROS2 CONNECTION
function ros2Listener_uxrce()
    
    disp('Starting ROS2 program.');

    %% ROS2 INITIALISATIONS
    uxrceNode = ros2node("uxrce_node", 1); %create node
    % coder.ceval('printf', 'Node created. \n');
    % disp('Node created');
    % 
    uxrceSub = ros2subscriber(uxrceNode, '/fmu/out/sensor_combined', "px4_msgs/SensorCombined", Reliability="besteffort"); % create subscriber
    % coder.ceval('printf', 'Sub created. Reading msgs... \n');
    % disp('Sub created. reading msgs...');

    %create output log
    fid = fopen('output_report', 'w');


    %% RECORDING LOOP
    %disp('Reading uxrce data...');
    %coder.ceval('printf', 'Node '
    for i=1:10
        newUxrceMsg = receive(uxrceSub, 2);  %receive mocap message
        newUxrceMsg_gyro = (newUxrceMsg .gyro_rad);
        newUxrceMsg_accel = (newUxrceMsg .accelerometer_m_s2);

        %coder.ceval('printf', strcat(newUxrceMsg_gyro, newUxrceMsg_accel, '\n'));
        disp([newUxrceMsg_gyro newUxrceMsg_accel]);
        fprintf(fid, '%s\n', sprintf('%f', newUxrceMsg_gyro(1))); 
        
    end
    % coder.ceval('printf', 'Msgs done. Terminating... \n');
    % 
    % clear('uxrceSub');
    % clear('uxrceNode');
    % 
    fclose(fid);


end