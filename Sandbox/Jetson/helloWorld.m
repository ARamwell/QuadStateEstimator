function helloWorld()

disp("Hello AGAIN World!");
%coder.cinclude('stdio.h');
%coder.ceval('printf', 'Ello\n');

%pause(1);

%out = 1+2;

    % %% ROS2 INITIALISATIONS
    % %coder.ceval('printf', 'Starting ROS2 program. \n');
    % chatterNode = ros2node("chatter_node", 2); %create node
    % % coder.ceval('printf', 'Node created. \n');
    % disp('Node created');
    % % 
    % chatterSub = ros2subscriber(chatterNode, '/chatter', "std_msgs/String", Reliability="besteffort"); % create subscriber
    % % coder.ceval('printf', 'Sub created. Reading msgs... \n');
    % disp('Sub created. reading msgs...');
    % 
    % %create log file
    % fid = fopen('output_report', 'w');
    % 
    % 
    % % 
    % % %% RECORDING LOOP
    % % %disp('Reading uxrce data...');
    % % %coder.ceval('printf', 'Node '
    % for i=1:10
    %      newMsg = receive(chatterSub, 5);  %receive mocap message
    %      newMsgData = newMsg.data;
    %      disp(newMsgData);
    %      fprintf(fid, '%s\n', newMsgData); % Writes each element of the row followed by a tab
    %      %fprintf(fid, '\n'); % Moves to the next line
    % 
    % 
    % %     newUxrceMsg_gyro = (newUxrceMsg .gyro_rad);
    % %     newUxrceMsg_accel = (newUxrceMsg .accelerometer_m_s2);
    % % 
    % %     coder.ceval('printf', strcat(newUxrceMsg_gyro, newUxrceMsg_accel, '\n'));
    % %     disp([newUxrceMsg_gyro newUxrceMsg_accel]);
    % %     pause(0.2);
    % end
    % 
    % fclose(fid);
    % % coder.ceval('printf', 'Msgs done. Terminating... \n');
    % % 
    % % clear('uxrceSub');
    % % clear('uxrceNode');
    % % 

end
