%function streamMocap(node, numMocap, saveFolder)

    saveFolder = fullfile('.', '/Tests/RobMech/Dynamic/TestSeries_3/');
    numMocap = 7000;

    clear mocapMsgLog timestamps_mocap

    mocapNode = ros2node("mocap_node", 11);

    mocapSub = ros2subscriber(mocapNode, '/fakeDrone/pose_stamped', Reliability="besteffort");

    %Initialise mocap log
    mocapMsgLog = [];
    timestamp_mocap = [];

    %% RECORDING LOOP
    disp('Recording mocap data...');
    for i=1:numMocap
        %receive mocap message
        newMocapMsg = receive(mocapSub, 1);
        timestamp_mocap = datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS');   

        % Update mocap log
        mocap_t = [newMocapMsg.pose.position.x newMocapMsg.pose.position.y newMocapMsg.pose.position.z];
        mocap_q = [newMocapMsg.pose.orientation.w newMocapMsg.pose.orientation.x  newMocapMsg.pose.orientation.y newMocapMsg.pose.orientation.z]; 
        mocapMsgLog(i,:) = [mocap_t mocap_q];
        timestamps_mocap(i,:) = timestamp_mocap;
    
    end
    
     save(strcat(saveFolder, '/mocapLog'), 'timestamps_mocap', 'mocapMsgLog');
   
    disp('Mocap log saved!');
   