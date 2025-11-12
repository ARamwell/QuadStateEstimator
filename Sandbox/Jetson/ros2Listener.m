
%MOCAP LISTENER - TEST ROS2 CONNECTION

%% LOG DESTINATION
clear mocapMsgLog timestamp_mocap
mocapMsgLog = [];
timestamp_mocap = [];


%% ROS2 INITIALISATIONS
mocapNode = ros2node("mocap_node", 11); %create node
topicList = ros2("topic","list","DomainID",11);

mocapSub = ros2subscriber(mocapNode, '/JohnMocap/pose_stamped', Reliability="besteffort"); % create subscriber



%% RECORDING LOOP
disp('Recording mocap data...');
for i=1:2000
    newMocapMsg = receive(mocapSub, 2);  %receive mocap message
    newPose = [newMocapMsg.pose.position.x newMocapMsg.pose.position.y newMocapMsg.pose.position.z newMocapMsg.pose.orientation.w newMocapMsg.pose.orientation.x  newMocapMsg.pose.orientation.y newMocapMsg.pose.orientation.z];
    disp(newPose);
    pause(0.2);

end
