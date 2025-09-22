function ros2_p3pPub(pose)
%ROS2_P3PPUB Summary of this function goes here
%   Detailed explanation goes here

p3pMsg =  ros2message("geometry_msgs/PoseStamped");

    %populate p3p message
    
    p3pMsg.pose.position.x = pose(1);
    p3pMsg.pose.position.y = pose(2);
    p3pMsg.pose.position.z =pose(3);
    p3pMsg.pose.orientation.w = pose(4);
    p3pMsg.pose.orientation.x = pose(5);
    p3pMsg.pose.orientation.y = pose(6);
    p3pMsg.pose.orientation.z = pose(7);
    %p3pMsg.header.stamp.sec = int32(timestamp_s);
    %p3pMsg.header.stamp.nanosec = uint32(timestamp_ns);

    send(p3pPub, p3pMsg);



end

