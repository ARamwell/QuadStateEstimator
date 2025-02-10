function ros2SubscriberExample
    % Initialize ROS 2 Node
    rosNode = ros2node('my_node');
    
    % Shared variable
    latestMsg = []; 
    
    % Create Subscriber with a nested callback function
    sub = ros2subscriber(rosNode, '/fmu/out/sensor_accel', @messageCallback, Reliability="besteffort");

    % Wait and display messages in a loop (or replace with GUI updates)
    for i = 1:10  % Run for 10 iterations
        pause(1);  % Wait for messages
        if ~isempty(latestMsg)
            disp(['Received: ', latestMsg]);  % Display the latest message
        end
    end
    
    % Cleanup
    delete(sub);
    delete(rosNode);

    % Nested callback function
    function messageCallback(message)
        latestMsg = [message.x message.y message.z];  % Directly modify shared variable
        disp(latestMsg);
    end
end