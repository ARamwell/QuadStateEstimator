camSubNode = ros2node('camSub_node');

imgSub = ros2subscriber(camSubNode, '/img');
imgRx = receive(imgSub, 30);