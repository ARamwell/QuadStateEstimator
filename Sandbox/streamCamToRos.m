%create ros node
camNode = ros2node('cam_node');

%create ros publisher
imgPub = ros2publisher(camNode, '/img', "sensor_msgs/Image");

%create ros2 message
imgMsg = ros2message("sensor_msgs/Image");

%create ip cam
cam = ipcam('http://192.168.159.121:81/stream');
%img = imread(fullfile('C:\Users\Alyssa\OneDrive - University of Cape Town\Projects\MSc\QuadStateEstimator\QuadSimEnv\Results\Traj-0007\3750.jpg'));
[img, timestamp] = snapshot(cam);

%populate img message
imgMsg.height = uint32(500);
imgMsg.width = uint32(800);
%imgMsg.encoding = "rgb8";
imgMsg.data= img;

send(imgPub,imgMsg);

%maybe run p3p here?

% %preview(cam)
% 
% n=1;
% nFrames = 1000;
% 
% while (n<nFrames) 
% 
%     [img, timestamp] = snapshot(cam); %capture image
%     n = n+1;
% end
% 
% clear all
