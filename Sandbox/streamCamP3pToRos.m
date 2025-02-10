%% ROS initialisations

% %create ros node
% camNode = ros2node('cam_node');
% 
% %create ros publisher
% imgPub = ros2publisher(camNode, '/img', "sensor_msgs/Image");
% 
% %create ros2 message
% imgMsg = ros2message("sensor_msgs/Image");

%% Camera initialisations

% intrinsics
K = [605.8071 0 316.6903; 0 608.4646 256.6409; 0 0 1.0000];

%create ip cam
cam = ipcam('http://192.168.159.121:81/stream');
%img = imread(fullfile('C:\Users\Alyssa\OneDrive - University of Cape Town\Projects\MSc\QuadStateEstimator\QuadSimEnv\Results\Traj-0007\3750.jpg'));

%p3p variables
p3pResult = struct();
inlierThreshold = 0.5;

%% Map initialisations

%INITIALISE WORLD MAP
map = load('./Resources/map.mat');

%Checkerboard variables
X_pnts_W = map.worldObjectStruct.checkers.Corners/1000; %in m
checkerSize = map.worldObjectStruct.checkers.NumSquares;
checkerEdgeLength = map.worldObjectStruct.checkers.SquareSize; %mm
checker_w_mm = 31*checkerSize(2);
checker_h_mm = 31*checkerSize(1);

%Known coordinate transformations (mapping)
R_uw2rw = map.worldObjectStruct.transforms.rotm_sim2world;
Rt_imu2rq = map.worldObjectStruct.transforms.rt_imu2genquad;

%plot
plotStruct = initPlots('KneipN');
p3pPlotting.addCheckerboard(plotStruct.traj.Ax, X_pnts_W);

%% Data
%get image
[img, timestamp] = snapshot(cam);

%% Run p3p
for i=1:10
    %Detect checkerboard corners
    [x_pnts_i, checkerSize_detected] = detectCheckerboardPoints(rgb2gray(img));
    x_pnts_i = transpose(x_pnts_i);    
    
    %discard any frames where too few corners have been detected
    if (checkerSize_detected(1) ~= checkerSize(1)) || (checkerSize_detected(2) ~= checkerSize(2))
        disp(strcat("Discarding frame ", string(i)));
        continue;
    end
    
    p3pResult = runP3P(p3pResult, i, x_pnts_i, X_pnts_W, K, imageSize, checkerSize, inlierThreshold, rtRounding,  'KneipN');
    plotStruct.traj = updatePlot(plotStruct.traj, p3pResult);

        %Display checkerboard image for this iteration - for troubleshooting
    if (checkerSize_detected(1) <= checkerSize(1)) || (checkerSize_detected(2) <= checkerSize(1)) 
        if i==1
            checkerFig = figure();
            checkerAx = axes('Parent', checkerFig);
            checkerImg = imshow(imgFuncs.markDetectedCheckers(I, x_pnts_i), 'Parent', checkerAx);
        else
            checkerImg.CData = (imgFuncs.markDetectedCheckers(I, x_pnts_i));
            drawnow;
        end 
    end
    
    % %populate img message
    % imgMsg.height = uint32(500);
    % imgMsg.width = uint32(800);
    % %imgMsg.encoding = "rgb8";
    % imgMsg.data= img;
    % 
    % send(imgPub,imgMsg);
    % 
    % %maybe run p3p here?
    % 
    % % %preview(cam)
    % % 
    % % n=1;
    % % nFrames = 1000;
    % % 
    % % while (n<nFrames) 
    % % 
    % %     [img, timestamp] = snapshot(cam); %capture image
    % %     n = n+1;
    % % end
    % % 
    % % clear all
    pause(1);
end

