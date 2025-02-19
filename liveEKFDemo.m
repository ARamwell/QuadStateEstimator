

%% GENERAL INITIALISATIONS

% Import map
map = load('./Resources/map.mat');

% Get checkerboard variables
X_pnts_W = map.worldObjectStruct.checkers.Corners/1000; %in m
checkerSize = map.worldObjectStruct.checkers.NumSquares;
checkerEdgeLength = map.worldObjectStruct.checkers.SquareSize; %mm
checker_w_mm = 31*checkerSize(2);
checker_h_mm = 31*checkerSize(1);

% Known transformations and offsets
R_uw2rw = map.worldObjectStruct.transforms.rotm_sim2world;
Rt_imu2rq = map.worldObjectStruct.transforms.rt_imu2genquad;
Rt_rc2rq = map.worldObjectStruct.transforms.rt_gencam2genquad;


% Set camera intrinsics
K = [ 267.7991 0 159.5525; 0 278.1177 109.0253; 0 0 1]; %virtual camera

% Initialise plot
plotStruct = initPlots('KneipN', 'EKF');
p3pPlotting.addCheckerboard(plotStruct.traj.Ax, X_pnts_W);

% Data histories
p3pResult = struct();
p3pResult.KneipN = struct();
p3pResult.KneipN.Rt = [];
imuHist = [];
ekfHist = [];
rtHist_ekf = [];
dt_hist = [];
dynterm_hist = [];
p3pHist = [];

%% ROS2 INITIALISATIONS
ekfNode = ros2node("ekf_node");

% Some flags
global newP3pFlag;
global newP3pData;
newP3pFlag = 0;
newP3pData = [];
numFrames = 0;
newImuFlag = 0;
newImuData = [];
trans = [0; 0; 0];
t_p3p = 0;

%parpool(1);

%f1 = parfeval(@run, 0, 'streamCamP3pToRos');
%pause(10);
disp('Starting EKF');

% create subscriber
%imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', @imuReceiveCallback, Reliability="besteffort");
imuSub = ros2subscriber(ekfNode, '/fmu/out/sensor_combined', Reliability="besteffort");
p3pSub = ros2subscriber(ekfNode, '/p3p', @p3pReceiveCallback, Reliability="besteffort");




%% EKF constants

%Initial State covariance
P_k = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]); %Initial

%process noise covariance (noise space)
%If low
Q = diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05]);

%and measurement covariance
W =diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05]);


%% EKF loop

% Set initial state
% while newP3pFlag == 0
%     pause(0.2);
% end   
% newP3pFlag = 0;
% x_k = [newP3pData; 0; 0; 0];
x_k = [0; 0; 0; 0; 1; 0; 0; 0; 0; 0];
%x_k = [0; 0; 0; 0.7071; 0; -0.7071; 0; 0; 0; 0]; %on rear
%x_k = [0; 0; 0; 0.7071; 0; 0.7071; 0; 0; 0; 0]; %on nose
%x_k = [0; 0; 0; 0.7071; -0.7071; 0; 0; 0; 0; 0]; %on left
%x_k = [0; 0; 0; 0.7071; 0.7071; 0; 0; 0; 0; 0]; %on right
ekfHist(:, 1) = x_k;
rtHist_ekf(:,4,1)= [(x_k(1:3,1))];
rtHist_ekf(:,1:3,1)= (quat2rotm(transpose(x_k(4:7, 1))));


% start timer
tic

% Loop until exited
for i=1:1000
    %if new image is received, incorporate it
    if newP3pFlag == 1
        newP3pFlag = 0;
        quat = [newP3pData.pose.orientation.w; newP3pData.pose.orientation.x;  newP3pData.pose.orientation.y; newP3pData.pose.orientation.z];
        trans_prev = trans;
        t_p3p_prev = t_p3p;
        trans = [newP3pData.pose.position.x; newP3pData.pose.position.y; newP3pData.pose.position.z];
        t_p3p = double(newP3pData.header.stamp.sec) + double(newP3pData.header.stamp.nanosec*(10e-6));
        vel = (trans - trans_prev)/(t_p3p - t_p3p_prev);
        z_k = [trans; quat; vel];
        p3pResult.KneipN.Rt(:,:,end+1) = [quat2rotm(transpose(z_k(4:7, 1))), (z_k(1:3, 1))];
        plotStruct.traj = updatePlot(plotStruct.traj, p3pResult);
        p3pHist(:,:, end+1) = z_k;
        %get velocity pseudo-measurement
        %v = 
    else
        z_k = NaN;
    end
    
    
    %If new sensor data is received, predict next state
    %if newImuFlag == 1
    newImuMsg = receive(imuSub, 2);
    % newImuData_gyro = newImuMsg .gyro_rad;
    % newImuData_accel = newImuMsg .accelerometer_m_s2;
    % cal_accel = transpose(transpose(newImuData_accel) * A + b); 
    % newImuData = [newImuData_gyro; cal_accel];
    newImuData = [newImuMsg.gyro_rad; newImuMsg.accelerometer_m_s2];

        dt = toc;
        tic;
        dt_hist(1, end+1)=dt;
        %dt=0.1;
        newImuFlag = 0;                 %reset flag
        u_k = newImuData;
        %u_k(1:4) = [0; 0; 0; 0];
        %u_k(6)=-9.81;
        %u_k=[0; 0; 0; 0; 0; -9.81];
        imuHist(:, end+1) = u_k;
               
        [x_k, P_k, proTerm_k, xHat_k, zHat_k] = EKF_3dQuad_funcs.EKF_loop(x_k, P_k, u_k, Q, z_k, W, dt, Rt_imu2rq, Rt_rc2rq);
        ekfHist(:,end+1) = x_k;
        rtHist_ekf(:,4,end+1)= [(x_k(1:3,1))];
        rtHist_ekf(:,1:3,end)= (quat2rotm(transpose(x_k(4:7, 1))));
        dynterm_hist(:,end+1) = proTerm_k;
  
        % Plot data
        plotStruct.traj.plotLines.EKF.Data = rtHist_ekf;
        p3pPlotting.updateTraj(plotStruct.traj.plotLines.EKF.line, plotStruct.traj.plotLines.EKF.frameText, plotStruct.traj.plotLines.EKF.frameLines, rtHist_ekf)
        drawnow

    % Save data
    %end

end

%% Callback functions

function p3pReceiveCallback(message)
    if ~isempty(message)
        global newP3pFlag;
        global newP3pData;
        newP3pFlag = 1;
        %q = [message.orientation.w; message.orientation.x;  message.orientation.y; message.orientation.z];
        %t = [message.position.x; message.position.y; message.position.z];
        newP3pData = message;
    end
end

function imuReceiveCallback(message)
    if ~isempty(message)
        newImuFlag = 1;
        newImuData = [message.gyro_rad; message.accelerometer_m_s2];
       
    end
end