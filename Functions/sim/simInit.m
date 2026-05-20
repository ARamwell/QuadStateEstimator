%function [outputArg1,outputArg2] = simInit(simset, traj)
%SIMINIT Summary of this function goes here
%   Detailed explanation goes here

    
    %% GET TRAJECTORY
    goodIndices = (~isnan(traj(1,:)));
    traj = traj(:, goodIndices);
    wp_times =  (traj(1, :));
    wp_pos =  (traj(2:4, :));
    wp_eul = (traj(5:7, :));
    %clean of nan

    traj_pos_ts = timeseries(wp_pos, wp_times);
    traj_eul_ts = timeseries(wp_eul, wp_times);


    %% OLD INIT
        
    map = simset.map;%load(fullfile('.', '/Resources/map.mat'));
    %featureMap = featureMap;
    
    T_imu2rq = map.worldObjectStruct.transforms.T_imu2genquad;
    R_imu2rq = T_imu2rq(1:3, 1:3);
    t_imu2rq = T_imu2rq(1:3, 4);
    
    T_rc2rq = map.worldObjectStruct.transforms.T_gencam2genquad;
    R_rc2rq= T_rc2rq(1:3, 1:3);
    t_rc2rq = T_rc2rq(1:3, 4);
    
    T_uc2rc = map.worldObjectStruct.transforms.T_simcam2gencam;
    R_uc2rc = T_uc2rc(1:3, 1:3);
    
    T_uw2rw = map.worldObjectStruct.transforms.T_sim2world;
    R_uw2rw = T_uw2rw(1:3, 1:3);
    
    T_uq2rq = map.worldObjectStruct.transforms.T_simquad2genquad;
    R_uq2rq = T_uq2rq(1:3, 1:3);
    
    if ~isempty(map.worldObjectStruct.checkers)
        t_checker = map.worldObjectStruct.checkers(1).Position;
        X_pnts_W = map.worldObjectStruct.checkers(1).Corners;
        checkerSize = map.worldObjectStruct.checkers(1).NumSquares;
        squareSize = map.worldObjectStruct.checkers(1).SquareSize;
    end
    
    
    simHz = simset.simHz;
    imuHz = simset.imuHz;
    imuHz = 1/(1/imuHz - rem((1/imuHz), (1/simHz)));
    ekfHz = simset.ekfHz;
    ekfHz = 1/(1/ekfHz - rem((1/ekfHz), (1/simHz)));
    envHz = simset.envHz;
    envHz = 1/(1/envHz - rem((1/envHz), (1/simHz)));
    fps = simset.fps;
    fps = 1/(1/fps - rem((1/fps), (1/envHz)));
    if exist('imuWait', 'var') == 0
        imuWait = (1/simHz)*2;
    end 
    
    camParams = simset.camParams;%load(camFile);
    camParams_p = cameraParameters(camParams);%
    K = camParams.K;
    k_rad = camParams.RadialDistortion;
    k_tan = camParams.TangentialDistortion;
    %p3pNode = ros2node('p3p_node', 2);

    accelCalib = simset.accelCalib;
    gyroCalib = simset.gyroCalib;
    
    
    %pose_0 = [wp_pos_ts.Data(:,1,1); eul2quat(deg2rad(wp_eul_ts.Data(:,1,1))', 'XYZ')']; 
    pose_0 = [simset.pos0; eul2quat(simset.eul0', 'XYZ')']; 
    
    %setIMUParams;
    %end

