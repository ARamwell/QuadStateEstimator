%% Initialise preliminaries
mapfile = './Resources/map.mat';
map = load(fullfile(mapfile));

g = [0 0 -9.81]'; %for simulation
aiding = true;
calibrate = true;%true;
down2kHz =true;
downEkf =true;



%% Choose folders for sim data
%Select a .mat file
[filename, pathname] = uigetfile('*.mat', 'Select the simout file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
simoutfilepath = fullfile(pathname, filename);
simsetfilepath = fullfile(strcat(pathname, '\simset.mat'));

%Select image directory
%[imgfilepath] = uigetdir(pwd, 'Select the folder of images to import');
imgfilepath = pathname;

%% Import data
simout = load(simoutfilepath);
if length(fieldnames(simout)) == 1
    simout = simout.simout;
end

simset = load(simsetfilepath);
if length(fieldnames(simset)) == 1
    simset = simset.simset;
end

ekfHz = 250; %simset.ekfHz;
imuHz = simset.imuHz;
simHz = simset.simHz;

%% Get data in usable format
[groundTruth, imuData, ~, ~ ] = processSimData(simout, 0, 0);
if aiding
    p3pResult = runP3pOnFile(imgfilepath, cameraParameters(simset.camParams), simset.camParams.K, p3pFuncs.invertT(map.worldObjectStruct.transforms.T_gencam2genquad));
end
%% Set up EKF

integ = 'mtrp';
ekfSize = 16;
alpha = 0;
down = 0;
startCalc = 3; %Which log values to start at
clear ekfResult

%Known coordinate transformations (mapping)
T_uw2rw = map.worldObjectStruct.transforms.T_sim2world;
T_imu2rq = map.worldObjectStruct.transforms.T_imu2genquad;
T_rc2rq = map.worldObjectStruct.transforms.T_gencam2genquad;

%Inputs
u_hist = imuData.rawdata(:, startCalc:end);
u_timeHist = imuData.time(:, startCalc:end);


if calibrate == true
    Ka = simset.accelCalib.scale;
    ba= -simset.accelCalib.turnOnBias';
    Kg = simset.gyroCalib.scale;
    bg = -simset.gyroCalib.turnOnBias';
    for i = 1:size(u_hist, 2)
        u_hist(1:3,i) = Kg*u_hist(1:3,i) + bg;
        u_hist(4:6,i) = Ka*u_hist(4:6,i) - ba;
    end
end

%Downsampler

    if down2kHz
        k = ceil(imuHz/2000);
        %first, downsample gyro to 2000Hz
        gyro_2kHz = downsample(u_hist(1:3,:)', k, (k-2))';
       
        %then, do basic averaging to downsample accel to 2kHz
        accel_fullkHz = u_hist(4:6,:);
        
        accel_means = movmean(accel_fullkHz, k,2);
        accel_2kHz = downsample(accel_means', k, (k-2))';
    
        u_timeHist = downsample(u_timeHist', k, 0)';
        u_hist = [gyro_2kHz; accel_2kHz];
       
        % u_hist = downsample(u_hist', imuHz/ekfHz, 0)';
        % u_timeHist = downsample(u_timeHist', imuHz/ekfHz, (0))';
    
        if downEkf
            %then, do basic averaging
            k = ceil(2000/ekfHz);
            u_means = movmean(u_hist, k, 2);
            u_hist = downsample(u_hist', k, (k-2))';
            u_timeHist = downsample(u_timeHist', k, 0)';
            
            [u_hist, u_timeHist] = imuHistDownsampler(u_hist, u_timeHist, 2000, ekfHz, 0);
    
    
        end
    end

    
    %     u_hist_ds =
    % end

if aiding
    z_timeHist = p3pResult.time;
    z_arr = p3pResult.poseArr;
    z_best = p3pResult.selected;
end


endCalc =size(u_hist,2);

%% Initialisation
x_k_ = groundTruth.quad.state(1:ekfSize,startCalc+1); %good initial guess
x_k_(8:10,1) = zeros(3,1);
%x_k_ = zeros(ekfSize, 1); %bad initial guess
%x_k_(4,1) = 1; 

%dt_av = 1/ekfHz
[P_k_, Q, W_k] = EKF_3dQuad_funcs.initEKF_params(1/ekfHz, x_k_, ekfHz);
%Q = 0.5*Q;
W_k = 10*W_k;


meas_count =  0; %how many frames used to correct so far? Used to initiate adaptive EKF
meas_oldIndex = 0; %what was the last frame used? So we don't reuse frames
count = 1;
t0 = 0;
lastCorrectionTime = 0;
timeSinceLastCorrection = 999;

ekfResult.time(:,count) = t0;
ekfResult.x_(:,count) = x_k_;
ekfResult.P(:,:,count) = P_k_;
ekfResult.Q(:,:,count) = Q;
ekfResult.W(:,:,count) = W_k;



%% Run EKF
for i = startCalc:(endCalc-1) %currently sample based
    
    % Grab newest IMU measurement
    u_new = u_hist(:, i); %current control input
    t_new = u_timeHist(1,i);
    dt_new = u_timeHist(1,i+1)-u_timeHist(1,i); 



    % Check for a camera measurement at this time
    z_new = NaN(7,1); %assume no measurement
    if aiding
        timeSinceLastCorrection = t_new-lastCorrectionTime;   
        [closestDiff, closestIndex] = min(abs(z_timeHist(1,:)-t_new)); %find index of closest time
        if ((closestDiff <= dt_new) && (z_timeHist(1, closestIndex) <= t_new) && closestIndex>meas_oldIndex)%if it is close enough, and not ahead
            % z_arr_k = reshape(z_arr(:,:,closestIndex), 7,[]);
            % %if not too much time has passed
            % if timeSinceLastCorrection < 0.2
            %     %Choose soln closest to previous estimate
            %     z_arr_k = reshape(z_arr(:,:,closestIndex), 7,[]);
            %     [z_new,a,b] = chooseMinPoseErr(z_arr_k, x_k_, 1.2, 0);
            % else %if too much time has passed
            %     %Or choose min reproj
            %     z_new = z_arr_k(:,1);
            % end
            z_new=z_best(:, closestIndex);
            meas_oldIndex = closestIndex; %update index
            lastCorrectionTime = z_timeHist(1, closestIndex);
            meas_count = meas_count + 1;
        end
    end

    [x_k_, P_k_, xHat_k, PHat_k, zHat_k, z_out_k, y_k, K_k, S_k, Q_k, W_k] = EKF_3dQuad_funcs.EKF_loop(g, x_k_, P_k_, u_new, Q, z_new, W_k, dt_new, integ, alpha, meas_count);

    count = count + 1;
    ekfResult.time(:,count) = t_new;
    ekfResult.x_(:,count) = x_k_;
    ekfResult.xHat(:,count) = xHat_k; 
    ekfResult.zHat(:,count)= zHat_k;
    ekfResult.u(:,count) = u_new;
    ekfResult.elapsedTime(:,count) = t_new - t0;
    ekfResult.timeSinceLastCorrection(:,count)= timeSinceLastCorrection;
    ekfResult.z(:,count) = z_out_k;
    ekfResult.y(:,count) = y_k; 
    ekfResult.K(:,:,count) = K_k;
    ekfResult.P(:,:,count) = P_k_; %these might still have zeroes in the lower triangle
    ekfResult.PHat(:,:,count) = PHat_k;
    ekfResult.S(:,:,count) = S_k;
    ekfResult.W(:,:,count) = W_k;
    ekfResult.Q(:,:,count) = Q_k;
end

%% Append EKF result with some useful information
indices = selectClosestTimeIndices(ekfResult.time, groundTruth.quad.time);
ekfResult.trueState = groundTruth.quad.state(:,indices);


%% Calculate and append metrics

%"perfect" conditions - 
%ATE, ARE, NEES
ekfResult.trajErr = evaluateTrackingPerformance(ekfResult.x_, ekfResult.trueState, 'none');
ekfResult.nees = evalNEES_noq(ekfResult.x_, ekfResult.PHat, ekfResult.trueState);
ekfResult.nis = evalNIS_noq(ekfResult.y, ekfResult.S);


%% Optionally, save

saveFile = 'ekfResult_spiral_noisy_16elM_aided_250Hz.mat';
%saveFolder = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\Validation\EKF\sim_2025-12-22_08-03-07_shortStatic\sim_static\";
saveFolder = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\Validation\EKF\";
destFile = strcat(saveFolder, saveFile);
%save(destFile, '-struct', 'ekfResult');

%% Some graphs

%ekfResult = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\Validation\EKF\sim_2025-12-18_13-22-13_elev8\sim_elev8\ekfResult_16el_rect_idealIMU_deadreckon_8kHz.mat");
figObj2 = plotStateEvolution(ekfResult, ekfResult.trueState, ekfResult.time);
formatFigForLatex(figObj2);
[vio, x_err]=evalPercentDivergence(ekfResult.x_, ekfResult.trueState, ekfResult.P, 2);

figObj1 = figure;
plotNEES(figObj1, ekfResult.time, ekfResult.nees, 9, 1, 'test');
formatFigForLatex_v2(figObj1);


idx_z_used = ~isnan(ekfResult.z(1,:));
z_used=ekfResult.z(:, idx_z_used);
z_times = ekfResult.time(:, idx_z_used);
figObj3 = plotPoseEvolution(ekfResult, ekfResult.trueState, ekfResult.time, z_used, z_times);
formatFigForLatex_v2(figObj3);

figObj1 = figure();
plotATE(figObj1, ekfResult.trajErr, 'test');

figObj3 = figure();
plotARE(figObj3, ekfResult.trajErr, 'test');

figObj4 = figure;
plotATE(figObj4, p3pResult.trajErr, 'p3p');

figObj5 = figure;
plotNIS(figObj5, z_times, ekfResult.nis, 6, 1, 'test');

%% W tuning

%get true measurement residuals
y_true = calcTrueResid(ekfResult.z, ekfResult.trueState);




%% functions

function [imuHist_ds, imuTimes_ds] = imuHistDownsampler(imuHist, imuTimes, oldRate, newRate, corrFlag)

    numToCombine = ceil(oldRate/newRate);
    cnt = 0;
    dt = 1/oldRate;
    a_down = createArray(3,0);
    w_down = createArray(3,0);
    
    for t=1:size(imuHist, 2)
        w_new = imuHist(1:3, t);
        a_new = imuHist(4:6,t);
        reset = 0;
        if cnt >= numToCombine
            a_down(:,end+1) = accumVel/(dt*cnt);
            w_down(:,end+1) = accumAngle/(dt*cnt);
            reset = 1;
            cnt = 0;
        end
        [accumVel, accumAngle] = accumImu(a_new, w_new, dt, corrFlag, corrFlag, reset);
        cnt = cnt+1;
    end

    imuHist_ds = [w_down; a_down];
    imuTimes_ds = downsample(imuTimes', numToCombine, 0)';

end