%% Initialise preliminaries
mapfile = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Resources\map_real';
map = load(fullfile(mapfile));

g = [0 0 -9.81]'; %for simulation
aiding = true;
calibrate = true;
down2kHz =false;
downEkf =true;
diffImuHz =true;
tiltAlign = true;

% Which EKFs to run
integ_arr = {'rect'};
ekfSize_arr = [16];
alpha_arr = [0];
numEkfs = size(ekfSize_arr, 2);

biasPerturb = false;

%Known coordinate transformations (mapping)
T_uw2rw = map.worldObjectStruct.transforms.T_sim2world;
T_imu2rq = map.worldObjectStruct.transforms.T_imu2genquad;
T_rc2rq = map.worldObjectStruct.transforms.T_gencam2genquad;

%% Choose folders
listOfFolderNames =selector_multiFolder(pwd, 'Select sim folders to run EKF on');
numFolders = length(listOfFolderNames);

%% Create bias perturb array
if biasPerturb == true
    %bias perturbment
    % Example: Create a sample array
    A = [-1, -0.9, -0.8, -0.7, -0.6, -0.5, 0.5, 0.6, 0.7, 0.8, 0.9, 1]; 
    numSamples = 6; 
    % Generate a vector of 'numSamples' random indices (integers between 1 and length(A))
    randomIndexes = randi([1, numel(A)], numSamples, numFolders); %
    % Select the elements using the indices
    selectedPerturb = A(randomIndexes);
else
    selectedPerturb = zeros(6, numFolders);
end


%%
for f=1:numFolders
    clear ekfResult p3pResult

    %% Get current folder
    currentFolder = string(listOfFolderNames(f));
    parts = strsplit(currentFolder, '\');
    trajName = string((parts{end}));

    %% Import data (also run P3P)
    [groundTruth, imuData, p3pResult, simset] = importSimData(currentFolder,diffImuHz, aiding, map);

    %% Preprocess IMU (downsample and calibrate)
    ekfHz = 250;
    [u_hist, u_timeHist] = preprocessImuData(imuData, diffImuHz, down2kHz, downEkf, calibrate, selectedPerturb(:,f), simset.accelCalib, simset.gyroCalib, simset.imuHz, ekfHz);

    %% Preprocess P3P
    if aiding
        [z_arr, z_timeHist, z_best]=preprocessP3pResult(p3pResult, groundTruth);
    end
    
    %% Set up EKF
    for e=1:numEkfs
        clear ekfResult
        integ = integ_arr{e};
        ekfSize = ekfSize_arr(e);
        alpha = alpha_arr(e);
        endCalc =size(u_hist,2);

        %% Initialisation
        %x_k_ = groundTruth.quad.state(1:ekfSize,startCalc+1); %good initial guess
        %x_k_(8:10,1) = zeros(3,1);
        x_k_ = zeros(ekfSize, 1); %bad initial guess
        x_k_(4,1) = 1; 
        if tiltAlign
            a_norm = u_hist(4:6, 1)/vecnorm( u_hist(4:6, 1));
            g_norm = -[0 0 1]';
            ang =acos(dot(a_norm, g_norm));
            ax= cross(a_norm, g_norm);
            ax = ax/vecnorm(ax);
            quat = axang2quat([ax', ang]);
            x_k_(4:7)=quat';
        end
        
        %dt_av = 1/ekfHz
        [P_k_, Q, W_k] = EKF_3dQuad_funcs.initEKF_params(1/ekfHz, x_k_, ekfHz);
        %Q = 0.5*Q;
        W_k = W_k;
        
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
        for i = 1:(endCalc-1) %currently sample based
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
                    %z_arr_k = reshape(z_arr(:,:,closestIndex), 7,[]);
                    % if timeSinceLastCorrection < 0.3 %if not too much time has passed
                    %     %Choose soln closest to previous estimate
                    %     
                    %     [z_new,a,b] = chooseMinPoseErr(z_arr_k, x_k_, 1, 0);
                    % else %if too much time has passed
                    %     % Or choose min reproj
                    %     z_new = z_arr_k(:,1);
                    % end
                    z_new=z_best(:, closestIndex);
                    meas_oldIndex = closestIndex; %update index
                    if ~isnan(z_new(1,1))
                        lastCorrectionTime = t_new;
                    end
                    meas_count = meas_count + 1;
                end
            end
        
            [flag_constVel, T_a, T_w]=constVelDetector(u_hist(:,i), 3*10^(-5), 0.1, 5, i); %guess 3e-5 and 0.08
            [x_k_, P_k_, xHat_k, PHat_k, zHat_k, z_out_k, y_k, K_k, S_k, Q_k, W_k] = EKF_3dQuad_funcs.EKF_loop(g, x_k_, P_k_, u_new, Q, z_new, W_k, dt_new, integ, alpha, meas_count, 0);
        
            count = count + 1;
            ekfResult.time(:,count) = t_new;
            ekfResult.x_(:,count) = x_k_;
            ekfResult.u(:,count) = u_new;
            ekfResult.z(:,count) = z_out_k;
            ekfResult.xHat(:,count) = xHat_k; 
            ekfResult.zHat(:,count)= zHat_k;
            ekfResult.elapsedTime(:,count) = t_new - t0;
            ekfResult.timeSinceLastCorrection(:,count)= timeSinceLastCorrection;
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
        ekfResult.trajErr = evaluateTrackingPerformance(ekfResult.x_, ekfResult.trueState, 'none');
        if size(ekfResult.trueState,1)>10
            for g=1:size(ekfResult.trueState,2)
                ekfResult.trueState(11:16,g) = ekfResult.trueState(11:16,g) +[selectedPerturb];
            end
        end
        ekfResult.nees = evalNEES_noq(ekfResult.x_, ekfResult.PHat, ekfResult.trueState);
        ekfResult.nis = evalNIS_noq(ekfResult.y, ekfResult.S);
        ekfResult.partialNees.pos =  evalNEES(ekfResult.x_(1:3,:), ekfResult.PHat(1:3,1:3,:), ekfResult.trueState(1:3, :));
        ekfResult.partialNees.orient =  evalNEES(ekfResult.x_(5:7,:), ekfResult.PHat(5:7,5:7,:), ekfResult.trueState(5:7, :));
        ekfResult.partialNees.vel =  evalNEES(ekfResult.x_(8:10,:), ekfResult.PHat(8:10, 8:10,:), ekfResult.trueState(8:10, :));
        ekfResult.partialNis.pos = evalNIS(ekfResult.y(1:3,:), ekfResult.S(1:3,1:3,:));
        ekfResult.partialNis.orient= evalNIS(ekfResult.y(5:7,:), ekfResult.S(5:7, 5:7 ,:));

        %% Optionally, save
        saveFile = strcat("ekfResult_", trajName, "_", string(ekfSize), "el", "_", string(integ), "_a", string(alpha), "_", string(ekfHz), "Hz", "_test", ".mat");
        %saveFolder = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\ekfComparison2\spiral\";
        %saveFolder = currentFolder; %"C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\Validation\EKF\";
        saveFolder = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\PostDiss\gravAiding";
        destFile = strcat(saveFolder, "\", saveFile);
        save(destFile, '-struct', 'ekfResult');
    end
end


%% Some graphs
% 
% %ekfResult = load("C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\Validation\EKF\sim_2025-12-18_13-22-13_elev8\sim_elev8\ekfResult_16el_rect_idealIMU_deadreckon_8kHz.mat");
% figObj3 =figure;
% figObj2 = plotStateEvolution(ekfResult, ekfResult.trueState, ekfResult.time);
% formatFigForLatex(figObj2);
% [vio, x_err]=evalPercentDivergence(ekfResult.x_, ekfResult.trueState, ekfResult.P, 2);
% 
% figObj1 = figure;
% plotNEES(figObj1, ekfResult.time, ekfResult.nees, 9, 1, 'test');
% formatFigForLatex_v2(figObj1);
% 
% 
 % idx_z_used = ~isnan(ekfResult.z(1,:));
 % z_used=ekfResult.z(:, idx_z_used);
 % z_times = ekfResult.time(:, idx_z_used);
 % figObj3 = plotPoseEvolution(ekfResult, ekfResult.trueState, ekfResult.time, z_used, z_times);
% formatFigForLatex_v2(figObj3);
% 
 % figObj1 = figure();
 % plotATE(figObj1, ekfResult.trajErr, 'test');
% 
% figObj3 = figure();
% plotARE(figObj3, ekfResult.trajErr, 'test');
% 
% figObj4 = figure;
% plotATE(figObj4, p3pResult.trajErr, 'p3p');
% 
% figObj5 = figure;
% plotNIS(figObj5, z_times, ekfResult.nis, 6, 1, 'test');

%% W tuning

%get true measurement residuals
% y_true = calcTrueResid(ekfResult.z, ekfResult.trueState);




%% functions
        % 
        % 
        % 
        % ekfResult1.x_ = x_k_;
        % ekfResult1.u = u_new;
        % ekfResult1.z = z_out_k;
        % ekfResult1.xHat = xHat_k; 
        % ekfResult1.zHat= zHat_k;
        % ekfResult1.elapsedTime = t_new-t0;
        % ekfResult1.timeSinceLastCorrection= timeSinceLastCorrection;
        % ekfResult1.y = y_k; 
        % ekfResult1.K = K_k;
        % ekfResult1.P = P_k_; %these might still have zeroes in the lower triangle
        % ekfResult1.PHat = PHat_k;
        % ekfResult1.S = S_k;
        % ekfResult1.W = W_k;
        % ekfResult1.Q = Q_k;
        % ekfResult1.time = t_new;
        % 
        % 
        % %%
        % t_full = imuData(1).time;
        % 
        % rawAccel=imuData(1).rawdata(4:6,:);
        % rawNorm = vecnorm(rawAccel, 2,1);
        % 
        % calibAccel = (imuData(1).rawdata(4:6,:));
        % for i=1:size(imuData(1).rawdata, 2)
        %     calibAccel(:,i) = Ka*imuData(1).rawdata(4:6, i);
        % end
        % calibNorm =vecnorm(calibAccel, 2,1);
        % 
        % 
        % t_ideal = simout.idealIMU.time;
        % idealAccel = simout.idealIMU.signals.values(:,4:6)';
        % idealNorm = vecnorm(idealAccel,2,1);
        % 
        % idealBias = calibAccel-idealAccel;
        % 
        % idealCorrAccel = calibAccel-repmat(mean(idealBias,2), 1, size(calibAccel,2));
        % idealCorrNorm = vecnorm(idealCorrAccel,2,1);
        % 
        % liveBias = ekfResult.x_(11:13,:);
        % liveCorrAccel = ekfResult.u(4:6,:)-liveBias;
        % liveCorrNorm =vecnorm(liveCorrAccel,2,1);
        % 
        % calibDsAccel =ekfResult.u(4:6,:);
        % calibDsNorm = vecnorm(calibDsAccel,2,1);
        % 
        % 
        % finalBias = ekfResult.x_(11:13,end);
        % finalCorrAccel = calibAccel-repmat(finalBias, 1, size(calibAccel,2));
        % finalCorrNorm = vecnorm(finalCorrAccel,2,1);
        % finalCorrDsAccel = calibDsAccel-repmat(finalBias, 1, size(calibDsAccel,2));
        % finalCorrDsNorm = vecnorm(finalCorrDsAccel,2,1);
        % 
        % 
        % figure;
        % plot(t_full, rawNorm);
        % hold on;
        % plot(t_full, calibNorm);
        % hold on;
        % plot(ekfResult.time, calibDsNorm);
        % hold on;
        % plot(ekfResult.time, finalCorrDsNorm);
        % hold on;
        % plot(ekfResult.time, liveCorrNorm);
        % hold on;
        % plot(t_ideal, idealNorm);
