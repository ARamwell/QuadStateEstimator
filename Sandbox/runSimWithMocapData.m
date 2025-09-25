
%clear all

%settings
simset.envHz = 100;
simset.SITL = false;
simset.runAll = false; 
simset.mocapTraj = true;
simset.imuHz = 8000;
simset.simHz = 8000;
simset.fps = 10;
simset.ekfHz = 125;
simset.pos0 = [0 0 0]';
simset.eul0 = [0 0 0]';

%folders
parentFolder = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\'; %all traj
parentFolder = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\pitchforward_low1'; %single traj

%% GENERAL PARAMS
g = [0; 0; 9.81];
map = load('./Resources/map.mat');

%% SET IMU PARAMETERS
accelCalib = load('./Resources/Calibrations/accel_accel0.mat');
gyroCalib = load('./Resources/Calibrations/gyro_gyro0.mat');
camCalib_pin = load('./Resources/Calibrations/camPin_esp32cam_320p.mat');
camCalib_fish = load('./Resources/Calibrations/camFish_esp32cam_320p.mat');
    
%% SET/EXTRACT MOCAP TRANSFORMATIONS
R_align =[    0.9995    0.0109   -0.0302;
           -0.0129    0.9975   -0.0700;
            0.0294    0.0703    0.9971]'; %all scrubbed data

%R_align = eul2rotm(deg2rad([7.0563   -2.6378   -0.3665]), 'XYZ'); %all static
%t_align     %R_align = eul2rotm(deg2rad([7   8  10]), 'XYZ');
R_align = eye(3);

T_mc2rw = map.worldObjectStruct.transforms.T_mocap2world;
T_mcq2rq = map.worldObjectStruct.transforms.T_markers2genquad;
T_mcq2rq(1:3, 1:3) = T_mcq2rq(1:3, 1:3)*R_align;  
T_uq2rq = map.worldObjectStruct.transforms.T_simquad2genquad;

%% GET ALL FOLDERS
if simset.runAll == true
    % Find all subfolders
    allSubFolders = genpath(parentFolder);
    % Parse into a cell array.
    remain = allSubFolders;
    listOfFolderNames = {};
    while true
	    [singleSubFolder, remain] = strtok(remain, ';');
	    if isempty(singleSubFolder)
		    break;
        end
        if ~contains(singleSubFolder, 'arb') && ~contains(singleSubFolder, 'calib') && contains(singleSubFolder, 'tatic') && ~contains(singleSubFolder, 'sim')
	        listOfFolderNames = [listOfFolderNames singleSubFolder];
        end
    end
    listOfFolderNames = listOfFolderNames(2:end);
    numberOfSubFolders = length(listOfFolderNames);
else
    listOfFolderNames = parentFolder;
    numberOfSubFolders = 1;
end



%% IMPORT TRAJECTORIES
for k=1:numberOfSubFolders
    if numberOfSubFolders ==1 
        currentFolder = listOfFolderNames;
    else
        currentFolder = string(listOfFolderNames(k));
    end
    
    %extract recorded trajectory
    mocapLogFolder = (fullfile(currentFolder, 'mocapLog_sync.mat'));
    [mocap.quadRt, mocap.quadState, mocap.quadTime, mocap.elapsedTime] = importMocapLog(mocapLogFolder, T_mc2rw, T_mcq2rq);
    
    %extract time limits
    startTime = datetime(mocap.quadTime(1), 'InputFormat', 'yyyyMMdd_HHmmss_SSS');
    endTime = datetime(mocap.quadTime(end), 'InputFormat', 'yyyyMMdd_HHmmss_SSS');
    simDur = milliseconds(endTime - startTime)/1000;
    
    %R_corr = [-1 0 0; 0 -1 0; 0 0 -1];

    %build time series trajectory for sim
    mc_times = milliseconds(mocap.quadTime(:)-startTime)/1000;
    mc_pos = mocap.quadState(1:3, :);
    mc_eul = ((quat2eul((mocap.quadState(4:7, :)'), 'XYZ')))';   
    skip =20; %make the array smaller and smoother by skipping a few values
    wp_times = mc_times(1:skip:end,:); % [mc_times(1:5,:); mc_times(6:skip:end,:)];
    wp_pos =  mc_pos(:, 1:skip:end); %[mc_pos(:,1:5), mc_pos(:, 6:skip:end)];
    wp_eul =  mc_eul(:, 1:skip:end);%[mc_eul(:,1:5), mc_eul(:, 6:skip:end)];  
    wp_eul = rad2deg(unwrap(wp_eul, [], 2));
    wp_eul_smooth = smoothdata(wp_eul, 2,"rlowess", 0.05);
    simset.pos0 = wp_pos(:, 1);
    simset.eul0 = wp_eul(:, 1);

    %figure();
    %plot(wp_times, wp_eul);

    %convert to timeseries
    wp_pos_ts = timeseries(wp_pos, wp_times);
    wp_eul_ts = timeseries(wp_eul, wp_times);
       

    %% RUN SIM
    %set_param('QuadSimEnv/SimulinkEnv.slx', 'StopTime', simDur)
    %out = sim('QuadSimEnv/SimulinkEnv.slx', 'StopTime', string(simDur));

    %% CONDITION DATA
    [ekfResult_cust, p3pData_cust, groundTruth] = processSimData(out);
    %get EKF time-aligned ground truth
    indices = selectClosestTimeIndices(ekfResult_cust.time, groundTruth.quad.time);
    groundTruth.quad.time_estAligned = groundTruth.quad.time(:,indices);
    groundTruth.quad.state_estAligned =  groundTruth.quad.state(:,indices);

    %get measurement time-aligned ground truth
    aid_indices = ~isnan(ekfResult_cust.z(1, :));
    indices = selectClosestTimeIndices(ekfResult_cust.time(:,aid_indices), groundTruth.quad.time);
    groundTruth.quad.time_aidAligned = groundTruth.quad.time(:,indices);
    groundTruth.quad.state_aidAligned =  groundTruth.quad.state(:,indices);
    if isempty(groundTruth.quad.state_aidAligned)
        aiding = false;
    else
        aiding = true;
    end

    %% TELL USER TO SAVE PX4 LOGS
    if simset.SITL == true

        %import PX4 logs automatically

        %first, find newest folder in directory
        px4LogFolders = dir('\\wsl.localhost\Ubuntu-22.04\home\alyssa\PX4-Autopilot\build\px4_sitl_default\log');
        px4LogFolders = px4LogFolders(3:end); %remove current and parent directory
        px4Folders_dates = [];
        for i=1:size(px4LogFolders, 1)
            px4Folders_dates = [px4Folders_dates; datetime(px4LogFolders(i).name)];
        end
        [~, ind_newest] = max(px4Folders_dates);
        px4LogFolder_newest =  px4LogFolders(ind_newest);

        %then, find newest log
        px4LogFiles = dir(strcat(px4LogFolder_newest.folder, '\', px4LogFolder_newest.name));
        px4LogFiles = px4LogFiles(3:end);
        px4Files_dates = [];
        for i=1:size(px4LogFiles, 1)
            px4Files_dates = [px4Files_dates, datetime(px4LogFiles(i).date, 'Locale', 'en_UK')];
        end
        %px4Files_dates = px4LogFiles(1:end).date;
        [~, ind_newest]= max(px4Files_dates);
        px4LogFiles_newest =  px4LogFiles(ind_newest);

        %import ulog
        ulog = ulogreader(strcat(px4LogFiles_newest.folder, '\', px4LogFiles_newest.name));

        %extract useful data- build ekfResult
        ekfResult_px4 = processPx4Data(ulog, aiding);

        %and condition it
        %get EKF time-aligned ground truth
        indices = selectClosestTimeIndices(ekfResult_px4.time, groundTruth.quad.time);
        groundTruth.quad.time_px4Aligned = groundTruth.quad.time(:,indices);
        groundTruth.quad.state_px4Aligned =  groundTruth.quad.state(:,indices);
    
        %get measurement time-aligned ground truth
        aid_indices_px4 = ~isnan(ekfResult_px4.z(1, :));
        indices = selectClosestTimeIndices(ekfResult_px4.time(:,aid_indices_px4), groundTruth.quad.time);
        groundTruth.quad.time_aidAligned_px4 = groundTruth.quad.time(:,indices);
        groundTruth.quad.state_aidAligned_px4 =  groundTruth.quad.state(:,indices);

    end    

    %% PROCESS RESULTS??
    % What should I plot and save? Where should I save it?

    % GET METRICS FOR CUSTOM EKF
    %run trajectory error
    trajErr_cust = evaluateTracking(ekfResult_cust.x_, groundTruth.quad.state_estAligned);
    
    %run nees on a priori state
    %nees_cust_apriori = evalNEES(ekfResult_cust.xHat(1:3, :), ekfResult_cust.PHat(1:3, 1:3, :), groundTruth.quad.state_estAligned(1:3,:));
    nees_cust_apriori = evalNEES_noq(ekfResult_cust.xHat, ekfResult_cust.PHat, groundTruth.quad.state_estAligned);
    mean(nees_cust_apriori);
    
    %run nees on a posteriori state
    if aiding == true
        nees_cust_apost = evalNEES(ekfResult_cust.x_(:, aid_indices), ekfResult_cust.P(:,:,aid_indices), groundTruth.quad.state_aidAligned);
        mean(nees_cust_apost);
    else
        nees_cust_apost = nan(1,1);
    end
        % %run nees on a posteriori state
    % %groundTruth.quad
    % nees_cust_apost = evalNEES(ekfResult_cust.x_(:, aid_indices), ekfResult_cust.P_(:,:,aid_indices), groundTruth.quad.state_aidAligned);
    % mean(nees_cust_apost);
    % alpha = 0.05; %confidence
    % ekfSize = size(ekfResult_cust.x_, 1);
    % numMonteCarlo = 1;
    % chiSquareLimits = [chi2inv(alpha/2, numMonteCarlo*ekfSize), chi2inv(1-alpha/2, numMonteCarlo*ekfSize)]/numMonteCarlo;
    % figure;
    % neesAx_cust_apost = plot(ekfResult_cust.elapsedTime(:,aid_indices), nees_cust_apost);
    % chiLine_lower = yline(chiSquareLimits(1), '--g', 'chi-squared lower');
    % chiLine_upper = yline(chiSquareLimits(2), '--g', 'chi-squared upper');
    % xlabel(neesAx_cust_apost, 'Elapsed time (s)');
    % ylabel(neesAx_cust_apost, 'NEES');
    % title(neesAx_cust_apost, 'NEES of a posteriori state estimate, custom EKF')

    %run NIS
    if aiding == true
        nis_cust = evalNIS(ekfResult_cust.y(:,aid_indices), ekfResult_cust.S(:,:,aid_indices));
        nis_cust_mean = mean(nis_cust);
    end

       
    % DO PLOTTING
    figObj = figure();
    tileLayout = tiledlayout(figObj, 2,2);

    %Plot traj err
    %trajFig_trans = figure;
    nexttile;
    trajAx_trans = plotATE(figObj, trajErr_cust, 'custom EKF');
    nexttile;
    trajAx_rot = plotARE(figObj, trajErr_cust, 'custom EKF');

    % trajAx = plot(trajFig, trajErr_cust, "absolute-translation");
    % set(trajAx, 'Ydir', 'reverse');
    % set(trajAx, 'Zdir', 'reverse');
    % view(trajAx, [-57 30]);

    %plot NEES
    %neesFig = figure;
    nexttile;
    neesAx_cust_apriori = plotNEES(figObj, ekfResult_cust.elapsedTime, nees_cust_apriori, size(ekfResult_cust.xHat, 1), 1, 'a priori state estimate, custom EKF'); 
    hold on;
    if aiding == true
        neesAx_cust_apost = plotNEES(figObj, ekfResult_cust.elapsedTime(:,aid_indices), nees_cust_apost, size(ekfResult_cust.x_, 1), 1, 'a posteriori state estimate, custom EKF');
    end
    hold off

    % Plot NIS
    %nisFig = figure;
    if aiding == true
        nexttile;
        nisAx = plotNIS(figObj, ekfResult_cust.elapsedTime(:,aid_indices), nis_cust, size(ekfResult_cust.y, 1), 1);
    end
    % alpha = 0.05; %confidence
    % measSize = size(ekfResult_cust.y, 1);
    % numMonteCarlo = 1;
    % chiSquareLimits = [chi2inv(alpha/2, numMonteCarlo*measSize), chi2inv(1-alpha/2, numMonteCarlo*measSize)]/numMonteCarlo;
    % figure;
    % nisAx_cust = plot(ekfResult_cust.elapsedTime(:,aid_indices), nis_cust);
    % chiLine_lower = yline(chiSquareLimits(1), '--g', 'chi-squared lower');
    % chiLine_upper = yline(chiSquareLimits(2), '--g', 'chi-squared upper');
    % xlabel(nisAx_cust, 'Elapsed time (s)');
    % ylabel(nisAx_cust, 'NIS');
    % title(nisAx_cust, 'NIS, custom EKF')

   
    if simset.SITL == true
        % GET METRICS FOR PX4 EKF
        %run trajectory error
        trajErr_px4 = evaluateTracking(ekfResult_px4.x_, groundTruth.quad.state_px4Aligned);
        
        %run nees on a priori state
        nees_px4_apriori = evalNEES(ekfResult_px4.xHat, ekfResult_px4.PHat, groundTruth.quad.state_px4Aligned);
        mean(nees_px4_apriori);
        
        %run nees on a posteriori state
        if aiding == true
            nees_px4_apost = evalNEES(ekfResult_px4.x_(:, aid_indices), ekfResult_px4.P(:,:,aid_indices), groundTruth.quad.state_aidAligned_px4);
            mean(nees_px4_apost);
        else
            nees_px4_apost = nan(1,1);
        end

        %AND PLOT
        nexttile(1);
        hold on;
        trajAx_trans = plotATE(figObj, trajErr_px4, 'custom EKF');
        
    end


    %% SAVE RESULTS

    %make folder for sim output
    currentTime = datetime('now');
    currentTimeStr = string(currentTime, 'yyyy-MM-dd_HH-mm-ss');
    targetName = strcat("sim_", currentTimeStr);
    targetFolder = strcat(currentFolder, '\', targetName);
    if ~exist(targetFolder, 'dir')
        mkdir(currentFolder, targetName);
    end

    % SIMULATION OUTPUT
    save(fullfile(targetFolder, '/simout.mat'), 'out'); %save out
    vidFile = fullfile('.', '/camOutput.avi'); %find video
    imgFuncs.convertVideo(vidFile,targetFolder);%save images

    if simset.SITL == true
        %copy log to sim output folder
        copyfile  strcat(px4LogFiles_newest.folder, '\', px4LogFiles_newest.name) targetFolder;
    end



 end


 function [ekfResult, p3pResult, groundTruth] = processSimData(out)

    ekfResult.time = out.ekf_x_.time';
    ekfResult.x_ = out.ekf_x_.signals.values';
    stateSize = size(ekfResult.x_, 1);
    ekfResult.xHat = out.ekf_xHat.signals.values';
    ekfResult.zHat = out.ekf_zHat.signals.values';
    ekfResult.u = out.ekf_u.signals.values';
    ekfResult.elapsedTime = out.ekf_x_.time';
    %ekfResult.timeSinceLastCorrection= timeSinceLastCorrection;
    ekfResult.z = out.ekf_z.signals.values';
    ekfResult.y = out.ekf_y.signals.values'; 
    %ekfResult.zIn;
    for t=1:size(ekfResult.time,2)
        ekfResult.K(:,:,t) = reshape(out.ekf_K.signals.values(t,:), [], 7);
       %P_flat = out.ekf_P_.signals.values(t,:);
        % P_ut = %upper triangular
        % P_ = P_ + triu(P_,1)'; %recover lower
        ekfResult.P(:,:,t) = reshape(out.ekf_P_.signals.values(t,:), stateSize, []); %these might still have zeroes in the lower triange
        ekfResult.PHat(:,:,t) = reshape(out.ekf_PHat.signals.values(t,:),stateSize, []);
        ekfResult.S(:,:,t) = reshape(out.ekf_S.signals.values(t,:), 7, []);
    end

    p3pResult.poseArr = out.p3p_poseArr.signals.values;
    p3pResult.mostIn = out.p3p_mostIn.signals.values(1:7,:)';
    p3pResult.numIn = out.p3p_mostIn.signals.values(8,:)';
    p3pResult.time = out.p3p_poseArr.time';

    groundTruth.quad.state = out.quadState.signals.values';
    groundTruth.quad.time = out.quadState.time';
    groundTruth.cam.state = out.camState_GT.signals.values';
    groundTruth.cam.time = out.camState_GT.time';

 end


function trajErr = evaluateTracking(estStateHist, trueStateHist)
    %estStateHist and trueStateHist as column vector [p; q]

    %convert inputs to rigidtform3d objects
    for i=1:size(estStateHist, 2)
        p = estStateHist(1:3, i)';
        q = estStateHist(4:7, i)';
        tform_est(i) = rigidtform3d(quat2rotm(q), p);
    end
    for i=1:size(trueStateHist, 2)
        p = trueStateHist(1:3, i)';
        q = trueStateHist(4:7, i)';
        tform_true(i) = rigidtform3d(quat2rotm(q), p);
    end

    %run ATE and RPE
    trajErr = compareTrajectories(tform_est, tform_true, AlignmentType = 'none');

    %and drift rate?
end

function [nees] = evalNEES(estStateHist, estCov, trueStateHist)
%will need groundtruth to be time-aligned in advance
    
    [ekfStates, numUpdates] = size(estStateHist);
    %calculate NEES
    for t=1:numUpdates
        x_err =  trueStateHist(1:ekfStates,t) - estStateHist(:,t);
        nees(t) = x_err' * (estCov(:,:,t) \ x_err);
    end

end

function [nees] = evalNEES_noq(estStateHist, estCov, trueStateHist)
%will need groundtruth to be time-aligned in advance
    
    [ekfStates, numUpdates] = size(estStateHist);
    %calculate NEES
    for t=1:numUpdates
        x_err =  trueStateHist(1:ekfStates,t) - estStateHist(:,t);
        x_err(4,:) = [];
        effCov = estCov(:,:,t);
        effCov(4,:) = [];
        effCov(:,4) = [];
        nees(t) = x_err' * (effCov \ x_err);
    end

end

function [nis] = evalNIS(meas_resid, meas_cov)
%will need groundtruth to be time-aligned in advance
        
    numUpdates = size(meas_resid, 2);
    %calculate NIS 
    for t=1:numUpdates
        %will nans be a problem?
        nis(t) = meas_resid(:,t)' * (meas_cov(:,:,t) \ meas_resid(:,t));
    end
    
end

function [ekfResult] = processPx4Data(ulog, aidingActive)

    msg_estStates = readTopicMsgs(ulog, 'TopicNames', 'estimator_states');
    msg_sense = readTopicMsgs(ulog, 'TopicNames', 'sensor_combined');

    ekfResult.time = msg_estStates.TopicMessages{1,1}.timestamp_sample';
    ekfResult.elapsedTime = seconds(msg_estStates.TopicMessages{1,1}.timestamp_sample)';

    p = msg_estStates.TopicMessages{1,1}.states(:,5:7)';
    q = msg_estStates.TopicMessages{1,1}.states(:,1:4)';
    v = msg_estStates.TopicMessages{1,1}.states(:,8:10)';
    bg = msg_estStates.TopicMessages{1,1}.states(:,11:13)';
    ba = msg_estStates.TopicMessages{1,1}.states(:,14:16)';
    ekfResult.x_ = [p; q; v; bg; ba];

    P_p = msg_estStates.TopicMessages{1,1}.covariances(:,5:7)';
    P_q =msg_estStates.TopicMessages{1,1}.covariances(:,1:4)';
    P_v= msg_estStates.TopicMessages{1,1}.covariances(:,8:10)';
    P_bg= msg_estStates.TopicMessages{1,1}.covariances(:,11:13)';
    P_ba= msg_estStates.TopicMessages{1,1}.covariances(:,14:16)';
    for t = 1: size(P_p, 2)
        ekfResult.P(:, :, t) = diag([P_p(:,t); P_q(:,t); P_v(:,t); P_bg(:,t); P_ba(:,t)]');
    end


    % measurements
    if aidingActive == true
        msg_aidpos = readTopicMsgs(ulog, 'TopicNames', 'estimator_aid_src_ev_pos');
        msg_aidhgt = readTopicMsgs(ulog, 'TopicNames', 'estimator_aid_src_ev_hgt');
        %ekfResult.statePred = out.ekf_xHat.signals.values'; can't get this
        %ekfResult.measPred = out.ekf_zHat.signals.values'; %measurements map
        %directly
        xy = msg_aidpos.TopicMessages{1,1}.observation';
        z = msg_aidhgt.TopicMessages{1,1}.observation';
        %yaw?
        ekfResult.z =  [xy; z];
    
        %innovations
        S_xy = msg_aidpos.TopicMessages{1,1}.innovation_variance';
        S_z = msg_aidhgt.TopicMessages{1,1}.innovation_variance';
        for t = 1: size(S_z, 2)
            ekfResult.S(:, :, t) = diag([S_xy(:,t); S_z(:,t)]');
        end
        y_xy = msg_aidpos.TopicMessages{1,1}.innovation_variance';
        y_z= msg_aidhgt.TopicMessages{1,1}.innovation_variance';
        ekfResult.y = [y_xy; y_z];
    else
        ekfResult.z = nan(3, size(ekfResult.x_, 2));
        ekfResult.y = nan(3, size(ekfResult.x_, 2));
        ekfResult.S = nan(3, 3, size(ekfResult.x_, 2));
    end

    %sensor readings
    ekfResult.u = [msg_sense.TopicMessages{1,1}.gyro_rad'; msg_sense.TopicMessages{1,1}.accelerometer_m_s2'];
    
    %ekfResult.timeSinceLastCorrection= timeSinceLastCorrection;

    %ekfResult.zIn;

end

function compareEKFMetrics(ekfResult)

end


function [axObj] = plotNEES(figObj, x, nees, stateSize, numMonteCarloRuns, title)
    
    figure(figObj);
    %axObj = axes('Parent', figObj);
    alpha = 0.05; %confidence
    chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*stateSize)]/numMonteCarloRuns;
    
    axObj = plot(x, nees, 'DisplayName', title);
    xlabel('Elapsed time (s)');
    ylabel('NEES');
    ylim([0, round(chiSquareLimits(2) * 1.5)])
    
    chiLine_lower = yline(chiSquareLimits(1), '--g', 'chi-squared lower');
    chiLine_upper = yline(chiSquareLimits(2), '--g', 'chi-squared upper');

end

function [axObj] = plotNIS(figObj, x, nis, stateSize, numMonteCarloRuns, title)
    
    %axObj = axes('Parent', figObj);
    figure(figObj);
    alpha = 0.05; %confidence
    chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*stateSize)]/numMonteCarloRuns;
    
    axObj = plot(x, nees, 'DisplayName', title);
    xlabel('Elapsed time (s)');
    ylabel('NIS')
    ylim([0, round(chiSquareLimits(2) * 1.5)])
    
    chiLine_lower = yline(chiSquareLimits(1), '--g', 'chi-squared lower');
    chiLine_upper = yline(chiSquareLimits(2), '--g', 'chi-squared upper');

end

function [ateAx] = plotATE(figObj, trajErrMetrics, qualifier)
    
    figure(figObj); %make current figure
    ateAx = plot(trajErrMetrics, "absolute-translation");
    set(ateAx, 'Ydir', 'reverse');
    set(ateAx, 'Zdir', 'reverse');
    view(ateAx, [-57 30]);
    %title(strcat('Absolute Translation Error - ', qualifier));

end

function [areAx] = plotARE(figObj, trajErrMetrics, qualifier)
    
    figure(figObj);
    areAx = plot(trajErrMetrics, "absolute-rotation");
    set(areAx, 'Ydir', 'reverse');
    set(areAx, 'Zdir', 'reverse');
    view(areAx, [-57 30]);
    %title(strcat('Absolute Rotation Error - ', qualifier));

end

function [rpeAx] = plotRPE(figObj, trajErrMetrics, qualifier)
    
    figure(figObj);
    rpeAx = plot(trajErrMetrics, "absolute-translation");
    set(rpeAx, 'Ydir', 'reverse');
    set(rpeAx, 'Zdir', 'reverse');
    view(rpeAx, [-57 30]);
    %title(strcat('Relative Pose Error - ', qualifier));

end

function plotInv(matArray)

    invArray = createArray(1, size(matArray, 3));
    
    for i=1:size(matArray, 3)
        invArray(1,i) = inv(matArray(:,:,i));
    end
    
    figure;
    plot(invArray);
    

end









