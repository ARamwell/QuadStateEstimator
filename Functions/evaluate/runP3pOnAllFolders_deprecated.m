% SET OPTIONS
importSimLogs = true;
importRealData = false;
importMocap = true;
importImages = true;
rtRounding = 10;

parentFolder = 'C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\RobMech\Dynamic\TestSeries_3\';
allSubFolders = genpath(parentFolder);
% Parse into a cell array.
remain = allSubFolders;
listOfFolderNames = {};
while true
	[singleSubFolder, remain] = strtok(remain, ';');
	if isempty(singleSubFolder)
		break;
	end
	listOfFolderNames = [listOfFolderNames singleSubFolder];
end
numberOfSubFolders = length(listOfFolderNames);


%% Loop through subfolders
for k=2:numberOfSubFolders %Ignore 1st entry (parent folder)

    %% Actually do correction
    currentFolder = string(listOfFolderNames(k));
    
    
    clear imuMsgLog timestamps_imu mocapMsgLog timestamps_mocap imageStream p3pResult;
  
    if ~contains(currentFolder, 'arb') && ~contains(currentFolder, 'calib')

        %File directories
        imgFolder = fullfile(currentFolder);
        %logFile = fullfile('.','/QuadSimEnv/Results/Traj-0024/simout_ideal_100hz_16s.mat');
        imuLogFolder = fullfile(currentFolder, '\imuReadings_sync.mat');
        mocapLogFolder =fullfile(currentFolder, '\mocapLog_sync.mat');
        
        
        % INITIALISE CONSTANTS
        
        global g;
        g = [0; 0; -9.7952];
        
        %INITIALISE WORLD MAP
        map = load('./Resources/map.mat');
        
        %Camera variables
        %K = [1274.58238813250 0 627.693937913345; 0 1273.36371905619 405.441250468717; 0 0 1]; %23 mm calibration
        %K=   [302.9345 0 161.3325; 0 310.8146 131.1024; 0 0 1.0000]; %31 mm calib, esp32 
        K = [458.944297528687 0 249.584321044399; 0 459.543641247509 172.625660963493; 0 0 1];%esp32 svga
        %K = [1107 0 960; 0 1107 540; 0 0 1]; %virtual camera
        
        %Checkerboard variables
        X_pnts_W = map.worldObjectStruct.checkers.Corners/1000; %in m
        checkerSize = map.worldObjectStruct.checkers.NumSquares;
        checkerEdgeLength = map.worldObjectStruct.checkers.SquareSize; %mm
        checker_w_mm = 31*checkerSize(2);
        checker_h_mm = 31*checkerSize(1);
        
        %Known coordinate transformations (mapping)
        Rt_uw2rw  = map.worldObjectStruct.transforms.rt_sim2world;
        Rt_imu2rq = map.worldObjectStruct.transforms.rt_imu2genquad;
        Rt_rc2rq = map.worldObjectStruct.transforms.rt_gencam2genquad;
        
        
        %other variables
        p3pResult = struct();
        inlierThreshold =1;
        
        
        %INTIALISE PLOT
        close all
        
        %ADD TRAJECTORIES
        plotStruct = initPlots('KneipN', 'Verification', 'EKF');
        p3pPlotting.addCheckerboard(plotStruct.traj.Ax, X_pnts_W);
        
        
        
        %IMPORT & PLOT LOGGED DATA
        
        if (importSimLogs == true) || (importMocap == true) || (importRealData == true)
        
            %Create struct to hold verification data
            groundTruth.quadState = [];
            groundTruth.quadRt = createArray(3,4, 0, 'double');
            groundTruth.quadTime = createArray(1, 0, 'datetime');
            groundTruth.quadTime = datetime(groundTruth.quadTime, 'Format', 'yyyyMMdd_HHmmss_SSS');
            groundTruth.camPose = createArray(7,0, 'double');
            groundTruth.camTime = createArray(1, 0, 'datetime');
            groundTruth.camTime = datetime(groundTruth.camTime, 'Format', 'yyyyMMdd_HHmmss_SSS');
            groundTruth.elapsedTime_synced = createArray(1,0, 'double');
        
            imu.rawdata = createArray(6,0, 'double');
            
            imu.time = createArray(1, 0, 'datetime');
            imu.time = datetime(imu.time, 'Format', 'yyyyMMdd_HHmmss_SSS');
        
            %Import sim data
            if importSimLogs ==true
                [groundTruth.quadRt, groundTruth.quadTime, groundTruth.quadState, groundTruth.camTime, imu.rawdata, imu.time] = imgFuncs.importSimLog(logFile);
            end
            
            %Or import mocap data
            if importMocap == true
                Rt_mc2rw = map.worldObjectStruct.transforms.rt_mocap2world;
                Rt_mcq2rq = map.worldObjectStruct.transforms.rt_mcquad2genquad;
                [groundTruth.quadRt, groundTruth.quadState, groundTruth.quadTime] = importMocapLog(mocapLogFolder,  Rt_mc2rw, Rt_mcq2rq);
            end
        
            %Or import real imu logs
            if importRealData ==    true
                imuLog = load(imuLogFolder);
                imu.time = datetime(imuLog.timestamps_imu', 'InputFormat', 'yyyyMMdd_HHmmss_SSS'); 
                imu.rawdata = imuLog.imuMsgLog';  
                imu.calibdata = createArray(6,size(imu.time, 2), 'double');
                %Also calibration?
                %Set calibration parameters
                ba_calib = [-0.173; -0.164; 0.329];
                Ka_calib = [0.992 0 0; 0 0.989 0; 0 0 0.975];
                %bg_calib = [0.009; -0.038; -0.006];
                bg_calib = [0 0 0]';
                Kg_calib = [1 0 0; 0 1 0; 0 0 1];
                R_imu2rq = eul2rotm(deg2rad([-0.431, -1.874, 0]), 'XYZ');
                Rt_imu2rq(1:3, 1:3) = R_imu2rq;   
                for r=1:size(imu.time, 2)
                    imu.calibdata(:,r) = [(Kg_calib * imu.rawdata(1:3, r) - bg_calib); (Ka_calib * imu.rawdata(4:6, r)-ba_calib)];
                end
                
            end
        
            %%synchronise ground truth and imu data
            startTime_imu = imu.time(1);
            endTime_imu = milliseconds(imu.time(end)-startTime_imu)/1000;
            for t=1:size(groundTruth.quadTime,2)
                groundTruth.elapsedTime_synced(1,t)=milliseconds(groundTruth.quadTime(1,t)-startTime_imu)/1000;
            end
            %get closest times
            [closestDiff, closestIndex] = min(abs(groundTruth.elapsedTime_synced));
            syncedTime_zeroIndex_gt = closestIndex;
            [closestDiff, closestIndex] = min(abs(groundTruth.elapsedTime_synced(1,:)-endTime_imu));
            syncedTime_endIndex_gt = closestIndex;
        
                
            %update plots
            lineObj_Ver = plotStruct.traj.plotLines.Verification.line;
            axTextArr_Ver=plotStruct.traj.plotLines.Verification.frameText;
            axLineArr_Ver = plotStruct.traj.plotLines.Verification.frameLines;
            p3pPlotting.updateTraj(lineObj_Ver, axTextArr_Ver, axLineArr_Ver, groundTruth.quadRt);
        
        end
        
        
        %Import images for P3P    
        if importImages == true  
            
            imageStream = struct();
            imageStream.img = [];
            imageStream.time = createArray(1, 0, 'datetime');
            imageStream.time = datetime(imageStream.time, 'Format', 'yyyyMMdd_HHmmss_SSS');
            
            if importRealData == true
                [imageStream.img, imageStream.time] = imgFuncs.importImageSeq(imgFolder, 1);
            else
                [imageStream.img, imageStream.time] = imgFuncs.importImageSeq(imgFolder, 0);
            end
            imageStream.res = size(imageStream.img(:,:,1));
        end
        
        
        %RUN AND PLOT P3P FOR EACH CHECKERBOARD IMAGE
        if importImages == true
        
            goodFrameCounter = 0;
            totalFrames = size(imageStream.time,2);
            imageSize = imageStream.res;
        
            for i=1:totalFrames
                disp(strcat("Working on frame ", string(i)));
                
                %Get current frame
                I=imageStream.img(:,:,i);
            
                %Find checkerboard corners
                [x_pnts_i, checkerSize_detected] = detectCheckerboardPoints(I);
                x_pnts_i = transpose(x_pnts_i);    
            
                %discard any frames where too few corners have been detected
                if (checkerSize_detected(1) ~= checkerSize(1)) || (checkerSize_detected(2) ~= checkerSize(2))
                    disp(strcat("Discarding frame ", string(i)));
                    continue
                end
                %Otherwise, continue:
                goodFrameCounter = goodFrameCounter +1;
                % 
                % if goodFrameCounter == 1
                %     startIndex = 1;
                % end
                
                %Run P3P
                p3pResult = runP3P(p3pResult, imageStream.time(i), x_pnts_i, X_pnts_W, K, Rt_rc2rq, imageSize, checkerSize, inlierThreshold, rtRounding,  'KneipN');
                plotStruct.traj = updatePlot(plotStruct.traj, p3pResult);
                
                % %Calculate error
                % if importSimLogs == true
                %     [~,closestIndex] = min(abs(cam_timeHist_GT-time_I));
                %     Rt_log_i = quad_rtHist_GT(:,:,closestIndex);
                %     p3pResult = calcRtError(p3pResult, quad_rtHist_GT, quad_timeHist_GT);    
                % end
            
                % %Display checkerboard image for this iteration - for troubleshooting
                % if (checkerSize_detected(1) <= checkerSize(1)) || (checkerSize_detected(2) <= checkerSize(1)) 
                %     if i==1
                %         checkerFig = figure();
                %         checkerAx = axes('Parent', checkerFig);
                %         checkerImg = imshow(imgFuncs.markDetectedCheckers(I, x_pnts_i), 'Parent', checkerAx);
                %     else
                %         checkerImg.CData = (imgFuncs.markDetectedCheckers(I, x_pnts_i));
                %         drawnow;
                %     end 
                % end
            
             end
             
             %Calculate errors?
            if (importMocap ==true) || (importSimLogs == true)
                for i=1:size(p3pResult.KneipN.time, 2)
                    %Find the closest ground truth measurement for this time
                    [closestDiff, closestIndex] = min(abs(groundTruth.quadTime(1,:)-p3pResult.KneipN.time(1, i)));
                    closestGT = groundTruth.quadState(:,closestIndex);
                    for n=1:size(p3pResult.KneipN.quadPoseArr,2)
                        [p3pResult.KneipN.quadPoseError(1, n, i), p3pResult.KneipN.quadPoseError(2, n, i)] = getPoseError(closestGT, p3pResult.KneipN.quadPoseArr(:,n,i));
                    end
                    [pose, err, ind] = chooseMinPoseErr(p3pResult.KneipN.quadPoseArr(:,:,i), closestGT, 1, 1);
                    if ~isnan(err)
                        p3pResult.KneipN.best.quadPose(:,i) = pose;
                        p3pResult.KneipN.best.err(:,i) = err;
                        p3pResult.KneipN.best.truePose(:,i) = closestGT;
                        %[p3pResult.KneipN.best.quadPose(:,i), p3pResult.KneipN.best.err(:,i), bestIndex] = chooseMinPoseErr(p3pResult.KneipN.quadPoseArr(:,:,i), closestGT, 1, 1);
                    end
                end
            end
        
            %p3pResultFile = fullfile('.', '/Results/p3pResult.mat')
            save(fullfile(currentFolder,  '/p3pResult.mat'), 'p3pResult'); 
        end 
    end
end
