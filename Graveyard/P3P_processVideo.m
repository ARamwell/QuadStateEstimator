

%Perform P3P on a prerecorded video of a checkerboard

%Some options
importVideo = false;
useVerificationData =true;
runP3P = true;
startTime = 0.5; %seconds
%startTimeStep = 2;    %seconds
timeIncrement = 0.2; %seconds


%% Initialise camera variables
%K = [1274.58238813250 0 627.693937913345; 0 1273.36371905619
%405.441250468717; 0 0 1]; 23 mm calibration
%K=   [605.8071 0 316.6903; 0 608.4646 256.6409; 0 0 1.0000];%31 mm calib
K = [1107 0 960; 0 1107 540; 0 0 1]; %virtual camera

%% Initialise targets (checkerboard)
checkerSize = [5, 8]; %number of squares
checkerSquareLength = 31; %mm

%Initialise world map
X_pnts_W = map.worldObjectStruct.checkers.Corners;


%% Initialise trajectory variables
t_hist_cust = zeros(3,1); %trajectory history, in the form of translation vector
t_hist_builtin = zeros(3,1);


%% Import verification data 
if useVerificationData
    [verificationData_fileName, verificationData_folder] = uigetfile({'*.mat'}, 'Select camera pose log');
    verificationData_fullFile = fullfile(verificationData_folder, verificationData_fileName);
    verificationData = load(verificationData_fullFile);
    numDataPts = size(verificationData.out.camPose.time, 1);

    dataTimeIncrement = verificationData.out.camPose.time(2)-verificationData.out.camPose.time(1);
    dataStepIncrement = round(timeIncrement/dataTimeIncrement);
    startTimeStep = round(startTime/dataTimeIncrement);
    startTime = startTimeStep*dataTimeIncrement;
    dataTimePnts = 0;
    

    %Initialise trajectory
    t_hist_verif = zeros(3, 1);
    for t = startTimeStep: dataStepIncrement : numDataPts
        
        %Extract translation
        t_verif = transpose(verificationData.out.camPose.signals.values(1,1:3, t));
        %Convert from m to mm
        t_verif = t_verif * 1000;
        %Convert to board coordinate system
        %R_unreal2board = [1 0 0; 0 -1 0; 0 0 -1];
        R_unreal2World = [-1 0 0; 0 -1 0; 0 0 1];
        t_verif=R_unreal2World * t_verif;
        
        if t == startTimeStep
            t_hist_verif = t_verif;
            dataTimePnts(1)=verificationData.out.camPose.time(t);
        else
            t_hist_verif = cat(2, t_hist_verif, t_verif);
            dataTimePnts = cat(1, dataTimePnts, verificationData.out.camPose.time(t));
        end
    end


    
    numFramesToProcess = size(t_hist_verif, 2);


end


%% Initialise trajectory plot
close all
trajPlotFig = figure(); 
trajPlotAx = axes('Parent', trajPlotFig);%, 'XLabel','x (mm)', 'YLabel', 'y (mm)', 'ZLabel', 'z (mm)');
xlim(trajPlotAx, [-500, 500]);
ylim(trajPlotAx,[-500, 500]);
zlim(trajPlotAx,[0, 1500]);
xlabel(trajPlotAx, 'x (mm)');
ylabel(trajPlotAx, 'y (mm)');
zlabel(trajPlotAx, 'z (mm)');
%set(trajPlotAx, 'Ydir', 'reverse');
view(trajPlotAx, 3);
grid(trajPlotAx, 'on');
hold (trajPlotAx, 'on');

%Line plot
trajPlotCust = plot3(trajPlotAx,[0],[0],[0], '-or', 'DisplayName', 'Camera pose (custom algo)');
trajPlotBuiltin = plot3(trajPlotAx,[0],[0],[0], '-ob', 'DisplayName', 'Camera pose (built-in algo)');

%Camera axes
camXPlotCust = plot3(trajPlotAx, [0 100], [0 0], [0 0], '-r');
camYPlotCust = plot3(trajPlotAx, [0 0], [0 100], [0 0], '-r');
camZPlotCust = plot3(trajPlotAx, [0 0], [0 0], [0 100], '-r');

camXPlotBuiltin = plot3(trajPlotAx, [0 100], [0 0], [0 0], '-b');
camYPlotBuiltin = plot3(trajPlotAx, [0 0], [0 100], [0 0], '-b');
camZPlotBuiltin = plot3(trajPlotAx, [0 0], [0 0], [0 100], '-b');
camXLabelBuiltin = text(trajPlotAx, 100,0, 0, 'x');
camYLabelBuiltin=text(trajPlotAx, 0, 100,0, 'y');
camZLabelBuiltin=text(trajPlotAx, 0, 0, 100, 'z');

%Insert checkerboard
scatter3(trajPlotAx,X_pnts_w(:,1), X_pnts_w(:,2),X_pnts_w(:,3), 4, "black", "filled");

if useVerificationData
    trajPlotVerif = plot3(trajPlotAx,[0],[0],[0], '-og', 'DisplayName', 'Camera pose (logged)');
    
    %Plot verification data
    set (trajPlotVerif, 'XData', t_hist_verif(1,:), 'YData',t_hist_verif(2,:), 'ZData', t_hist_verif(3,:));
end

%Insert legend
% legend([trajPlotCust trajPlotBuiltin trajPlotVerif],'Camera pose (custom)','Camera pose (built-in)', 'Camera pose (logged)') 


%% Import video

if runP3P
    if importVideo
        %Convert video .avi to series of .jpegs
        [videoName, videoFolder] = uigetfile({'*.avi'}, 'Pick a Video to Import');
        videoFullFileName = fullfile(videoFolder, videoName);
        video = VideoReader(videoFullFileName);
        
        %Determine how many frames there are
        numFrames = video.NumFrames;
        vidHeight = video.Height;
        vidWidth = video.Width;
        outputFolder = videoFolder;
    
        
        %Determine how many frames to read
        vidDuration = video.Duration;
        secPerFrame = vidDuration/numFrames;
        framesToRead = zeros(1,1);
        if useVerificationData
            %Create array to relate data points with frames
            
            for i=1:size(dataTimePnts)
                time_i = dataTimePnts(i);
                frame_i = round(time_i / secPerFrame);
                framesToRead(i,1) = frame_i;
            end
        else
            startFrame = (startTimeStep*timeIncrement)/secPerFrame;
            frameIncrement = timeIncrement/secPerFrame;
            for n=startFrame:frameIncrement:numFrames
                if n ==startFrame
                    framesToRead(1,1)=n;
                else
                    framesToRead= cat(1, framesToRead, n);
                end
            end
        end   
    
        
        %Convert to series of images
        for j=1 : size(framesToRead, 1)
            f = framesToRead(j,1);
            frame = read(video,f);
            I_gray = rgb2gray(frame);
            if j==1
                I_allFrames = I_gray;
            else
                I_allFrames = cat(3, I_allFrames, I_gray);
            end
            %Write images to file
            outputBaseFileName = sprintf('-%4.4d.jpg', f);
            outputFullFileName = fullfile(outputFolder, outputBaseFileName); %output filename
            imwrite(frame, outputFullFileName, 'jpg'); %write output file
        end
    
    else
        %Import series of images
        % files_ds = fileDatastore(fullfile(uigetdir(matlabroot,'Select a folder containing only the required stills')), 'ReadFcn', @importdata, "FileExtensions",".jpg");
        files_ds = fileDatastore("C:\Users\alyss\OneDrive - University of Cape Town\Sandbox\PnP\Verification\Few\Left", 'ReadFcn', @importdata, "FileExtensions",".jpg");
        allFileNames = files_ds.Files;
        numFiles = length(allFileNames);
        numFramesTotal = numFiles;
    
        %For first iteration
        I_colour = imread(string(allFileNames(1)));
        I_gray = rgb2gray(I_colour);
        I_allFrames = I_gray;
        
        
        %Loop to read all image files and put them into a multidim array
        for n=2: numFiles
        
            I_colour = imread(string(allFileNames(n)));
            I_gray = rgb2gray(I_colour);
            I_allFrames = cat(3, I_allFrames, I_gray);
        end
    end
    
    numFramesToProcess = size(I_allFrames, 3);
    

    

    
    %% Do PNP
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%Do P3P!
    
    %Initialise sim
    %mdl = "P3P_Grunert";
    %open_system(mdl);
    %simIn = Simulink.SimulationInput(mdl);
    
    
    
    %set_param(mdl,"FastRestart","on")

end

%% Update plot

for q=1:numFramesToProcess
    
    if runP3P
        %Extract current frame
        I_current = I_allFrames(:,:,q);
        imageSize = size(I_current);
        
        disp(strcat("Working on frame ", string(q)));
    
        %Detect checkerboard corners
        [x_pnts_i, checkerSize_detected] = detectCheckerboardPoints(I_current);
        x_pnts_i = round(x_pnts_i);
        

        
        
        if checkerSize_detected == checkerSize

            %[x_ABCD_i, X_ABCD_W] = p3pFuncs.checkerOuterCornerSelector(x_pnts_i, X_pnts_w, checkerSize(1), checkerSize(2));

            %%Run simulation
            %out = sim(simIn);
            Rt_Cust = (p3pRun.KneipA(transpose(x_pnts_i), transpose(X_pnts_w), K, checkerSize));
            Rt_Mat = p3pRun.MatlabPnP(K, x_pnts_i, X_pnts_w, imageSize, 0.2);

            %Load new augmented Rt matrix results
            % R_new_cust = out.simout(1:3,1:3,1);
            % t_new_cust = out.simout(1:3,4,1);
            % R_new_builtin = out.simout(1:3,1:3,2);
            % t_new_builtin = out.simout(1:3,4,2);
            R_new_cust = Rt_Cust(:,1:3);
            t_new_cust = Rt_Cust(:,4);
            R_new_builtin = Rt_Mat(:,1:3);
            t_new_builtin = Rt_Mat(:,4);
            
            

            %Add new results to trajectory history
            if q == 1
                t_hist_cust = t_new_cust;
                t_hist_builtin = t_new_builtin;
            else
                t_hist_cust = cat(2, t_hist_cust, t_new_cust);
                t_hist_builtin = cat(2, t_hist_builtin, t_new_builtin);
            end
        
        

            %Plot results

    
            %Plot new trajectory history
            ryPlot(trajPlotCust, camXPlotCust, camXLabelBuiltin)
            set(trajPlotCust, 'XData', t_hist_cust(1,:), 'YData',t_hist_cust(2,:), 'ZData', t_hist_cust(3,:));
            set(trajPlotBuiltin, 'XData', t_hist_builtin(1,:), 'YData',t_hist_builtin(2,:), 'ZData', t_hist_builtin(3,:));
            
            %Plot new camera axes
            %Custom algorithm
            cam_xAxis_cust = cat(2, t_new_cust, (t_new_cust + (R_new_cust * [100;0;0])));
            cam_yAxis_cust = cat(2, t_new_cust, (t_new_cust + (R_new_cust * [0;100;0])));
            cam_zAxis_cust = cat(2, t_new_cust, (t_new_cust + (R_new_cust * [0;0;100])));
            set(camXPlotCust, 'XData', cam_xAxis_cust(1,:), 'YData',cam_xAxis_cust(2,:), 'ZData', cam_xAxis_cust(3,:));
            set(camYPlotCust, 'XData', cam_yAxis_cust(1,:), 'YData',cam_yAxis_cust(2,:), 'ZData', cam_yAxis_cust(3,:));
            set(camZPlotCust, 'XData', cam_zAxis_cust(1,:), 'YData',cam_zAxis_cust(2,:), 'ZData', cam_zAxis_cust(3,:));
            
            %Builtin algorithm
            cam_xAxis_builtin = cat(2, t_new_builtin, (t_new_builtin + (R_new_builtin * [100;0;0])));
            cam_yAxis_builtin = cat(2, t_new_builtin, (t_new_builtin + (R_new_builtin * [0;100;0])));
            cam_zAxis_builtin = cat(2, t_new_builtin, (t_new_builtin + (R_new_builtin * [0;0;100])));
            set(camXPlotBuiltin, 'XData', cam_xAxis_builtin(1,:), 'YData',cam_xAxis_builtin(2,:), 'ZData', cam_xAxis_builtin(3,:));
            set(camYPlotBuiltin, 'XData', cam_yAxis_builtin(1,:), 'YData',cam_yAxis_builtin(2,:), 'ZData', cam_yAxis_builtin(3,:));
            set(camZPlotBuiltin, 'XData', cam_zAxis_builtin(1,:), 'YData',cam_zAxis_builtin(2,:), 'ZData', cam_zAxis_builtin(3,:));
            delete(camXLabelBuiltin);
            delete(camYLabelBuiltin);
            delete(camZLabelBuiltin);
            camXLabelBuiltin = text(trajPlotAx, cam_xAxis_builtin(1,2), cam_xAxis_builtin(2,2), cam_xAxis_builtin(3,2), 'x');
            camYLabelBuiltin =text(trajPlotAx, cam_yAxis_builtin(1,2), cam_yAxis_builtin(2,2), cam_yAxis_builtin(3,2), 'y');
            camZLabelBuiltin =text(trajPlotAx, cam_zAxis_builtin(1,2), cam_zAxis_builtin(2,2), cam_zAxis_builtin(3,2), 'z');

        end
    end

    % %Plot verification data
    % if useVerificationData
    %     set (trajPlotVerif, 'XData', t_hist_verif(1,1:q), 'YData',t_hist_verif(2,1:q), 'ZData', t_hist_verif(3,1:q));
    % end
    
    drawnow
end

%plotCamera(Size=10,Orientation=R_new_builtin', ...
%            Location=t_new_builtin,Color=[0 1 0]);


%% Some functions
function X_pts_w = calcCheckerEdgeCoords_w(checkerSize, squareEdgeLength)
    
    boardWidth = checkerSize(2);
    boardHeight = checkerSize(1);
    X_pts_w = zeros((boardWidth-1)*(boardHeight-1), 3);
    X_pts_b = zeros((boardWidth-1)*(boardHeight-1), 3);

    R_board2World = [1 0 0; 0 -1 0; 0 0 -1];

    %for each column
    for c=1 : boardWidth-1
        for r = 1: boardHeight-1
            x = (c-1)*squareEdgeLength;
            y = (r-1)*squareEdgeLength;
            z = 0;
            X_pts_b(  (((c-1)*(boardHeight-1)) + r), 1) = x;
            X_pts_b(  (((c-1)*(boardHeight-1)) + r), 2) = y;
            X_pts_b(  (((c-1)*(boardHeight-1)) + r), 3) = z;
        end
    end

    X_pts_w = transpose(R_board2World * transpose(X_pts_b));
    
end

%Function below not used
% function [bodyXLabel, bodyYLabel, bodyZLabel] = updateTraj(updatedTrajHist, updatedR, parentAx, targetPlot, bodyXPlot, bodyYPlot, bodyZPlot, bodyXLabel, bodyYLabel, bodyZLabel)
% 
%     set(targetPlot, 'XData', updatedTrajHist(1,:), 'YData', updatedTrajHist(2,:), 'ZData', updatedTrajHist(3,:));
%     body_xAxis = cat(2, updatedTrajHist, (updatedTrajHist + (updatedR * [100;0;0])));
%     body_yAxis = cat(2, updatedTrajHist, (updatedTrajHist + (updatedR * [0;100;0])));
%     body_zAxis = cat(2, updatedTrajHist, (updatedTrajHist + (updatedR * [0;0;100])));
%     set(bodyXPlot, 'XData', body_xAxis(1,:), 'YData', body_xAxis(2,:), 'ZData', body_xAxis(3,:));
%     set(bodyYPlot, 'XData', body_yAxis(1,:), 'YData',body_yAxis(2,:), 'ZData', body_yAxis(3,:));
%     set(bodyZPlot, 'XData', body_zAxis(1,:), 'YData',body_zAxis(2,:), 'ZData', body_zAxis(3,:));
%     delete(bodyXLabel);
%     delete(bodyYLabel);
%     delete(bodyZLabel);
%     bodyXLabel = text(parentAx, body_xAxis(1,2), body_xAxis(2,2), body_xAxis(3,2), 'x');
%     bodyYLabel =text(parentAx, body_yAxis(1,2), body_yAxis(2,2), body_yAxis(3,2), 'y');
%     bodyZLabel =text(parentAx, body_zAxis(1,2), body_zAxis(2,2), body_zAxis(3,2), 'z');
%     drawnow;
% 
% end

