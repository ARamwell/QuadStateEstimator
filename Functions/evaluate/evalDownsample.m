%% Initialise preliminaries
%% Initialise preliminaries
mapfile = './Resources/map.mat';
map = load(fullfile(mapfile));
g = [0 0 -9.81]'; %for simulation
calibrate = true;%true;
down2kHz =true;
downEkf = true;
simpleDown = true;


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

%% 
idealImuData.rawdata = simout.idealIMU.signals.values';
idealImuData.time = simout.IMU.time';

[groundTruth, imuData, ~, ~ ] = processSimData(simout, 0, 0);

%%
integ = 'rect';
ekfSize = 16;
alpha = 0;
down = 0;
startCalc = 5; %Which log values to start at
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

%% with calibrated IMUs, try different downsampling
    for i=1:6
        stat(i) = fitdist((idealImuData.rawdata(i,startCalc:end)'-u_hist(i,:)'),'Normal');
    end

%simple on ideal
    k = ceil(imuHz/ekfHz);
    u_timeHist_ideal = downsample(idealImuData.time(:,startCalc:end)', k, 0)';
    u_hist_ideal= downsample(idealImuData.rawdata(:,startCalc:end)', k, 0)';
  


%simple
    k = ceil(imuHz/ekfHz);
    u_timeHist_simple = downsample(u_timeHist', k, 0)';
    u_hist_simple = downsample(u_hist', k, 0)';
    for i=1:6
        stat_simple(i) = fitdist((u_hist_ideal(i,:)'-u_hist_simple(i,:)'),'Normal');
    end

% others
 %first, downsample to 2000Hz
        gyro_2kHz = downsample(u_hist(1:3,:)', imuHz/2000, 0)';
       
        %then, do basic averaging to downsample accel to 2kHz
        accel_fullkHz = u_hist(4:6,:);
        k = ceil(imuHz/2000);
        accel_means = movmean(accel_fullkHz, k,2);
        accel_2kHz = downsample(accel_means', k, (k-startCalc+1))';
    
        u_timeHist_2kHz = downsample(u_timeHist', k, 0)';
        u_hist_2kHz = [gyro_2kHz; accel_2kHz];
       
        % u_hist = downsample(u_hist', imuHz/ekfHz, 0)';
        % u_timeHist = downsample(u_timeHist', imuHz/ekfHz, (0))';
 
 % averaging
         %then, do basic averaging
        k = ceil(2000/ekfHz);
        u_means = movmean(u_hist_2kHz, k, 2);
        u_hist_avg = downsample(u_hist_2kHz', k, (k-2))';
        u_timeHist_avg = downsample(u_timeHist_2kHz', k, 0)';
        for i=1:6
            stat_avg(i) = fitdist((u_hist_ideal(i,:)'-u_hist_avg(i,:)'),'Normal');
        end

 % technical downsampler
        [u_hist_tech, u_timeHist_tech] = imuHistDownsampler(u_hist_2kHz, u_timeHist_2kHz, 2000, ekfHz, 0);
        for i=1:6
            stat_tech(i) = fitdist((u_hist_ideal(i,1:end-1)'-u_hist_tech(i,:)'),'Normal');
        end

 % technical downsampler
    [u_hist_corr, u_timeHist_corr] = imuHistDownsampler(u_hist_2kHz, u_timeHist_2kHz, 2000, ekfHz, 1);
    for i=1:6
        stat_corr(i) = fitdist((u_hist_ideal(i,1:end-1)'-u_hist_corr(i,:)'),'Normal');
    end


 %

    

 %% function
 
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
        if t==1
            reset=1;
        end
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