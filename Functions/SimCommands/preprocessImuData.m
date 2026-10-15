function [u_hist, u_timeHist] = preprocessImuData(imuData, diffImuHz, down2kHz, downEkf,calibrate, biasPerturbment, accelCalib, gyroCalib, imuHz, ekfHz)
%PREPROCESSIMUDATA Summary of this function goes here
%   Detailed explanation goes here


    startCalc = 3; %Which log values to start at (get rid of initital transients from sim)
 
    
    %Inputs
    ua_hist = imuData(1).rawdata(4:6, 1:end);
    ua_timeHist = imuData(1).time;
    if diffImuHz == true
        ug_hist = imuData(2).rawdata(1:3, 1:end);
        ug_timeHist = imuData(2).time;
    else
        ug_hist = imuData(1).rawdata(1:3, 1:end);
        ug_timeHist = imuData(1).time;
    end

    % if vecnorm(biasPerturbm) ~= 0
    %     bg_perturb = 0.05*selectedPerturb(f, 1:3)';
    %     ba_perturb = 0.15*selectedPerturb(f, 4:6)';
    % else
    %     bg_perturb = zeros(3,1);
    %     ba_perturb = zeros(3,1);
    % end
    bg_perturb = biasPerturbment(1:3);
    ba_perturb = biasPerturbment(4:6);

    if calibrate == true
        Ka = accelCalib.scale;
        ba= 0*accelCalib.turnOnBias';
        Kg = gyroCalib.scale;
        bg = 0*gyroCalib.turnOnBias';
        for i = 1:size(ug_hist, 2)
            ug_hist(1:3,i) = Kg*ug_hist(1:3,i) - bg +bg_perturb;
        end
        for i = 1:size(ua_hist, 2)
            ua_hist(1:3,i) = Ka*ua_hist(1:3,i) + ba - ba_perturb;
        end
    end

    %% Downsampler

    if down2kHz || diffImuHz
        k = ceil(imuHz/2000);

        if diffImuHz == false
            %first, downsample gyro to 2000Hz
            gyro_2kHz = downsample(ug_hist(1:3,:)', k)';
            gyro_times = downsample(ug_hist(1:3,:)', k)';
            
        else
            gyro_2kHz = ug_hist;
            gyro_times = ug_timeHist;
        end
        

       
        %then, do basic averaging to downsample accel to 2kHz
        accel_fullkHz = ua_hist(1:3,:);
        accel_means = movmean(accel_fullkHz, k,2);
        accel_2kHz = downsample(accel_means', k)';

        %get new ua and ug
        u_hist = [gyro_2kHz; accel_2kHz];   
        u_timeHist = gyro_times;

        if downEkf
            %then, do basic averaging
            k = ceil(2000/ekfHz);
            % u_means = movmean(u_hist, k, 2);
            % u_hist = downsample(u_hist', k, (k-2))';
            % u_timeHist = downsample(u_timeHist', k, 0)';
            
            % run downsampler
            [u_hist, u_timeHist] = imuHistDownsampler(u_hist, u_timeHist, 2000, ekfHz, 0);  
        end
    end
   
    u_timeHist = u_timeHist(:,startCalc:size(u_hist, 2));
    u_hist = u_hist(:,startCalc:end);
end

