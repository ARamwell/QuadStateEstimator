function [simset, targetSaveFolder] = simGetSettings()
    
    %settings
    simset.envHz = 20;
    simset.SITL = false;
    simset.mocapTraj = false;
    simset.runEstimator =true;
    simset.imuHz = 20;
    simset.simHz = 20;
    simset.fps = 20;
    simset.ekfHz = 20;
    simset.pos0 = [0 0 0]';
    simset.eul0 = [0 0 0]';
    simset.g = [0; 0; 9.795];
    simset.imuDelay = 0.002;
    simset.duration = 10;
    simset.aidingActive = true;
    simset.save = true;
    simset.multisim = false;
    simset.imu = 2; %0-no processing; 1-no downsampling; 2-downsampled; 3-PX4 downsampled (ROS2)
   
    
    
    %directories
    targetFolder = 'C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1';
    mapFile = './Resources/map.mat';
    accelParamFile = './Resources/Calibrations/accel_accel0_20251211_allan_stable.mat';
    gyroParamFile = './Resources/Calibrations/gyro_gyro0_20251211_allan.mat';
    camParamFile = fullfile('C:/Users/Alyssa/Documents/QuadStateEstimator/Resources/Calibrations/params_imx219_640p_lowdist.mat');
    %% GENERAL PARAMS
    g = simset.g;
    simset.map = load(mapFile);
    
    %% SET IMU PARAMETERS
    simset.accelCalib = load(accelParamFile);
    simset.gyroCalib = load(gyroParamFile);
    %camCalib_pin = load('./Resources/Calibrations/camPin_imx219_640p_ideal.mat');
    %camCalib_fish = load('./Resources/Calibrations/camFish_esp32cam_320p.mat');
    %camCalib_pin.K(1,1)  = camCalib_pin.K(1,1)/1.1;
    %camCalib_pin.K(2,2)  = camCalib_pin.K(2,2)/1.1;
    simset.camParams = load(camParamFile);
    
    %% Some logic
    % if simset.runEstimator == true
    %     simset.runEKF = true;
    %     if simset.aidingActive == true
    %         simset.runP3P = true;
    %     end
    % end

    if simset.SITL == false && simset.imu==3
        simset.imu = 1;
    end
    
    %% SAVE SETTINGS
    if simset.save == true
        %make folder for sim output
        currentTime = datetime('now');
        currentTimeStr = string(currentTime, 'yyyy-MM-dd_HH-mm-ss');
        targetName = strcat("sim_", currentTimeStr);
        targetSaveFolder = strcat(targetFolder, '\', targetName);
        if ~exist(targetSaveFolder, 'dir')
            mkdir(targetFolder, targetName);
        end
end



