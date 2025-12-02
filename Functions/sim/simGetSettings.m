function [simset, targetFolder] = simGetSettings()
    
    %settings
    simset.envHz = 100;
    simset.SITL = false;
    simset.mocapTraj = false;
    simset.runEstimator = true;
    simset.imuHz = 1000;
    simset.simHz = 1000;
    simset.fps = 10;
    simset.ekfHz = 1000;
    simset.pos0 = [0 0 0]';
    simset.eul0 = [0 0 0]';
    simset.g = [0; 0; 9.81];
    simset.imuDelay = 0.002;
    simset.duration = 10;
    simset.aidingActive = true;
    simset.runEKF = true;
    simset.runP3P = true;
    simset.save = false;
    simset.multisim = false;
    simset.imu = 2; %0-no processing; 1-no downsampling; 2-downsampled; 3-PX4 downsampled (ROS2)
   
    
    
    %directories
    targetFolder = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\Diss1';
    mapFile = './Resources/map.mat';
    accelParamFile = './Resources/Calibrations/accel_accel0_20250808_gauss_nobias.mat';
    gyroParamFile = './Resources/Calibrations/gyro_gyro0_nobias.mat';
    camParamFile = fullfile('C:/Users/Alyssa/Documents/QuadStateEstimator/Resources/Calibrations/params_imx219_640p.mat');
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
    if simset.runEstimator == true
        simset.runEKF = true;
        if simset.aidingActive == true
            simset.runP3P = true;
        end
    end

    if simset.SITL == false && simset.imu==3
        simset.imu = 1;
    end
    
    %% SAVE SETTINGS
    if simset.save == true
        %make folder for sim output
        currentTime = datetime('now');
        currentTimeStr = string(currentTime, 'yyyy-MM-dd_HH-mm-ss');
        targetName = strcat("sim_", currentTimeStr);
        targetFolder = strcat(targetFolder, '\', targetName);
        if ~exist(targetFolder, 'dir')
            mkdir(targetSaveFolder, targetName);
        end
end



