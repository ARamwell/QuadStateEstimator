
clear all

%% Find all subfolders
parentFolder = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\';
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
numberOfSubFolders = length(listOfFolderNames);

 % for k=2:numberOfSubFolders
 %    currentFolder = string(listOfFolderNames(k));
currentFolder = fullfile('C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\pitchforward_low1');
    
    %make folder for sim output
    targetName = "sim_16hz";
    targetFolder = strcat(currentFolder, '\', targetName);
    if ~exist(targetFolder, 'dir')
        mkdir targetFolder;
    end
    
    %imu calibration parameters
     %Also calibration?
    ba_stat =[-0.1550; 0.0886; -0.0226];
    bg_stat = [0.0016; 0.0028; 0.0015];
    %Set calibration parameters
    ba_calib = [-0.173; -0.164; 0.329]+ ba_stat;
    Ka_calib = [0.992 0 0; 0 0.989 0; 0 0 0.975];
    %bg_calib = [0.009; -0.038; -0.006];
    bg_calib = [0 0 0]' +bg_stat;
    Kg_calib = [1 0 0; 0 1 0; 0 0 1];
    
    imuHz= 16; 
    
    %set parameters
    
    map = load('./Resources/map.mat');
    %K = [458.944297528687 0 249.584321044399; 0 459.543641247509 172.625660963493; 0 0 1];%esp32 svga
    %K = [462.0327 0 241.8730; 0 462.4926 166.7321; 0 0 1];
   
    K = [458.8803 0 235.5162; 0 458.9989 167.8936; 0 0 1];
    k_rad = [-0.0571 0.1245];
    k_tan = [-0.0035 -0.0064];
    fps = 10;
    
    g = [0 0 -9.81];
    
    %accelerometer
    acc_noise = [5.809e-03  8e-03  11e-03];
    acc_bias_instability = [6.551e-05 1e-06 3.35e-05];
    acc_arw = [1.081e-05 1e-07 6.55e-06];
    acc_max = 78.5;
    acc_resolution = 0.001;
    acc_skew = inv(Ka_calib)*100;
    acc_bias = ((acc_skew/100) *ba_calib)';
    
    %gyro
    gyro_noise = [1.037e-03 1e-03 1.29e-03];
    gyro_bias_instability = [8.998e-07 1e-08 7.66e-07];
    gyro_arw = [0 0 0];
    gyro_max = 34.9;
    gyro_resolution = 0.0001;
    gyro_skew =inv(Kg_calib) * 100; 
    gyro_bias = ((gyro_skew/100) *bg_calib)';
    
    %%Extract trajectory and times
    R_align =[    0.9995    0.0109   -0.0302;
               -0.0129    0.9975   -0.0700;
                0.0294    0.0703    0.9971]'; %all scrubbed data

    %R_align = eul2rotm(deg2rad([7.0563   -2.6378   -0.3665]), 'XYZ'); %all static
    %t_align     %R_align = eul2rotm(deg2rad([7   8  10]), 'XYZ');
    R_align = eye(3);
    mocapLogFolder = (fullfile(currentFolder, 'mocapLog_sync.mat'));
    T_mc2rw = map.worldObjectStruct.transforms.T_mocap2world;
    T_mcq2rq = map.worldObjectStruct.transforms.T_markers2genquad;
    T_mcq2rq(1:3, 1:3) = T_mcq2rq(1:3, 1:3)*R_align;  
    T_uq2rq = map.worldObjectStruct.transforms.T_simquad2genquad;
    [mocap.quadRt, mocap.quadState, mocap.quadTime, mocap.elapsedTime] = importMocapLog(mocapLogFolder, T_mc2rw, T_mcq2rq);
    
    startTime = datetime(mocap.quadTime(1), 'InputFormat', 'yyyyMMdd_HHmmss_SSS');
    endTime = datetime(mocap.quadTime(end), 'InputFormat', 'yyyyMMdd_HHmmss_SSS');
    simDur = milliseconds(endTime - startTime)/1000;
    
    R_corr = [-1 0 0; 0 -1 0; 0 0 -1];
    mc_times = milliseconds(mocap.quadTime(:)-startTime)/1000;
    mc_pos = mocap.quadState(1:3, :);
    mc_eul = ((quat2eul((mocap.quadState(4:7, :)'), 'XYZ')))';
    %mc_rotm = quat2rotm(quatinv(groundTruth.quadState(4:7, :)'));
    %mc_rotm = R_align &
    
    skip =20;
    wp_times = mc_times(1:skip:end,:); % [mc_times(1:5,:); mc_times(6:skip:end,:)];
    wp_pos =  mc_pos(:, 1:skip:end); %[mc_pos(:,1:5), mc_pos(:, 6:skip:end)];
    wp_eul =  mc_eul(:, 1:skip:end);%[mc_eul(:,1:5), mc_eul(:, 6:skip:end)];

   
    wp_eul = rad2deg(unwrap(wp_eul, [], 2));

    wp_eul_smooth = smoothdata(wp_eul, 2,"rlowess", 0.05);

    %figure();
    %plot(wp_times, wp_eul);

    wp_pos_ts = timeseries(wp_pos, wp_times);
    wp_eul_ts = timeseries(wp_eul, wp_times);
    
    
    
    
    
    %% RUN SIM
    %set_param('QuadSimEnv/SimulinkEnv.slx', 'StopTime', simDur)
    out = sim('QuadSimEnv/SimulinkEnv.slx', 'StopTime', string(simDur));
    save(fullfile(targetFolder, '/simout.mat'), 'out');
    % 
    % %% Convert images
    %Script to import video from file, convert each frame to an image, and save
    %those images to the same folder
    vidFile = fullfile('.', '/camOutput.avi');
    imgFuncs.convertVideo(vidFile,targetFolder);

% end













