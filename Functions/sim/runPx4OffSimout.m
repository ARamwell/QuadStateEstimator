

%% Initialise preliminaries
g = [0 0 -9.81]'; %for simulation
biasPerturb = true;
invKa = inv(simset.accelCalib.scale);
invKb =inv(simset.gyroCalib.scale);

%% Choose folders
listOfFolderNames =selector_multiFolder(pwd, 'Select sim folders to run EKF on');
numFolders = length(listOfFolderNames);

%%
for f=1:numFolders
    clear px4Result groundTruth

    %% Get current folder
    %figure;
    currentFolder = string(listOfFolderNames(f));
    parts = strsplit(currentFolder, '\');
    trajName = string((parts{end}));

    simoutfilepath = fullfile(strcat(currentFolder, '\simout.mat'));
    simsetfilepath = fullfile(strcat(currentFolder, '\simset.mat'));
    imgfilepath = currentFolder;

    %% Import data
    simin = load(simoutfilepath);
    if length(fieldnames(simin)) == 1
        simin = simin.simout;
    end
    
    simset = load(simsetfilepath);
    if length(fieldnames(simset)) == 1
        simset = simset.simset;
    end


    %% get timeseries data out
    
    %weirdly, simulink won't accept NaNs in a timeseries... so we need to
    %replace them with something else obvious - all zeroes
    p3pData = simin.p3p_selected.signals.values;
    %idx_nan = isnan(p3pData(:,1));
    for i = 1:size(p3pData, 1)
        if isnan(p3pData(i,1))
            p3pData(i,:) = [0 0 0 0 0 0 0];
        end
    end
    

    if biasPerturb == true
        %bias perturbment
        % Example: Create a sample array
        A = [-1, -0.9, -0.8, -0.7, -0.6, -0.5, 0.5, 0.6, 0.7, 0.8, 0.9, 1]; 
        numSamples = 6; 
        % Generate a vector of 'numSamples' random indices (integers between 1 and length(A))
        randomIndexes = randi([1, numel(A)], 1, numSamples); %
        % Select the elements using the indices
        selectedElements = A(randomIndexes);
        bg_perturb = invKa*0.05*selectedElements(1:3)';
        ba_perturb = invKb*0.15*selectedElements(4:6)';
    else
        bg_perturb = zeroes(3,1);
        ba_perturb = zeroes(3,1);
    end
    u_a = simin.IMU.signals.values(:, 4:6)'-ba_perturb;
    u_g = simin.IMU.signals.values(:, 1:3)'+bg_perturb;
    
    u_a_ts = timeseries(u_a, simin.IMU.time);
    u_g_ts = timeseries(u_g, simin.IMU.time);
    p3p_ts = timeseries(p3pData, simin.p3p_selected.time);
    
    
    %% run sim
    out = sim('Other/sitlFromRecordedData.slx', 'StopTime', string(simset.duration)); %run the sim
    
    %% process results
    px4LogFile_struct = findNewestPx4Log('\\wsl.localhost\Ubuntu-22.04\home\alyssa\PX4-Autopilot\build\px4_sitl_default\log');
    px4LogFile = strcat(px4LogFile_struct.folder, "\", px4LogFile_struct.name);

    %% PROCESS SIM DATA
    [groundTruth, imuData,ekfResult, p3pResult] = processSimData(simin, 0, 0); %make simout more usable and readable
    %%add perturbment to ground truth
    if size(groundTruth.quad.state,1)>10
            for g=1:size(groundTruth.quad.state,1,2)
                groundTruth.quad.state(11:16,g) = groundTruth.quad.state(11:16,g) +[ba_perturb; bg_perturb];
            end
        end
    px4Result = processPx4Data(px4LogFile, simset.aidingActive, groundTruth); %import ulogs into readable and useful format\

    %% DO COMPARISONS, MAKE GRAPHS
    % p3pName = strcat('p3pResult_', trajNames(i), '.mat');
    % p3pFile = strcat(p3pFolder, '\', p3pName);
    % save(p3pFile, 'p3pResult.mat', '-struct');
    saveFolder = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\comparison\px4_default_badBias";
    px4File = strcat(saveFolder, '\px4Result_gb_bb', trajName, '.mat');
    save(px4File, '-struct', 'px4Result');
    

end
