function [groundTruth, imuData, p3pResult, simset] = importSimData(currentFolder, diffImuHz, aiding, map)
%IMPORTSIMDATA Function to import data from QuadSimEnv, and downsample if
%needed. 
%   Detailed explanation goes here

    %% get file paths
    simoutfilepath = fullfile(strcat(currentFolder, '\simout.mat'));
    simsetfilepath = fullfile(strcat(currentFolder, '\simset.mat'));
    imgfilepath = currentFolder;

    %% Import

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


    %% Get data in usable format

    [groundTruth, imuData, ~, ~ ] = processSimData(simout, 0, 0, diffImuHz);
    if aiding
        p3pResult = runP3pOnFile(imgfilepath,  simset.camParams.K, p3pFuncs.invertT(map.worldObjectStruct.transforms.T_gencam2genquad));
    end

   
end

