

[simset, parentFolder] = simGetSettings(); %get user input for type of sim to run
[trajList, trajNames, simset] = simGetTraj(simset, 1);%get user input on trajectories to run
if simset.runEstimator == true
    estset = estGetSettings();
end

%run the simulations for all trajectories
for i = 1: size(trajList, 3)
    %% RUN SIM
    traj = trajList(:,:, i);
    simInit;%(simset, ); %init sim for this run
    out = sim('QuadSimEnv/SimulinkEnv.slx', 'StopTime', string(simset.duration)); %run the sim
    [saveFolder, px4LogFile] = simSave(parentFolder, trajNames(i), out, simset); %saves simset, out, PX4 logs, and captured images to parentFolder/trajName

    %% PROCESS SIM DATA
    [ekfResult, p3pResult, groundTruth] = processSimData(out, simset); %make simout more usable and readable
    px4Result = processPx4Data(px4LogFile, simset.aidingActive, groundTruth); %import ulogs into readable and useful format\

    %% DO COMPARISONS, MAKE GRAPHS




end

%


