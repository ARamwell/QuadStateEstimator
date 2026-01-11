[simset, parentSaveFolder] = simGetSettings(); %get user input for type of sim to run
%[trajList, trajNames, simset] = simGetTraj(simset, "spiral");%get user input on trajectories to run
[trajList, trajNames, simset] = simGetWP(simset,"C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\RobMech\Dynamic\TestSeries_3\rollleft_high2");%get user input on trajectories to run
if simset.runEstimator == true
    estset = estGetSettings();
end

p3pFolder = ('C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\p3pResults');
%%
%run the simulations for all trajectories
for i = 1: size(trajList, 3)
    %% RUN SIM
    trajWP = trajList(:,:, i);
    traj = simGetTrajFromWP(trajWP, simset, 0);
    simInit;%(simset, ); %init sim for this run
    out = sim('QuadSimEnv/SimulinkEnv.slx', 'StopTime', string(simset.duration)); %run the sim
    [saveFolder, px4LogFile] = simSave(parentSaveFolder, trajNames(i), out, simset); %saves simset, out, PX4 logs, and captured images to parentSaveFolder/trajName

    %% PROCESS SIM DATA
    [groundTruth, imuData,ekfResult, p3pResult] = processSimData(out, estset.runEKF, estset.runP3P); %make simout more usable and readable
    px4Result = processPx4Data(px4LogFile, simset.aidingActive, groundTruth); %import ulogs into readable and useful format\

    %% DO COMPARISONS, MAKE GRAPHS
    % p3pName = strcat('p3pResult_', trajNames(i), '.mat');
    % p3pFile = strcat(p3pFolder, '\', p3pName);
    % save(p3pFile, 'p3pResult.mat', '-struct');
    px4File = strcat(saveFolder, '\px4Result_gb_', trajNames(i), '.mat');
    save(px4File, '-struct', 'px4Result');


end

%

