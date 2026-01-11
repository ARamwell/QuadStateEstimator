srcFolder = "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-01-07_12-31-15\sim_pitch_mid1\";


%import simout
simin_file = strcat(srcFolder, "simout.mat");
simin = load(simin_file);
if length(fieldnames(simin)) == 1
    simin = simin.simout;
end

%and sim settings
simset_file = strcat(srcFolder, "simset.mat");
simset = load(simset_file);
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


u_a_ts = timeseries(simin.IMU.signals.values(:, 4:6)', simin.IMU.time);
u_g_ts = timeseries(simin.IMU.signals.values(:, 1:3)', simin.IMU.time);
p3p_ts = timeseries(p3pData, simin.p3p_selected.time);


%% run sim
out = sim('Other/sitlFromRecordedData.slx', 'StopTime', string(simset.duration)); %run the sim

%% process results
px4LogFile ="\\wsl.localhost\Ubuntu-22.04\home\alyssa\PX4-Autopilot\build\px4_sitl_default\log\2026-01-11\15_30_10.ulg";
[groundTruth, imuData,ekfResult, p3pResult] = processSimData(simin, 0, 0); %make simout more usable and readable
px4Result = processPx4Data(px4LogFile, simset.aidingActive, groundTruth); %import ulogs into readable and useful format\

px4LogFile2= "C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\sim_2026-01-07_12-31-15\sim_pitch_mid1\11_05_57.ulg";
px4Result2 = processPx4Data(px4LogFile2, simset.aidingActive, groundTruth); %import ulogs into readable and useful format\
