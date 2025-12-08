% Script to plot IMU readings over time and calculate Gaussian fit

%% %Select a .mat file
[filename, pathname] = uigetfile('*.mat', 'Select a MAT file to import');
if isequal(filename,0)
    disp('User canceled file selection.');
    return;
end
filepath = fullfile(pathname, filename);

%% Import data
imuData = load(filepath);

if isstruct(imuData)
    fields = fieldnames(imuData);
    imuData = getfield(imuData, fields{1});
end

if (size(imuData, 2) > 6)
    imuData = imuData';
end

%% Init plot
figObj = figure();
hold on;
tileLayoutObj = tiledlayout(figObj, 3, 1, "TileSpacing", "tight");
xlabel(tileLayoutObj, '$\mathbf{sample\ number}$', 'fontweight', 'bold',  'Interpreter', 'latex');
ylabel(tileLayoutObj,'$\mathbf{raw\ measured\ values\ (rad/s)}$', 'fontweight', 'bold', 'Interpreter', 'latex');


%% Do plotting
titles = {'x-channel', 'y-channel', 'z-channel'};
for i = 1:3 %for each channel 
    
    channelData = imuData(:,i);
    channelData_clean = rmoutliers(channelData, 'mean', 'ThresholdFactor', 6);
    nexttile;
    plotGaussNoise(channelData_clean, figObj, '', '', titles(i));

end

formatFigForLatex(figObj);



