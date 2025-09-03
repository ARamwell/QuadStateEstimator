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
    if ~contains(singleSubFolder, 'arb') && ~contains(singleSubFolder, 'calib') && contains(singleSubFolder, 'sim_16')
	    listOfFolderNames = [listOfFolderNames singleSubFolder];
    end
end
numberOfSubFolders = length(listOfFolderNames);

basename = 'ekfResult_10el_trap_reorient';
filename = strcat('\', basename, '.mat');
combinedData = createArray(4,0);
scrubbedData = createArray((4+4+4),0);

cmap = turbo(numberOfSubFolders-1);

%% Prep scrubbed figures
%POSITION
scrubFig_pos = figure();
scrubAx_pos = axes('Parent', scrubFig_pos);
title({'Position error over time for various trials:', '16-element EKF, rectangular  integration', 'real data, reoriented','scrubbed of image loss > 1s'}, 'FontSize', 14);
xlabel('Time since initialisation (s)', 'FontSize', 12);
ylabel('Position error (m)', 'FontSize', 12);
hold on;
%ORIENTATION
scrubFig_orient = figure();
scrubAx_orient = axes('Parent', scrubFig_orient);
title({'Orientation error over time for various trials:', '16-element EKF,  rectangular integration', 'real data, reoriented','scrubbed of image loss > 1s'}, 'FontSize', 14);
xlabel('Time since initialisation (s)', 'FontSize', 12);
ylabel('Orientation error (degrees)', 'FontSize', 12);
hold on;

%% Prep combined figures
%POSITION
combFig_pos = figure();
combAx_pos = axes('Parent', combFig_pos);
title({'Position error over time for various trials:', '16-element EKF, rectangular integration', 'real data, reoriented'}, 'FontSize', 14);
xlabel('Time since initialisation (s)', 'FontSize', 12);
ylabel('Position error (m)', 'FontSize', 12);
hold on;
%ORIENTATION
combFig_orient = figure();
combAx_orient = axes('Parent', combFig_orient);
title({'Orientation error over time for various trials:', '16-element EKF, rectangular integration', 'real data, reoriented'}, 'FontSize', 14);
xlabel('Time since initialisation (s)', 'FontSize', 12);
ylabel('Orientation error (degrees)', 'FontSize', 12);
hold on;

for k=2:numberOfSubFolders
    currentFolder = string(listOfFolderNames(k));

    ekfData = load(fullfile(currentFolder, filename));
    newData_scrubbed = createArray((4+4+4),0);
    newData_all = createArray(4,0);
    scrubbed = false;

    for c=1:size(ekfData.ekfResult.elapsedTime, 2)
        if ekfData.ekfResult.timeSinceLastCorrection(1,c) > 1 && ~scrubbed
            scrubbed = true;
        end
        if ~scrubbed
            newData_scrubbed = [newData_scrubbed, [ekfData.ekfResult.elapsedTime(:,c); ekfData.ekfResult.error(:,c); ekfData.ekfResult.timeSinceLastCorrection(:,c); ekfData.ekfResult.stateEst(4:7,c); ekfData.ekfResult.truePose(4:7,c)]];
        end
    end
    
    newData_all = [ekfData.ekfResult.elapsedTime; ekfData.ekfResult.error; ekfData.ekfResult.timeSinceLastCorrection];

    scrubbedData = [scrubbedData, newData_scrubbed];
    combinedData = [combinedData, newData_all];

    %% plot
    %scrubbed position error
    scatter(scrubAx_pos, newData_scrubbed(1,:),  newData_scrubbed(2,:), 15, cmap(k-1,:), 'filled');
    %plot(scrubAx_pos, newData_scrubbed(1,:),  newData_scrubbed(2,:), 'Color', cmap(k-1,:));
    hold on;
    %scrubbed orientation error
    scatter(scrubAx_orient, newData_scrubbed(1,:),  newData_scrubbed(3,:), 15, cmap(k-1,:), 'filled');
    %plot(scrubAx_pos, newData_scrubbed(1,:),  newData_scrubbed(2,:), 'Color', cmap(k-1,:));
    hold on;

    %combined position error
    scatter(combAx_pos, newData_all(1,:),  newData_all(2,:), 15, cmap(k-1,:), 'filled');
    %plot(combAx_pos, newData_all(1,:),  newData_all(2,:), 'Color', cmap(k-1,:));
    hold on;
    scatter(combAx_orient, newData_all(1,:),  newData_all(3,:), 15, cmap(k-1,:), 'filled');
    %plot(combAx_pos, newData_all(1,:),  newData_all(2,:), 'Color', cmap(k-1,:));
    hold on;

end


%% draw means
%scrubbed position
[scrubbedData_pos_means_t, scrubbedData_pos_means] = binMean(scrubbedData(1,:), scrubbedData(2,:)) ;
plot(scrubAx_pos, scrubbedData_pos_means_t, scrubbedData_pos_means, 'Color', '0 0 0' , 'LineWidth', 2);
scrubbedData_pos_mean = mean(scrubbedData(2,:));
scrubbedData_pos_std = std(scrubbedData(2,:));
dispText(scrubAx_pos, scrubbedData_pos_mean,  scrubbedData_pos_std )
hold on;
%scrubbed orientation
[scrubbedData_orient_means_t, scrubbedData_orient_means] = binMean(scrubbedData(1,:), scrubbedData(3,:)) ;
plot(scrubAx_orient, scrubbedData_orient_means_t, scrubbedData_orient_means, 'Color', '0 0 0' , 'LineWidth', 2);
scrubbedData_orient_mean = mean(scrubbedData(3,:));
scrubbedData_orient_std = std(scrubbedData(3,:));
dispText(scrubAx_orient, scrubbedData_orient_mean,  scrubbedData_orient_std )
hold on;
hold off;
drawnow;



%all position
[combinedData_pos_means_t, combinedData_pos_means] = binMean(combinedData(1,:), combinedData(2,:)) ;
plot(combAx_pos, combinedData_pos_means_t, combinedData_pos_means, 'Color', '0 0 0' , 'LineWidth', 2);
combinedData_pos_mean = mean(combinedData(2,:));
combinedData_pos_std = std(combinedData(2,:));
dispText(combAx_pos, combinedData_pos_mean,  combinedData_pos_std )
hold on;
all orientation
[combinedData_orient_means_t, combinedData_orient_means] = binMean(combinedData(1,:), combinedData(3,:)) ;
plot(combAx_orient, combinedData_orient_means_t, combinedData_orient_means, 'Color', '0 0 0' , 'LineWidth', 2);
combinedData_orient_mean = mean(combinedData(3,:));
combinedData_orient_std = std(combinedData(3,:));
dispText(combAx_orient, combinedData_orient_mean,  combinedData_orient_std )
hold on;
hold off;
drawnow; 



%% save plots
savePlots(scrubFig_pos, parentFolder, strcat(basename, '_scrubbedPos'));
savePlots(scrubFig_orient, parentFolder, strcat(basename, '_scrubbedOrient'));
savePlots(combFig_pos, parentFolder, strcat(basename, '_allPos'));
savePlots(combFig_orient, parentFolder, strcat(basename, '_allOrient'));

close all;

%% functions

% 
% figure();
% scatter(combinedData(1,:), combinedData(2,:));
% 
% mean(combinedData(2,:))
% 
% figure();
% scatter(scrubbedData(1,:), scrubbedData(2,:));
% 
% drawnow


%% create mean line
function [mx, my] = binMean(x, y)
    %create bins
    bin_dx = 0.1;
    bin_edges = min(x):bin_dx:max(x);
    binCenters = bin_edges(1:end-1) + bin_dx/2;
    
    % assign scrubbed data to bins
    [~, ~, binIdx] = histcounts(x, bin_edges);
    % meanError = accumarray(binIdx, y, [], @mean, NaN);  % NaN if bin is empty
    

    meanError = NaN(size(binCenters));

    for i = 1:length(binCenters)
        % Find data points in this bin
        inBin = x >= bin_edges(i) & x < bin_edges(i+1);
        if any(inBin)
            meanError(i) = mean(y(inBin));
        end
    end


    mx = binCenters;
    my = meanError;
end


function dispText(ax, meanval, stdval)
    %%add mean and std text
    str = sprintf('Mean Error: %.3f\nStd Dev: %.3f', meanval, stdval);
    % Get axis limits
    xLimits = xlim(ax);
    yLimits = ylim(ax);
    % Define position near top-right (adjust offset as needed)
    xPos = xLimits(2) - 0.05*(xLimits(2)-xLimits(1));
    yPos = yLimits(2) - 0.05*(yLimits(2)-yLimits(1));
    %txt = char(strcat('Mean: ', string(scrubbedData_pos_mean)));
    text(ax,xPos, yPos, str, ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', ...
        'FontSize', 12, ...
        'BackgroundColor', 'white', ...
        'EdgeColor', 'black', ...
        'Margin', 4);
    hold on;
end

function savePlots(fig, destFolder, name)
    exportgraphics(fig, fullfile(destFolder, strcat(name, '.emf')));
    savefig(fig, fullfile(destFolder,  strcat(name, '.fig')));
end