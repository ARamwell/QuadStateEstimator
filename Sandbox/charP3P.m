clear lowErr midErr highErr allErr
lowErr = createArray(3,0, 'double');
midErr = createArray(3,0, 'double');
highErr = createArray(3,0, 'double');

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
	if ~contains(singleSubFolder, 'arb') && ~contains(singleSubFolder, 'calib') && ~contains(singleSubFolder, 'sim')
	    listOfFolderNames = [listOfFolderNames singleSubFolder];
    end
    
end
numberOfSubFolders = length(listOfFolderNames);

%for i=2:numberOfSubFolders
    %currentFolder = string(listOfFolderNames(i));
    currentFolder = fullfile('C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\pitchforward_high1\sim_16Hz');
    
    p3pData = load(fullfile(currentFolder, '/p3pResult_realigned.mat'));
    
    if contains(currentFolder, 'low')
        % lowErr(:,end) = p3pData.p3pResult.KneipN.best.err;
        lowErr = [lowErr, [p3pData.p3pResult.KneipN.best.err; p3pData.p3pResult.KneipN.best.truePose]];
    elseif contains(currentFolder, 'mid')
        midErr = [midErr, [p3pData.p3pResult.KneipN.best.err; p3pData.p3pResult.KneipN.best.truePose]];
    elseif contains(currentFolder, 'high')
        highErr = [highErr, [p3pData.p3pResult.KneipN.best.err; p3pData.p3pResult.KneipN.best.truePose]]; 
    end
%end

allErr = [lowErr, midErr, highErr];

lowErr_pos_mean = mean(lowErr(1,:));
lowErr_pos_std = std(lowErr(1,:));
lowErr_orient_mean = mean(lowErr(2,:));
lowErr_orient_std = std(lowErr(2,:));

midErr_pos_mean = mean(midErr(1,:));
midErr_pos_std = std(midErr(1,:));
midErr_orient_mean = mean(midErr(2,:));
midErr_orient_std = std(midErr(2,:));

highErr_pos_mean = mean(highErr(1,:));
highErr_pos_std = std(highErr(1,:));
highErr_orient_mean = mean(highErr(2,:));
highErr_orient_std = std(highErr(2,:));

allErr_pos_mean = mean(allErr(1,:));
allErr_pos_std = std(allErr(1,:));
allErr_orient_mean = mean(allErr(2,:));
allErr_orient_std = std(allErr(2,:));

save(fullfile(parentFolder, '/p3pError.mat'), 'lowErr', 'midErr', 'highErr')

%%


checker_pos = [-0.106090000000000;	-0.149490000000000;	0.00200000000000000];

horiz = createArray(1, size(allErr, 2));
vert = createArray(1, size(allErr, 2));

 
for k=1:size(allErr, 2)

    x_k = allErr(4, k);
    y_k = allErr(5, k);
    z_k = allErr(6, k);

    %calculate horizontal distance from target
    horiz(k) = sqrt( (x_k - checker_pos(1))^2 + (y_k - checker_pos(2))^2);
        
    %calculate vertical distance from target
    vert(k) = z_k - checker_pos(3);
end


%% Plot position error
figure;
scatter(horiz, vert, [], allErr(1,:), 'filled'); %'MarkerEdgeColor',[0 0 0] , 'LineWidth',0.5
title('P3P pose estimate error vs robot distance from target', 'FontSize', 14)
xlabel('Horizontal distance from target (actual, m)', 'FontSize', 12)
ylabel('Vertical distance from target (actual, m)', 'FontSize', 12)
cb=colorbar;
hold on;
clim([0, 0.3])
cb.Label.String = 'Position error (m)';
cb.Label.FontSize =  12;
%cm = colormap('gray');
%cm = cm(0:0/)
colormap hot;

%% Plot orientation error
figure;
scatter(horiz, vert, [], allErr(2,:), 'filled'); %'MarkerEdgeColor',[0 0 0] , 'LineWidth',0.5
title('P3P pose estimate error vs robot distance from target', 'FontSize', 14)
xlabel('Horizontal distance from target (actual, m)', 'FontSize', 12)
ylabel('Vertical distance from target (actual, m)', 'FontSize', 12)
cb=colorbar;
hold on;
clim([2, 20])
cb.Label.String = 'Orientation error (deg)';
cb.Label.FontSize =  12;
%cm = colormap('gray');
%cm = cm(0:0/)
colormap hot;




function [posDiff] = getAvgPosErrPerAxis(listOfFolderNames)

    posDiff = createArray(3,0);
    numberOfSubFolders = length(listOfFolderNames);
    
    for i=1:numberOfSubFolders
        currentFolder = string(listOfFolderNames(i));
              p3pData = load(fullfile(currentFolder, '/p3pResult.mat'));

              posDiff = [posDiff (p3pData.p3pResult.KneipN.best.truePose(1:3,:) - p3pData.p3pResult.KneipN.best.quadPose(1:3, :))];

    end
end
    

  



