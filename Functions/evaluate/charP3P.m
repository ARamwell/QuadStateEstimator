clear lowErr midErr highErr allErr allResid
lowErr = createArray(3,0, 'double');
midErr = createArray(3,0, 'double');
highErr = createArray(3,0, 'double');
allResid = createArray(7,0, 'double');

%% Find all subfolders
%listOfFolders = selector_multiFolder;\
% add function to search for appropriate p3presults



%% Or all files
listOfFiles = selector_multiFile();
numberOfFiles = length(listOfFiles);
parentFolder = 'C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\p3pResults';


%% Calc error
for i=1:numberOfFiles
    currentFile = string(listOfFiles(i));
    %currentFolder = fullfile('C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\pitchforward_high1\sim_16Hz');
    
    p3pData = load(fullfile(currentFile));
    numPoses = size(p3pData.p3pResult.truePose, 2);
    resid = createArray(7, numPoses);

    for d = 1:numPoses
        [~, ~, p3pData.p3pResult.err(:,d)] = getPoseError(p3pData.p3pResult.truePose(:,d), p3pData.p3pResult.selected(:, d));
        resid(:,d) = p3pData.p3pResult.selected(:, d) - p3pData.p3pResult.truePose(:,d);
        if norm(resid(4:7,d))>1
            resid(4:7,d) = p3pData.p3pResult.selected(4:7, d) + p3pData.p3pResult.truePose(4:7,d);
        end
    end


    allResid = [allResid, resid];
    if contains(currentFile, 'low')
        % lowErr(:,end) = p3pData.p3pResult.KneipN.best.err;
        lowErr = [lowErr, [p3pData.p3pResult.err; p3pData.p3pResult.truePose]];
    elseif contains(currentFile, 'mid')
        midErr = [midErr, [p3pData.p3pResult.err; p3pData.p3pResult.truePose]];
    elseif contains(currentFile, 'high')
        highErr = [highErr, [p3pData.p3pResult.err; p3pData.p3pResult.truePose]]; 
    end


  
end


allErr = [lowErr, midErr, highErr];

lowErr_pos_mean = mean(abs(lowErr(1,:)), 'omitmissing');
lowErr_pos_std = std(abs(lowErr(1,:)), 'omitmissing');
lowErr_orient_mean = mean(abs(lowErr(2,:)), 'omitmissing');
lowErr_orient_std = std(abs(lowErr(2,:)), 'omitmissing');

midErr_pos_mean = mean(abs(midErr(1,:)), 'omitmissing');
midErr_pos_std = std(abs(midErr(1,:)), 'omitmissing');
midErr_orient_mean = mean(abs(midErr(2,:)), 'omitmissing');
midErr_orient_std = std(abs(midErr(2,:)), 'omitmissing');

highErr_pos_mean = mean(abs(highErr(1,:)), 'omitmissing');
highErr_pos_std = std(abs(highErr(1,:)), 'omitmissing');
highErr_orient_mean = mean(abs(highErr(2,:)), 'omitmissing');
highErr_orient_std = std(abs(highErr(2,:)), 'omitmissing');

allErr_pos_mean = mean(abs(allErr(1,:)), 'omitmissing');
allErr_pos_std = std(abs(allErr(1,:)), 'omitmissing');
allErr_orient_mean = mean(abs(allErr(2,:)), 'omitmissing');
allErr_orient_std = std(abs(allErr(2,:)), 'omitmissing');

save(fullfile(parentFolder, '/p3pError.mat'), 'lowErr', 'midErr', 'highErr')

%%


%target_pos = [-0.106090000000000;	-0.149490000000000;	0.00200000000000000];
target_pos = [0; 0;	0];

horiz = createArray(1, size(allErr, 2));
vert = createArray(1, size(allErr, 2));
angle = createArray(1, size(allErr, 2));

 
for k=1:size(allErr, 2)

    x_k = allErr(3, k);
    y_k = allErr(4, k);
    z_k = allErr(5, k);

   % q_k = allErr(6:end, k);

    %calculate horizontal distance from target
    horiz(k) = sqrt( (x_k - target_pos(1))^2 + (y_k - target_pos(2))^2);
        
    %calculate vertical distance from target
    vert(k) = abs(z_k - target_pos(3));

    % %calculate "off angle"
    % %R_k = quat2rotm(q_k');
    % %z_vec = R_k(:, 3);
    % %angle(k)=acos(dot(z_vec, [0; 0; 1])/norm(dot(z_vec, [0; 0; 1])));
    % angle(k) = abs(quat2angle(q_k'));



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
xlim([0, 2.5]);
ylim([0, 2]);
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
clim([2, 20]);
xlim([0, 2.5]);
ylim([0, 2]);
cb.Label.String = 'Orientation error (deg)';
cb.Label.FontSize =  12;
%cm = colormap('gray');
%cm = cm(0:0/)
colormap hot;


%% How about 3d plots?
% %% Plot position error
% figure;
% scatter3(horiz, vert, angle, [], allErr(1,:), 'filled'); %'MarkerEdgeColor',[0 0 0] , 'LineWidth',0.5
% title('P3P pose estimate error vs robot distance from target', 'FontSize', 14)
% xlabel('Horizontal distance from target (actual, m)', 'FontSize', 12)
% ylabel('Vertical distance from target (actual, m)', 'FontSize', 12)
% cb=colorbar;
% hold on;
% clim([0, 0.3])
% cb.Label.String = 'Position error (m)';
% cb.Label.FontSize =  12;
% %cm = colormap('gray');
% %cm = cm(0:0/)
% colormap hot;
% 
% %% Plot orientation error
% figure;
% scatter3(horiz, vert, angle, [], allErr(2,:), 'filled'); %'MarkerEdgeColor',[0 0 0] , 'LineWidth',0.5
% title('P3P pose estimate error vs robot distance from target', 'FontSize', 14)
% xlabel('Horizontal distance from target (actual, m)', 'FontSize', 12)
% ylabel('Vertical distance from target (actual, m)', 'FontSize', 12)
% cb=colorbar;
% hold on;
% clim([2, 20])
% cb.Label.String = 'Orientation error (deg)';
% cb.Label.FontSize =  12;
% %cm = colormap('gray');
% %cm = cm(0:0/)
% colormap hot;
% 
% 
% 
% function [posDiff] = getAvgPosErrPerAxis(listOfFolderNames)
% 
%     posDiff = createArray(3,0);
%     numberOfSubFolders = length(listOfFolderNames);
% 
%     for i=1:numberOfSubFolders
%         currentFolder = string(listOfFolderNames(i));
%               p3pData = load(fullfile(currentFolder, '/p3pResult.mat'));
% 
%               posDiff = [posDiff (p3pData.p3pResult.KneipN.best.truePose(1:3,:) - p3pData.p3pResult.KneipN.best.quadPose(1:3, :))];
% 
%     end
% end
    

  

function calcMeasCov(resid)

    numEl = size(resid, 1);
    numSamp = size(resid, 2);

    mu = mean(resid, 2);
   % Rhat = zeros(numEl, numEl);

    Rhat = cov(resid.', 1);

    %for k=1:numSamp

end

