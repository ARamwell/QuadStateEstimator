%% Initialise preliminaries
mapfile = './Resources/map.mat';
map = load(fullfile(mapfile));

g = [0 0 -9.81]'; %for simulation
aiding = true;
calibrate = true;%true;
down2kHz =true;
downEkf =true;

% Which EKFs ran
integ_arr = {'rect' 'rect' 'rect' 'rect'};
ekfSize_arr = [10 10 16 16];
alpha_arr = [0 0.99 0 0.99];
numEkfs = size(ekfSize_arr, 2);

%ekfMetricsArr = createArray(numEkfs,0);
clear ekfMetricsArr ekfTempMetricsArr
ekfSearchTerms = {};
for i = 1:numEkfs
    ekfSearchTerms{i} = strcat(string(ekfSize_arr(i)), "el", "_", string(integ_arr(i)), "_a", string(alpha_arr(i)), "_");
end

%% Choose folders
listOfFolderNames =selector_multiFolder(pwd, 'Select sim folders to run EKF on');
numFolders = length(listOfFolderNames);


%% Search for patterns in all folders

%% Extract EKFs
listOfFileNames = {};
for f=1:numFolders
    %% Get current folder
    currentFolder = string(listOfFolderNames(f));
    
    %% There is more than one EKF in each folder
    extraID = "tune1_scaledNoise_bb";
    searchTerm = strcat("ekfResult*", extraID, ".mat");
    fileStruct = dir(fullfile(currentFolder, searchTerm));
    fileTable = struct2table(fileStruct);
    newFileNames = string(fileTable.name);

    % Get full path
    for k=1:length(newFileNames)
        newFileNames(k) = fullfile(currentFolder, string(newFileNames(k)));
    end

    listOfFileNames = [listOfFileNames; newFileNames];

end

%% Now, group EKFs
for i=1:numEkfs

    %% find all ekfResults of this EKF type
    %logicalIndices = contains(listOfFileNames, ekfSearchTerms{i});
    logicalIndices = createArray(1,0);
    for m=1:length(listOfFileNames)
        logicalIndices = [logicalIndices, contains(listOfFileNames{m}, ekfSearchTerms{i})];
    end
    indices = find(logicalIndices, length(listOfFileNames));
    selectListOfFileNames = listOfFileNames(indices);

    %% do some analysis for this type of EKF
    trajErrArr = createArray(2,0);
    simpleErrArr = createArray(ekfSize_arr(i),0);
    percentVioArr = createArray(ekfSize_arr(i),0);
    neesPercArr =  createArray(ekfSize_arr(i)-1,0);
     nisPercArr = createArray(6,0);
     ttcArr = createArray(2,0);
     ttcNeesArr = createArray(1,0);
     maxStableErrArr =createArray(5,0);

    %% init figures
    areFig = figure;
    hold on;
    title(strcat("ARE: ", string(ekfSearchTerms{i}), extraID));
    xlabel("time (s)");
    ylabel("absolute rotation error (degrees)");

    ateFig = figure;
    hold on;
    title(strcat("ATE: ", string(ekfSearchTerms{i}), extraID));
    xlabel("time (s)");
    ylabel("absolute translation error (m)");

    figObj3 = figure;
    hold on;
    title(strcat("NEES: ", string(ekfSearchTerms{i}), extraID));
    xlabel("time (s)");
    ylabel("NEES")

    figObj4 = figure;
    hold on;
    title(strcat("NIS: ", string(ekfSearchTerms{i}), extraID));
    xlabel("time (s)");
    ylabel("NIS")

    %% Analysis

    for j=1:length(selectListOfFileNames)
        ekfResult = load(selectListOfFileNames{j});
        parts = strsplit(string(selectListOfFileNames{j}), '\');
        trajName = string((parts{end-1}));
        trajName = erase(trajName, "sim_");
        
        %*******************************************
        %optionally, scrub of results with more than 1 second of blindndess
        idx = find(ekfResult.timeSinceLastCorrection > 0.4, 1, "first");
        if isempty(idx)
            idx = size(ekfResult.x_, 2);
        end
       
        %*******************************************
        %time to converge
        ttc_are_idx = find(ekfResult.trajErr.AbsoluteError(1:idx,1)'<= 5, 1, "first");%time to converge
        if isempty(ttc_are_idx)
            ttc_are = nan(1,1);
        else
            ttc_are = ekfResult.time(1, ttc_are_idx);
        end
        ttc_ate_idx = find(ekfResult.trajErr.AbsoluteError(1:idx,2)'<= 0.3, 1, "first");
        if isempty(ttc_ate_idx)
            ttc_ate = nan(1,1);
        else
            ttc_ate = ekfResult.time(1, ttc_ate_idx);
        end
        ttc=[ttc_are; ttc_ate];

        ttc_nees = ekfResult.time(:,testNeesConv(ekfResult.nees));

           
        % %remove  first 20 estimates
        % ekfMetrics = ekfMetricsArr(i);
        % [trajErrClean, rmIdx] = rmoutliers(ekfMetrics.trajErr(:,20:end)', "mean");
        % ekfMetrics.trajErr_clean = trajErrClean';
        % ekfMetrics.simpleErr_clean




        %*******************************************
        %nees percent exceed
        alpha = 0.05; %confidence
        numMonteCarloRuns = 1;%;length(selectListOfFileNames);
        stateSize = ekfSize_arr(i)-1;
        chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*stateSize)]/numMonteCarloRuns;
        nees_i =ekfResult.nees(:, 1:idx);
        neesVioHi_idx = find(nees_i > chiSquareLimits(2), size(nees_i,2));
        neesVioLo_idx =find(nees_i < chiSquareLimits(1), size(nees_i,2));
        neesVioEith_idx = unique([neesVioHi_idx, neesVioLo_idx]);
        neesVioPercent = [size(neesVioLo_idx, 2); size(neesVioHi_idx, 2); size(neesVioEith_idx, 2)]/idx;
        nees_i = rmoutliers(nees_i, 'mean', 'ThresholdFactor', 8);
        
        %*******************************************
        %nis percent exceed
        alpha = 0.05; %confidence
        numMonteCarloRuns = 1;%length(selectListOfFileNames);
        measSize = 6;
        if size(ekfResult.nis, 2)< size(ekfResult.x_, 2)
            nis_idx = size(ekfResult.nis, 2);
        else
            nis_idx =idx;
        end
        nis_i = ekfResult.nis(:,1:nis_idx);
        chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*measSize)]/numMonteCarloRuns;
        nisVioHi_idx = find(nis_i > chiSquareLimits(2), size(nis_i,2));
        nisVioLo_idx =find(nis_i < chiSquareLimits(1), size(nis_i,2));
        nisVioEith_idx = unique([nisVioHi_idx, nisVioLo_idx]);
        nisVioPercent = [size(nisVioLo_idx, 2); size(nisVioHi_idx, 2); size(nisVioEith_idx, 2)]/idx;
        nan_idx = ~isnan(nis_i);
        nis_i = nis_i(:,nan_idx);
        nis_i =rmoutliers(nis_i, 'mean', 'ThresholdFactor', 8);
    
        %*******************************************
        % 
        % %max, min, mean error per state
        trajErr_i =ekfResult.trajErr.AbsoluteError(1:idx,:)';
        [vio, x_err]=evalPercentDivergence(ekfResult.x_(:, 1:idx), ekfResult.trueState(:, 1:idx), ekfResult.P(:, :,1:idx), 2);
        velErr_i = abs(vecnorm(x_err(8:10,:), 2, 1));
        velErr_i = rmoutliers(velErr_i, 'mean');
        if size(x_err,1)>10
            baErr_i = abs(vecnorm(x_err(11:13,:), 2, 1));
            bgErr_i = abs(vecnorm(x_err(14:16,:), 2, 1));
        else
            baErr_i = nan(size(x_err, 2));
            bgErr_i= nan(size(x_err, 2));
        end

        %*******************************************
        % get max stabilised error per state
        maxStableErr(1,1) = max(trajErr_i(2,250:end));
        maxStableErr(2,1) = max(trajErr_i(1,250:end));
        maxStableErr(3,1) = max(velErr_i(250:end));
        %if size(ekfMetricsArr(i).simpleErr, 1) > 10
            maxStableErr(4,1) = max(baErr_i(250:end));
            maxStableErr(5,1) = max(bgErr_i(250:end));
        %end

        % % put all vel, all bias together? like position
        % velErr = vecnorm(x_err(8:10,:), 2,2);
        % if stateSize>10
        %     baErr = vecnorm()
           % Get a few extra error metrics

        
    %*******************************************
    %build arrays
        trajErrArr = [trajErrArr, trajErr_i];
        simpleErrArr = [simpleErrArr, x_err];
        percentVioArr = [percentVioArr, vio];
        ttcArr = [ttcArr, ttc];
        nisPercArr = [nisPercArr, nisVioPercent];
        neesPercArr = [neesPercArr, neesVioPercent];
        ttcNeesArr = [ttcNeesArr, ttc_nees];
        maxStableErrArr = [maxStableErrArr, maxStableErr];
        % 

        %*******************************************
        %plot
        figure(areFig);
        plot(ekfResult.time(:, 1:idx),  ekfResult.trajErr.AbsoluteError(1:idx,1)', 'DisplayName', trajName);
        hold on;

        figure(ateFig);
        plot(ekfResult.time(:, 1:idx),  ekfResult.trajErr.AbsoluteError(1:idx,2)','DisplayName', trajName);
        hold on;

        figure(figObj3);
        plot(ekfResult.time(:, 1:idx),  ekfResult.nees(:, 1:idx)','DisplayName', trajName);
        hold on;


        figure(figObj4);
        plot(ekfResult.time(:, nan_idx),  ekfResult.nis(:, nan_idx)','DisplayName', trajName);
        hold on;
        
    end
    % 
    saveDest ="C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\ekfComparison2\badBias\";
    formatFigForLatex(areFig);
    formatFigForLatex(ateFig);
    formatFigForLatex(figObj3);
    formatFigForLatex(figObj4);
    name_areFig = strcat(saveDest, "are_combined_", ekfSearchTerms{i}, extraID, ".fig");
    name_ateFig = strcat(saveDest, "ate_combined_", ekfSearchTerms{i}, extraID, ".fig");
     savefig(areFig, name_areFig);
     savefig(ateFig, name_ateFig);

    
    ekfMetrics.trajErr = trajErrArr;
    ekfMetrics.simpleErr = simpleErrArr;
    ekfMetrics.percentVio = percentVioArr;
    ekfMetrics.percentNees = neesPercArr;
    ekfMetrics.percentNis = nisPercArr;
    ekfMetrics.ttc = ttcArr;
    ekfMetrics.ekfType = ekfSearchTerms{i};
    ekfMetrics.maxStableErr = maxStableErrArr;

    ekfMetricsArr(i) = ekfMetrics;

    ekfTempMetricsArr(i).nees = nees_i;
    ekfTempMetricsArr(i).nis = nis_i;
    ekfTempMetricsArr(i).ekfType = ekfSearchTerms{i};
    ekfTempMetricsArr(i).ttc_nees = ttcNeesArr;

end

mean(ekfMetricsArr(1).trajErr(1,:))

mean(ekfMetricsArr(1).trajErr(2,:))
%%
%neesnisViolin(ekfTempMetricsArr);


%%Plot some violin plots for nees and nis
function neesnisViolin(metricsArr)
    
    clear dynArgs_nees dynArgs_nis 
    cut = 0; %how many readings to cut (to account for bad initialisation)
       
    %prepare data for box plot 
    for i=1:size(metricsArr,2)
        nees_i = metricsArr(i).nees;
        nis_i = metricsArr(i).nis;
        group_i = metricsArr(i).ekfType;
       
        dynArgs_nees{i*2-1} = nees_i;
        dynArgs_nees{i*2} = group_i;
        dynArgs_nis{i*2-1} = nis_i;
        dynArgs_nis{i*2} = group_i;
    end

    dynArgs_nees{end+1} = 150;
    dynArgs_nees{end+1} = 'NEES';
    dynArgs_nis{end+1} = 150;
    dynArgs_nis{end+1} = 'NIS';

    plotViolin(dynArgs_nees{:})
    plotViolin(dynArgs_nis{:})

end






% %% Other metrics
% numEkfs = size(ekfMetricsArr, 1);
% for i= 1:numEkfs
%     %remove outliers and first 20 measurements
%     ekfMetrics = ekfMetricsArr(i);
%     [trajErrClean, rmIdx] = rmoutliers(ekfMetrics.trajErr(:,20:end)', "mean");
%     ekfMetrics.trajErr_clean = trajErrClean';
%     ekfMetrics.simpleErr_clean
% 
%     %time to converge
%     ttc_are_idx = find(trajErr(1,:)<= 5, 1, "first");%time to converge
%     ttc_are = ekfResult.time(1, ttc_are_idx);
%     ttc_ate_idx = find(trajErr(2,:)<= 0.2, 1, "first");
%     ttc_ate = ekfResult.time(1, ttc_ate_idx);
%     ekfMetrics.ttc = [ttc_are, ttc_ate];
% 
%     %find some more metrics
%     ekfMetrics.meanErr = 
% 
%     %nees percent exceed
% 
% 
%     %nis percent exceed
% 
% 
%     %vio percent
% 
% 
%     %max, min, mean error per state
%     % put all vel, all bias together? like position
% 
% end

%
% for i=2:size(ekfMetricsArr, 2)
%     trueNis =downsample(ekfMetricsArr(i).percentNis', 2, 1) ;
%     ekfMetricsArr(i).percentNis = trueNis';
%     trueNees =downsample(ekfMetricsArr(i).percentNees', 2, 1) ;
%     ekfMetricsArr(i).percentNees = trueNees';
% end


%% effective ttc
% 
% for i=1:size(ekfMetricsArr, 2)
%     ttc_eff = 
% 
% end 

function [idx_conv] = testNeesConv(nees)
    %get rolling window average
    nees_avg = movmean(nees, 10);

    %check convergence
    converged = false;
    conv_cnt = 0;
    dnees = createArray(1,size(nees,2));
    dnees(1) = 0;
    
    %we consider that the filter has converged if nees changes by less than 5%
    for i=2:size(nees_avg,2)
        dnees(i) = (norm(nees_avg(i)-nees_avg(i-1)))/norm(nees_avg(i-1));
        conv_i =false;       
        if dnees(i) < 0.01
            conv_i = true;
            if conv_prev == true
                conv_cnt = conv_cnt+1;
            else
                conv_cnt = 1;
            end
        end
        if conv_cnt ==1
            idx_conv = i;
        end
        if conv_cnt>=500
            converged = true;
        end
        if converged
            break;
        end
        conv_prev = conv_i;
    end
    if ~converged
        idx_conv = [];
    end
end