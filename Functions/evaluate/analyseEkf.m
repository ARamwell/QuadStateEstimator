function [ekfMetrics, areFig, ateFig, neesFig, nisFig] = analyseEkf(ekfSize,ekfName,trajFolderList, extraID, doScrub)
%ANALYSEEKF Summary of this function goes here
%   Detailed explanation goes here

    %% do some analysis for this type of EKF
    trajErrArr = createArray(2,0);
    simpleErrArr = createArray(ekfSize,0);
    percentVioArr = createArray(ekfSize,0);
    neesPercArr =  createArray(ekfSize-1,0);
     nisPercArr = createArray(6,0);
     ttcArr = createArray(2,0);
     ttcNeesArr = createArray(1,0);
     maxStableErrArr =createArray(5,0);

      %% init figures
    areFig = figure;
    hold on;
    title(strcat("ARE: ", string(ekfName), extraID));
    xlabel("time (s)");
    ylabel("absolute rotation error (degrees)");

    ateFig = figure;
    hold on;
    title(strcat("ATE: ", string(ekfName), extraID));
    xlabel("time (s)");
    ylabel("absolute translation error (m)");

    neesFig = figure;
    hold on;
    title(strcat("NEES: ", string(ekfName), extraID));
    xlabel("time (s)");
    ylabel("NEES")

    nisFig = figure;
    hold on;
    title(strcat("NIS: ", string(ekfName),extraID));
    xlabel("time (s)");
    ylabel("NIS")

 %% Analysis

    for j=1:length(trajFolderList)
        ekfResult = load(trajFolderList{j});
        if size(ekfResult)==1
            ekfResult = ekfResult.ekfResult;
        end
        parts = strsplit(string(trajFolderList{j}), '\');
        trajName = string((parts{end-1}));
        trajName = erase(trajName, "sim_");
        
        %*******************************************
        %optionally, scrub of results with more than 1 second of blindndess
        idx = size(ekfResult.x_, 2);
        if doScrub
            idx = find(ekfResult.timeSinceLastCorrection > 0.4, 1, "first");
        end
        if isempty(idx)
            idx = size(ekfResult.x_, 2);
        end
        gtNotNan=~isnan(ekfResult.trueState(1,:));
        idx_gt = sum(gtNotNan(1:idx));
       
        %*******************************************
        %time to converge
        ttc_are_idx = find(ekfResult.trajErr.AbsoluteError(1:idx_gt,1)'<= 5, 1, "first");%time to converge
        if isempty(ttc_are_idx)
            ttc_are = nan(1,1);
        else
            tempIdx = find(gtNotNan==1,ttc_are_idx,'first');
            ttc_are_idx = tempIdx(end);
            ttc_are = ekfResult.time(1, ttc_are_idx);
        end
        ttc_ate_idx = find(ekfResult.trajErr.AbsoluteError(1:idx_gt,2)'<= 0.3, 1, "first");
        if isempty(ttc_ate_idx)
            ttc_ate = nan(1,1);
        else
            tempIdx = find(gtNotNan==1,ttc_ate_idx,'first');
            ttc_ate_idx = tempIdx(end);
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
        stateSize = ekfSize-1;
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
        % %max, min, mean error per state - as much as possible
        gtSize = size(ekfResult.trueState, 1);
        trajErr_i =ekfResult.trajErr.AbsoluteError(1:idx_gt,:)';

        if exist(ekfResult.P, 'var')
            [vio, x_err]=evalPercentDivergence(ekfResult.x_(1:gtSize, 1:idx), ekfResult.trueState(1:gtSize, 1:idx), ekfResult.P(1:gtSize, 1:gtSize,1:idx), 2);
        else
            [vio, x_err]=evalPercentDivergence(ekfResult.x_(1:gtSize, 1:idx), ekfResult.trueState(1:gtSize, 1:idx), ekfResult.PHat(1:gtSize, 1:gtSize,1:idx), 2);
        end
        if gtSize > 7
            velErr_i = abs(vecnorm(x_err(8:10,:), 2, 1));
            velErr_i = rmoutliers(velErr_i, 'mean');
            if size(x_err,1)>10
                baErr_i = abs(vecnorm(x_err(11:13,:), 2, 1));
                bgErr_i = abs(vecnorm(x_err(14:16,:), 2, 1));
            else
                baErr_i = nan(size(x_err, 2));
                bgErr_i= nan(size(x_err, 2));
            end
        end

        %*******************************************
        % get max stabilised error per state
        maxStableErr(1,1) = max(trajErr_i(2,250:end));
        maxStableErr(2,1) = max(trajErr_i(1,250:end));
        if gtSize>7
            maxStableErr(3,1) = max(velErr_i(250:end));
        %if size(ekfMetricsArr(i).simpleErr, 1) > 10
            maxStableErr(4,1) = max(baErr_i(250:end));
            maxStableErr(5,1) = max(bgErr_i(250:end));
        %end
        end

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
        gtTime = ekfResult.elapsedTime(gtNotNan);
        validNees = ekfResult.nees(gtNotNan);
        figure(areFig);
        plot(gtTime(1:idx_gt),  ekfResult.trajErr.AbsoluteError(1:idx_gt,1)', 'DisplayName', trajName);
        hold on;

        figure(ateFig);
        plot(gtTime(1:idx_gt),  ekfResult.trajErr.AbsoluteError(1:idx_gt,2)','DisplayName', trajName);
        hold on;
        
        figure(neesFig);
        plot(gtTime(1:idx_gt),  validNees(:, 1:idx_gt)','DisplayName', trajName);
        hold on;


        figure(nisFig);
        plot(ekfResult.elapsedTime(:, nan_idx),  ekfResult.nis(:, nan_idx)','DisplayName', trajName);
        hold on;
        
    end

    ekfMetrics.trajErr = trajErrArr;
    ekfMetrics.simpleErr = simpleErrArr;
    ekfMetrics.percentVio = percentVioArr;
    ekfMetrics.percentNees = neesPercArr;
    ekfMetrics.percentNis = nisPercArr;
    ekfMetrics.ttc = ttcArr;
    ekfMetrics.ekfType = ekfName;
    ekfMetrics.maxStableErr = maxStableErrArr;



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

end