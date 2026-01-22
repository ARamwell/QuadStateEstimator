%function ekfMetrics = evalPx4EKF()

%ekfMetricsArr = createArray(numEkfs,0);
clear ekfMetricsArr
ekfSearchTerms = {};

%% Choose folders
listOfFolderNames =selector_multiFolder(pwd, 'Select folders with EKF results to combine');
numFolders = length(listOfFolderNames);


%% Search for patterns in all folders

%% Extract EKFs
listOfFileNames = {};
for f=1:numFolders
    %% Get current folder
    currentFolder = string(listOfFolderNames(f));
    %parts = strsplit(currentFolder, '\');
    %trajName = string((parts{end}));

    %% There is only one EKF in each folder
    searchTerm = strcat('px4Result*.mat');
    fileStruct = dir(fullfile(currentFolder, searchTerm));
    fileTable = struct2table(fileStruct);
    newFileName = fileTable.name;

    % Get full path
    newFileName = fullfile(currentFolder, newFileName);

    listOfFileNames = [listOfFileNames; newFileName];
end

%% Now, group and evaluate EKFs

%% do some analysis for this type of EKF
trajErrArr = createArray(2,0);
simpleErrArr = createArray(16,0);
percentVioArr = createArray(16,0);
neesArr =  createArray(15,0);
nisArr = createArray(4,0);
ttcArr = createArray(2,0);


figObj1 = figure;
figObj2 = figure;

for j=1:length(listOfFileNames)
    ekfResult = load(listOfFileNames{j});
    parts = strsplit(string(listOfFileNames{j}), '\');
    trajName = string((parts{end-1}));
    trajName = erase(trajName, "sim_");

    %optionally, scrub of results with more than 1 second of blindndess
    idx = find(ekfResult.timeSinceLastCorrection > 0.4, 1, "first");
    if isempty(idx)
        idx = size(ekfResult.x_, 2);
    end
    ekfSize = 16;
    measSize = 4;
    %idx = size(ekfResult.x_, 2);

    ekfResult.trajErr = evaluateTrackingPerformance(ekfResult.x_, ekfResult.trueState, 'none');
    [vio, x_err]=evalPercentDivergence(ekfResult.x_(:, 1:idx), ekfResult.trueState(:, 1:idx), ekfResult.P(:, :,1:idx), 2);

    %time to converge
    ttc_are_idx = find(ekfResult.trajErr.AbsoluteError(:,1)'<= 5, 1, "first");%time to converge
    ttc_are = ekfResult.time(1, ttc_are_idx);
    ttc_ate_idx = find(ekfResult.trajErr.AbsoluteError(:,2)'<= 0.3, 1, "first");
    ttc_ate = ekfResult.time(1, ttc_ate_idx);
    ttc=[ttc_are; ttc_ate];

    % %remove  first 20 estimates
    % ekfMetrics = ekfMetricsArr(i);
    % [trajErrClean, rmIdx] = rmoutliers(ekfMetrics.trajErr(:,20:end)', "mean");
    % ekfMetrics.trajErr_clean = trajErrClean';
    % ekfMetrics.simpleErr_clean


    %nees percent exceed
    % alpha = 0.05; %confidence
    % numMonteCarloRuns = length(listOfFileNames);
    % stateSize = ekfSize-1;
    % chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*stateSize)]/numMonteCarloRuns;
    % neesVioHi_idx = find(ekfResult.nees > chiSquareLimits(2), size(ekfResult.nees,2));
    % neesVioLo_idx =find(ekfResult.nees < chiSquareLimits(1), size(ekfResult.nees,2));
    % neesVioEith_idx = unique([neesVioHi_idx, neesVioLo_idx]);
    % neesVioPercent = [size(neesVioLo_idx, 2); size(neesVioHi_idx, 2);size(neesVioEith_idx, 2)]/idx;
    % 
    % %nis percent exceed
    % alpha = 0.05; %confidence
    % numMonteCarloRuns = length(listOfFileNames);
    % chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*measSize)]/numMonteCarloRuns;
    % nisVioHi_idx = find(ekfResult.nis > chiSquareLimits(2), size(ekfResult.nis,2));
    % nisVioLo_idx =find(ekfResult.nis < chiSquareLimits(1), size(ekfResult.nis,2));
    % nisVioEith_idx = unique([nisVioHi_idx, nisVioLo_idx]);
    % nisVioPercent = [size(nisVioLo_idx, 2); size(nisVioHi_idx, 2); size(nisVioEith_idx, 2)]/idx;


    trajErrArr = [trajErrArr, ekfResult.trajErr.AbsoluteError(1:idx,:)'];
    simpleErrArr = [simpleErrArr, x_err];
    percentVioArr = [percentVioArr, vio];
    ttcArr = [ttcArr, ttc];
    % nisArr = [nisArr, nisVioPercent];
    % neesArr = [neesArr, neesVioPercent];


    figure(figObj1);
    plot(ekfResult.time(:, 1:idx),  ekfResult.trajErr.AbsoluteError(1:idx,1)', 'DisplayName', trajName);
    hold on;

    figure(figObj2);
    plot(ekfResult.time(:, 1:idx),  ekfResult.trajErr.AbsoluteError(1:idx,2)', 'DisplayName', trajName);
    hold on;

end

ekfMetrics.trajErr = trajErrArr;
ekfMetrics.simpleErr = simpleErrArr;
ekfMetrics.percentVio = percentVioArr;
ekfMetrics.ekfType = "px4_bb";

%end

