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
listOfFolderNames =selector_multiFolder(pwd, 'Select folders to search for trajectories');
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

    [ekfMetrics, ateFig, areFig, neesFig, nisFig]=analyseEkf(ekfSize_arr(i),ekfSearchTerms{i}, selectListOfFileNames, extraID);

    %%
    
    % 
    saveDest ="C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\Diss1\p3p_test_sim\ekfComparison2\badBias\";
    formatFigForLatex(areFig);
    formatFigForLatex(ateFig);
    formatFigForLatex(neesFig);
    formatFigForLatex(nisFig);
    name_areFig = strcat(saveDest, "are_combined_", ekfSearchTerms{i}, extraID, ".fig");
    name_ateFig = strcat(saveDest, "ate_combined_", ekfSearchTerms{i}, extraID, ".fig");
     savefig(areFig, name_areFig);
     savefig(ateFig, name_ateFig);

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