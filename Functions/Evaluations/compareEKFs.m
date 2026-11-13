%% Choose folders
listOfFolderNames =selector_multiFolder(pwd, 'Select folders to search for trajectories');
numFolders = length(listOfFolderNames);


%% Initialise preliminaries

g=[0 0 -9.81]'; %for simulation
%g = [0 0 -9.79]'; %for physical

% Which EKFs ran
integ_arr = {'rect'};%{'rect' 'rect'};
ekfSize_arr =[16];%[16 16  ];
alpha_arr = [0];%[0 0.99];
numEkfs = size(ekfSize_arr, 2);
extraID = "simMap";

%ekfMetricsArr = createArray(numEkfs,0);
clear ekfMetricsArr ekfTempMetricsArr
ekfSearchTerms = {};
for i = 1:numEkfs
    ekfSearchTerms{i} = strcat(string(ekfSize_arr(i)), "el", "_", string(integ_arr(i)), "_a", string(alpha_arr(i)), "_");
end



%% Search for patterns in all folders

%% Extract EKFs
listOfFileNames = {};
for f=1:numFolders
    %% Get current folder
    currentFolder = string(listOfFolderNames(f));
    
    %% There is more than one EKF in each folder
    searchTerm = strcat("ekfResult_", "*", extraID,  ".mat");
    %searchTerm =strcat("ekfResult*", extraID);
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
    %idxStatic=find(contains(listOfFileNames, 'tatic'));
    %logicalIndices(idxStatic)=0;
    indices = find(logicalIndices, length(listOfFileNames));
    selectListOfFileNames = listOfFileNames(indices);

    [ekfMetrics, ateFig, areFig, neesFig, nisFig]=analyseEkf(ekfSize_arr(i),ekfSearchTerms{i}, selectListOfFileNames, extraID,true);

    %%
    
    % 
    saveDest ="C:\Users\Alyssa\OneDrive - University of Cape Town\Thesis\TestsAndResults\PostDiss\errorState\";
    formatFigForLatex(areFig);
    formatFigForLatex(ateFig);
    formatFigForLatex(neesFig);
    formatFigForLatex(nisFig);
    name_areFig = strcat(saveDest, "are_combined_", ekfSearchTerms{i}, extraID, ".fig");
    name_ateFig = strcat(saveDest, "ate_combined_", ekfSearchTerms{i}, extraID, ".fig");
    savefig(areFig, name_areFig);
    savefig(ateFig, name_ateFig);

    name_neesFig = strcat(saveDest, "nees_combined_", ekfSearchTerms{i}, extraID, ".fig");
    name_nisFig = strcat(saveDest, "nis_combined_", ekfSearchTerms{i}, extraID, ".fig");
    savefig(neesFig, name_neesFig);
    savefig(nisFig, name_nisFig);

    ekfMetricsArr(i) = ekfMetrics;


end

mean(ekfMetricsArr(1).trajErr(1,:));

mean(ekfMetricsArr(1).trajErr(2,:));
%%
%neesnisViolin(ekfTempMetricsArr);


%% Plot some violin plots for nees and nis
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

    dynArgs_nees{end+1} = 0.5;   % bin size
    dynArgs_nees{end+1} = 'NEES';
    dynArgs_nis{end+1} = 0.5;   % bin size
    dynArgs_nis{end+1} = 'NIS';

    plotViolin(dynArgs_nees{:})
    plotViolin(dynArgs_nis{:})

end


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