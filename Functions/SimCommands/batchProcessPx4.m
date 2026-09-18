
%% Choose folders
listOfFolderNames =selector_multiFolder(pwd, 'Select sim folders to run EKF on');
numFolders = length(listOfFolderNames);


%% Search for patterns in all folders

%% Extract EKFs
for f=1:numFolders
    %% Get current folder
    currentFolder = string(listOfFolderNames(f));
    parts = strsplit(currentFolder, '\');
    trajName = string((parts{end}));

    %% There is only one EKF in each folder
    searchTerm = strcat('*.ulg');
    fileStruct = dir(fullfile(currentFolder, searchTerm));
    fileTable = struct2table(fileStruct);
    newUlgFileName = fileTable.name;

    % Get full path
    newUlgFileName = fullfile(currentFolder, newUlgFileName);
    newSimoutFileName = fullfile(currentFolder, 'simout.mat');

    px4LogFile =string(newUlgFileName);
    simout = load(string(newSimoutFileName));

    if length(fieldnames(simout)) == 1
        simout = simout.simout;
    end
   
    [groundTruth, imuData, ekfResult, p3pResult] = processSimData(simout, estset.runEKF, estset.runP3P); %make simout more usable and readable
    px4Result = processPx4Data(px4LogFile, simset.aidingActive, groundTruth); %import ulogs into readable and useful format\

    %% DO COMPARISONS, MAKE GRAPHS
    % p3pName = strcat('p3pResult_', trajNames(i), '.mat');
    % p3pFile = strcat(p3pFolder, '\', p3pName);
    % save(p3pFile, 'p3pResult.mat', '-struct');
    px4File = strcat(currentFolder, '\px4Result_', trajName, '.mat');
    save(px4File, '-struct', 'px4Result');
end
