
[listOfFolderNames] = selector_multiFolder();
numFolders = length(listOfFolderNames);

[saveFolder] = uigetdir(pwd, 'Choose Save Folder');
mapFile = './Resources/map.mat';
map = load(mapFile);
T_rc2rq = map.worldObjectStruct.transforms.T_gencam2genquad;
T_rq2rc = p3pFuncs.invertT(T_rc2rq);

camParamFile_d = fullfile('C:/Users/Alyssa/Documents/QuadStateEstimator/Resources/Calibrations/params_imx219_640p_lowdist.mat');
camParams_d = cameraParameters(load(camParamFile_d));
K_d = camParams_d.K;

camParamFile_u = fullfile('C:/Users/Alyssa/Documents/QuadStateEstimator/Resources/Calibrations/params_imx219_640p_lowdist.mat');
camParams_u = cameraParameters(load(camParamFile_u));
K_u = camParams_u.K;
%%
for f=1:numFolders
    %figure;
    currentFolder = string(listOfFolderNames(f));
    parts = strsplit(currentFolder, '\');
    trajName = string((parts{end}));

    p3pResult = runP3pOnFile(currentFolder, camParams_d, K_u, T_rq2rc);

    saveName = strcat(saveFolder, '\p3pResult_', trajName, '.mat');
    save(saveName, "p3pResult", "-mat");
end





