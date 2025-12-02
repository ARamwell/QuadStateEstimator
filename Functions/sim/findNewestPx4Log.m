function [fileName_newest] = findNewestPx4Log(searchFolder)
%FINDNEWESTPX4LOG Summary of this function goes here
%   Detailed explanation goes here
    %first, find newest folder in directory
    %for SITL, search directory is: \\wsl.localhost\Ubuntu-22.04\home\alyssa\PX4-Autopilot\build\px4_sitl_default\log
    px4LogFolders = dir(searchFolder);
    px4LogFolders = px4LogFolders(3:end); %remove current and parent directory
    px4Folders_dates = [];
    for i=1:size(px4LogFolders, 1)
        px4Folders_dates = [px4Folders_dates; datetime(px4LogFolders(i).name)];
    end
    [~, ind_newest] = max(px4Folders_dates);
    px4LogFolder_newest =  px4LogFolders(ind_newest);

    %then, find newest log
    px4LogFiles = dir(strcat(px4LogFolder_newest.folder, '\', px4LogFolder_newest.name));
    px4LogFiles = px4LogFiles(3:end);
    px4Files_dates = [];
    for i=1:size(px4LogFiles, 1)
        px4Files_dates = [px4Files_dates, datetime(px4LogFiles(i).date, 'Locale', 'en_UK')];
    end
    %px4Files_dates = px4LogFiles(1:end).date;
    [~, ind_newest]= max(px4Files_dates);
    px4LogFiles_newest =  px4LogFiles(ind_newest);

    fileName_newest = px4LogFiles_newest;
end

