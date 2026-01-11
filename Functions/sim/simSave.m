function [saveFolder, px4Name] = simSave(parentFolder, saveName, simout, simset)
%we should already have a main folder for everything to go into: saveFolder
%However, if we are running multiple simulations, we will want subfolders.
%Let's do subfolders anyway, so we have "traj" identifiers for all cases
px4Name = '';

if simset.save == false
    saveFolder = '';
else
    %make folder for sim output
    subName = strcat("sim_", saveName);
    saveFolder = strcat(parentFolder, '\', subName);
    if ~exist(saveFolder, 'dir')
        mkdir(parentFolder, subName);
    end


    %okay, now we have the subfolder, we can save all our simulation results and settings
    %in there
    save(fullfile(saveFolder, '/simset.mat'), '-struct', "simset"); %save settings
    save(fullfile(saveFolder, '/simout.mat'), "simout"); %save results
    vidFile = fullfile('.', '/camOutput.avi'); %find video
    imgFuncs.convertVideo(vidFile, saveFolder);%save images
    
    %if we did SITL or live EKF, this is a bit harder
    if simset.SITL == true
        %find newest log entry
        px4LogFiles_newest = findNewestPx4Log('\\wsl.localhost\Ubuntu-22.04\home\alyssa\PX4-Autopilot\build\px4_sitl_default\log');
           
        %copy log to sim output folder
        px4Name = strcat(px4LogFiles_newest.folder, '\', px4LogFiles_newest.name);
        copyfile(px4Name, saveFolder);
    end

end

