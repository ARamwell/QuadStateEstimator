%Script to import video from file, convert each frame to an image, and save
%those images to the same folder
vidFile = fullfile('.', '/QuadSimEnv/Results/Traj-0027/camOutput.avi');
targetFolder = fileparts(vidFile);
imgFuncs.convertVideo(vidFile,targetFolder);
