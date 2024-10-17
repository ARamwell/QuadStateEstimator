%Script to import video from file, convert each frame to an image, and save
%those images to the same folder
vidFile = fullfile("C:\Users\alyss\OneDrive - University of Cape Town\Sandbox\PnP\Pnp_solver\GitClone\SimEnv\Results\camOutput.avi");
targetFolder = fileparts(vidFile);
imgFuncs.convertVideo(vidFile,targetFolder);
