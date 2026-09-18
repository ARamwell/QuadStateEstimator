
rawImageFolder = uigetdir(pwd, 'Select folder of raw images for calibration');
undistImageFolder = uigetdir(pwd, 'Select destination folder for undistorted images');

camParamFile = fullfile('C:/Users/Alyssa/Documents/QuadStateEstimator/Resources/Calibrations/params_imx219_640p.mat');
camParams = cameraParameters(load(camParamFile));
refTime = datetime(2000, 01, 01); %if importing simulation data
[imageStream, imageTime] = imgFuncs.importImageSeq(rawImageFolder, 0, refTime); %returns grayscale
totalFrames = size(imageTime,2);

for f = 1: totalFrames
    img=imageStream(:,:,f);
    img_u = undistortImage(img, camParams);
    
    imgname = strcat('img_u_', int2str(f));
    outputFullFileName = (strcat(undistImageFolder, '/', imgname, '.jpg'));
    imwrite(img_u, outputFullFileName, 'jpg');
end