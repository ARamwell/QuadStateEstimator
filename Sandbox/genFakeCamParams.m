% MATLAB script to create a cameraParameters object for an ideal 640p camera
% This creates camera intrinsic parameters for a camera with 640 pixel width

% Camera image size (640p typically refers to 640x480 or 640x360)
imageSize = [360, 640]; % [height, width] in pixels

% Focal length in pixels
% For an ideal camera, we'll use a typical focal length
% Common values range from 400-800 pixels for this resolution
focalLength = [380, 380]; % [fx, fy] in pixels

% Principal point (optical center) - for ideal camera, at image center
principalPoint = [imageSize(2)/2, imageSize(1)/2]; % [cx, cy] in pixels

% Skew coefficient (typically 0 for most cameras)
skew = 0;

% Distortion coefficients (for ideal camera, assume no distortion)
% [k1, k2, p1, p2, k3] - radial and tangential distortion
radialDistortion = [0, 0, 0]; % [k1, k2, k3]
tangentialDistortion = [0, 0]; % [p1, p2]

% Create camera intrinsics object
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize, ...
    'RadialDistortion', radialDistortion, ...
    'TangentialDistortion', tangentialDistortion, ...
    'Skew', skew);

% % Display the camera parameters
% fprintf('Camera Parameters for Ideal 640p Camera:\n');
% fprintf('==========================================\n');
% fprintf('Image Size: %dx%d pixels\n', imageSize(2), imageSize(1));
% fprintf('Focal Length: [%.2f, %.2f] pixels\n', focalLength(1), focalLength(2));
% fprintf('Principal Point: [%.2f, %.2f] pixels\n', principalPoint(1), principalPoint(2));
% fprintf('Radial Distortion: [%.4f, %.4f, %.4f]\n', radialDistortion(1), ...
%     radialDistortion(2), radialDistortion(3));
% fprintf('Tangential Distortion: [%.4f, %.4f]\n', tangentialDistortion(1), ...
%     tangentialDistortion(2));
% fprintf('\n');

% The cameraIntrinsics object is ready to use
% You can also convert it to cameraParameters if needed for older MATLAB versions
cameraParams = cameraParameters('IntrinsicMatrix', camIntrinsics.IntrinsicMatrix);
camPin = toStruct(cameraParams);

[saveFile, saveFolder] = uiputfile({'*.mat','MAT-files (*.mat)'; ...
                                     '*.txt','Text files (*.txt)'; ...
                                     '*.*','All Files (*.*)'}, ...
                                     'Save camera params as');
fullFileName = fullfile(saveFolder, saveFile);

save(fullFileName, '-struct', "camPin"); 

