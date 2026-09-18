% Example script demonstrating how to use correctRadialDistortion
% This shows how to correct radial distortion for a single pixel
%
% COORDINATE SYSTEM:
%   - Origin: Top-left corner of image
%   - x-axis: Increases to the right (0 = left edge)
%   - y-axis: Increases downward (0 = top edge)
%   - Units: Pixels

%% Example 1: Basic usage with k1 and k2 only
% Distorted pixel coordinates (in standard image coordinates: top-left origin)
x_distorted = 320;  % pixels from left edge
y_distorted = 240;  % pixels from top edge

% Distortion coefficients
k1 = -0.1;
k2 = 0.05;

% Correct the distortion
[x_corrected, y_corrected] = correctRadialDistortion(x_distorted, y_distorted, k1, k2);

fprintf('Example 1: Basic usage\n');
fprintf('Distorted: (%.2f, %.2f)\n', x_distorted, y_distorted);
fprintf('Corrected: (%.2f, %.2f)\n\n', x_corrected, y_corrected);

%% Example 2: With k3 coefficient
k3 = 0.01;
[x_corrected2, y_corrected2] = correctRadialDistortion(x_distorted, y_distorted, k1, k2, k3);

fprintf('Example 2: With k3\n');
fprintf('Distorted: (%.2f, %.2f)\n', x_distorted, y_distorted);
fprintf('Corrected: (%.2f, %.2f)\n\n', x_corrected2, y_corrected2);

%% Example 3: With principal point (center of distortion)
% If your principal point is not at (0,0), specify it
% Principal point is also in pixel coordinates (top-left origin)
imageWidth = 640;
imageHeight = 480;
principalPoint = [imageWidth/2, imageHeight/2];  % Center of image: [320, 240]

% Pixel coordinates in standard image coordinates (top-left origin)
x_pixel = 400;  % 400 pixels from left edge
y_pixel = 300;  % 300 pixels from top edge

[x_corrected3, y_corrected3] = correctRadialDistortion(x_pixel, y_pixel, k1, k2, k3, principalPoint);

fprintf('Example 3: With principal point\n');
fprintf('Principal point: (%.2f, %.2f)\n', principalPoint(1), principalPoint(2));
fprintf('Distorted pixel: (%.2f, %.2f)\n', x_pixel, y_pixel);
fprintf('Corrected pixel: (%.2f, %.2f)\n\n', x_corrected3, y_corrected3);

%% Example 4: Verify the correction works
% Apply forward distortion to corrected coordinates to verify
x_norm = x_corrected3 - principalPoint(1);
y_norm = y_corrected3 - principalPoint(2);
r2 = x_norm^2 + y_norm^2;
r4 = r2^2;
r6 = r2^3;
distortionFactor = 1 + k1*r2 + k2*r4 + k3*r6;
x_verify = x_norm * distortionFactor + principalPoint(1);
y_verify = y_norm * distortionFactor + principalPoint(2);

fprintf('Example 4: Verification\n');
fprintf('Original distorted: (%.2f, %.2f)\n', x_pixel, y_pixel);
fprintf('After correction and re-distortion: (%.2f, %.2f)\n', x_verify, y_verify);
fprintf('Error: (%.6f, %.6f)\n', abs(x_pixel - x_verify), abs(y_pixel - y_verify));

