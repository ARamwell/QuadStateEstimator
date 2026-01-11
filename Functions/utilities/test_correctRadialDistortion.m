% Test script for correctRadialDistortion
% This helps diagnose why the function might return the same values

clear; clc;

%% Test Case 1: Simple case with known distortion
fprintf('=== Test Case 1: Simple distortion correction ===\n');
x_dist = 100;  % pixel from left
y_dist = 100;  % pixel from top
k1 = -0.1;     % barrel distortion (negative k1)
k2 = 0.05;
principalPoint = [320, 240];  % center of 640x480 image

[x_corr, y_corr] = correctRadialDistortion(x_dist, y_dist, k1, k2, 0, principalPoint);

fprintf('Input (distorted): (%.2f, %.2f)\n', x_dist, y_dist);
fprintf('Output (corrected): (%.2f, %.2f)\n', x_corr, y_corr);
fprintf('Difference: (%.4f, %.4f)\n\n', x_corr - x_dist, y_corr - y_dist);

% Verify: apply forward distortion to corrected coordinates
x_rel_corr = x_corr - principalPoint(1);
y_rel_corr = y_corr - principalPoint(2);
r2_corr = x_rel_corr^2 + y_rel_corr^2;
r4_corr = r2_corr^2;
distortionFactor = 1 + k1*r2_corr + k2*r4_corr;
x_verify = x_rel_corr * distortionFactor + principalPoint(1);
y_verify = y_rel_corr * distortionFactor + principalPoint(2);
fprintf('Verification: Applying distortion to corrected coords gives: (%.2f, %.2f)\n', x_verify, y_verify);
fprintf('Original distorted: (%.2f, %.2f)\n', x_dist, y_dist);
fprintf('Verification error: (%.6f, %.6f)\n\n', abs(x_verify - x_dist), abs(y_verify - y_dist));

%% Test Case 2: With normalized coordinates (typical case)
fprintf('=== Test Case 2: With normalized coordinates ===\n');
fprintf('(This is the typical case - distortion coefficients in normalized coords)\n');
focalLength = 350;  % typical focal length in pixels
k1_norm = -0.1;     % normalized distortion coefficient
k2_norm = 0.05;

[x_corr2, y_corr2] = correctRadialDistortion(x_dist, y_dist, k1_norm, k2_norm, 0, principalPoint, focalLength);

fprintf('Input (distorted): (%.2f, %.2f)\n', x_dist, y_dist);
fprintf('Output (corrected): (%.2f, %.2f)\n', x_corr2, y_corr2);
fprintf('Difference: (%.4f, %.4f)\n\n', x_corr2 - x_dist, y_corr2 - y_dist);

%% Test Case 3: Edge case - pixel at principal point
fprintf('=== Test Case 3: Pixel at principal point ===\n');
x_dist3 = principalPoint(1);
y_dist3 = principalPoint(2);

[x_corr3, y_corr3] = correctRadialDistortion(x_dist3, y_dist3, k1, k2, 0, principalPoint);

fprintf('Input (distorted): (%.2f, %.2f)\n', x_dist3, y_dist3);
fprintf('Output (corrected): (%.2f, %.2f)\n', x_corr3, y_corr3);
fprintf('(Should be the same - no distortion at center)\n\n');

%% Test Case 4: Large distortion
fprintf('=== Test Case 4: Large distortion (edge of image) ===\n');
x_dist4 = 600;  % near right edge
y_dist4 = 400;  % near bottom edge

[x_corr4, y_corr4] = correctRadialDistortion(x_dist4, y_dist4, k1, k2, 0, principalPoint);

fprintf('Input (distorted): (%.2f, %.2f)\n', x_dist4, y_dist4);
fprintf('Output (corrected): (%.2f, %.2f)\n', x_corr4, y_corr4);
fprintf('Difference: (%.4f, %.4f)\n\n', x_corr4 - x_dist4, y_corr4 - y_dist4);

%% Instructions for user
fprintf('=== DIAGNOSTIC TIPS ===\n');
fprintf('If you see the same values returned:\n');
fprintf('1. Check if your distortion coefficients are very small (< 0.001)\n');
fprintf('2. Check if your pixel is at or very close to the principal point\n');
fprintf('3. If your coefficients are in normalized coordinates, provide focalLength parameter\n');
fprintf('4. Try Test Case 4 above - edge pixels should show larger correction\n');




