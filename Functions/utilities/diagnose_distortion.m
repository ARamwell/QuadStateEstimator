% Diagnostic script to help identify distortion correction issues
% Run this with your actual values to see what's happening

clear; clc;

% YOUR VALUES - EDIT THESE
x_distorted = 400;  % Your distorted pixel x coordinate
y_distorted = 300;  % Your distorted pixel y coordinate
k1 = -0.34;         % Your k1 value
k2 = 0.1;           % Your k2 value
principalPoint = [320, 240];  % Your principal point [cx, cy]
focalLength = [];   % Your focal length (leave empty if unknown)

fprintf('=== DIAGNOSTIC INFORMATION ===\n\n');
fprintf('Input values:\n');
fprintf('  Distorted pixel: (%.2f, %.2f)\n', x_distorted, y_distorted);
fprintf('  k1 = %.4f, k2 = %.4f\n', k1, k2);
fprintf('  Principal point: (%.2f, %.2f)\n', principalPoint(1), principalPoint(2));
fprintf('  Focal length: %s\n\n', mat2str(focalLength));

% Calculate relative coordinates
x_rel = x_distorted - principalPoint(1);
y_rel = y_distorted - principalPoint(2);
r_pixel = sqrt(x_rel^2 + y_rel^2);

fprintf('Coordinates relative to principal point:\n');
fprintf('  (%.2f, %.2f), distance = %.2f pixels\n\n', x_rel, y_rel, r_pixel);

% Test 1: Apply distortion coefficients directly to pixel coordinates
fprintf('=== TEST 1: Coefficients applied to PIXEL coordinates ===\n');
r2_pixel = r_pixel^2;
r4_pixel = r2_pixel^2;
distortionFactor_pixel = 1 + k1*r2_pixel + k2*r4_pixel;
fprintf('  r^2 (pixels) = %.2f\n', r2_pixel);
fprintf('  Distortion factor = %.6f\n', distortionFactor_pixel);
if abs(distortionFactor_pixel - 1) < 0.001
    fprintf('  WARNING: Distortion factor is very close to 1!\n');
    fprintf('  This means almost no correction will be applied.\n');
    fprintf('  Your coefficients are likely in NORMALIZED coordinates.\n');
end
fprintf('\n');

% Test 2: Apply to normalized coordinates (typical case)
if isempty(focalLength)
    % Try a typical focal length
    testFocalLengths = [300, 350, 400, 450, 500];
    fprintf('=== TEST 2: Coefficients applied to NORMALIZED coordinates ===\n');
    fprintf('(Testing with typical focal lengths since you didn''t provide one)\n\n');
    
    for fx = testFocalLengths
        x_norm = x_rel / fx;
        y_norm = y_rel / fx;
        r_norm = sqrt(x_norm^2 + y_norm^2);
        r2_norm = r_norm^2;
        r4_norm = r2_norm^2;
        distortionFactor_norm = 1 + k1*r2_norm + k2*r4_norm;
        
        fprintf('  Focal length = %.0f pixels:\n', fx);
        fprintf('    Normalized coords: (%.4f, %.4f), r = %.4f\n', x_norm, y_norm, r_norm);
        fprintf('    r^2 (normalized) = %.6f\n', r2_norm);
        fprintf('    Distortion factor = %.6f\n', distortionFactor_norm);
        if abs(distortionFactor_norm - 1) > 0.01
            fprintf('    -> This looks reasonable! Try using focalLength = %.0f\n', fx);
        end
        fprintf('\n');
    end
else
    fprintf('=== TEST 2: Coefficients applied to NORMALIZED coordinates ===\n');
    if length(focalLength) == 1
        fx = focalLength;
        fy = focalLength;
    else
        fx = focalLength(1);
        fy = focalLength(2);
    end
    x_norm = x_rel / fx;
    y_norm = y_rel / fy;
    r_norm = sqrt(x_norm^2 + y_norm^2);
    r2_norm = r_norm^2;
    r4_norm = r2_norm^2;
    distortionFactor_norm = 1 + k1*r2_norm + k2*r4_norm;
    
    fprintf('  Normalized coords: (%.4f, %.4f), r = %.4f\n', x_norm, y_norm, r_norm);
    fprintf('  r^2 (normalized) = %.6f\n', r2_norm);
    fprintf('  Distortion factor = %.6f\n', distortionFactor_norm);
    fprintf('\n');
end

% Test 3: Try the correction function
fprintf('=== TEST 3: Running correction function ===\n');
if isempty(focalLength)
    fprintf('Without focal length:\n');
    [x_corr1, y_corr1] = correctRadialDistortion(x_distorted, y_distorted, k1, k2, 0, principalPoint);
    fprintf('  Corrected: (%.4f, %.4f)\n', x_corr1, y_corr1);
    fprintf('  Change: (%.4f, %.4f)\n', x_corr1 - x_distorted, y_corr1 - y_distorted);
    
    fprintf('\nWith focal length = 350 (example):\n');
    [x_corr2, y_corr2] = correctRadialDistortion(x_distorted, y_distorted, k1, k2, 0, principalPoint, 350);
    fprintf('  Corrected: (%.4f, %.4f)\n', x_corr2, y_corr2);
    fprintf('  Change: (%.4f, %.4f)\n', x_corr2 - x_distorted, y_corr2 - y_distorted);
else
    [x_corr, y_corr] = correctRadialDistortion(x_distorted, y_distorted, k1, k2, 0, principalPoint, focalLength);
    fprintf('  Corrected: (%.4f, %.4f)\n', x_corr, y_corr);
    fprintf('  Change: (%.4f, %.4f)\n', x_corr - x_distorted, y_corr - y_distorted);
end

fprintf('\n=== RECOMMENDATION ===\n');
fprintf('With k1 = %.2f and k2 = %.2f, these are LARGE coefficients.\n', k1, k2);
fprintf('They are almost certainly in NORMALIZED coordinates.\n');
fprintf('You MUST provide the focalLength parameter for the function to work correctly.\n');
fprintf('Find your focal length from your camera calibration (usually in the K matrix).\n');



