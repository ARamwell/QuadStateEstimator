function [x_corrected, y_corrected] = correctRadialDistortion(x_distorted, y_distorted, k1, k2, k3, principalPoint, focalLength)
%CORRECTRADIALDISTORTION Corrects radial distortion for a single pixel
%   This function takes a distorted pixel coordinate and applies the
%   inverse of the radial distortion model to return the corrected
%   (undistorted) pixel coordinate.
%
%   Inputs:
%       x_distorted, y_distorted: Distorted pixel coordinates
%                                 - Coordinate system: Standard image coordinates
%                                 - Origin: Top-left corner of image
%                                 - x-axis: Increases to the right
%                                 - y-axis: Increases downward
%                                 - Units: Pixels (e.g., x=320, y=240 for center of 640x480 image)
%       k1, k2: Radial distortion coefficients (required)
%               - These should be in the same coordinate system as your calibration
%               - Typically in normalized coordinates (relative to focal length)
%               - If your calibration used pixel coordinates, use those coefficients
%       k3: Radial distortion coefficient (optional, defaults to 0)
%       principalPoint: [cx, cy] - principal point/center of distortion in pixel coordinates
%                      - Same coordinate system as x_distorted, y_distorted (top-left origin)
%                      - Typically the image center: [imageWidth/2, imageHeight/2]
%                      - Optional, defaults to [0, 0] if not provided
%       focalLength: [fx, fy] - focal length in pixels (optional)
%                    - If provided, coordinates will be converted to normalized coordinates
%                    - If not provided, assumes coefficients are already in pixel coordinates
%
%   Outputs:
%       x_corrected, y_corrected: Corrected (undistorted) pixel coordinates
%                                 - Same coordinate system as inputs (top-left origin)
%
%   Distortion model:
%       x_distorted = x_undistorted * (1 + k1*r^2 + k2*r^4 + k3*r^6)
%       y_distorted = y_undistorted * (1 + k1*r^2 + k2*r^4 + k3*r^6)
%       where r^2 = (x_undistorted - cx)^2 + (y_undistorted - cy)^2
%       (applied in coordinates relative to principal point)
%
%   Example:
%       % For a 640x480 image with principal point at center
%       x_pixel = 400;  % pixel from left edge
%       y_pixel = 300;  % pixel from top edge
%       k1 = -0.1;
%       k2 = 0.05;
%       principalPoint = [320, 240];  % center of 640x480 image
%       [x_corr, y_corr] = correctRadialDistortion(x_pixel, y_pixel, k1, k2, [], principalPoint);

    % Set defaults
    if nargin < 5 || isempty(k3)
        k3 = 0;
    end
    
    if nargin < 6 || isempty(principalPoint)
        principalPoint = [0, 0];
    end
    
    useNormalizedCoords = false;
    if nargin >= 7 && ~isempty(focalLength)
        useNormalizedCoords = true;
        if length(focalLength) == 1
            fx = focalLength;
            fy = focalLength;
        else
            fx = focalLength(1);
            fy = focalLength(2);
        end
    end
    
    % Convert to coordinates relative to principal point
    x_rel = x_distorted - principalPoint(1);
    y_rel = y_distorted - principalPoint(2);
    
    % Convert to normalized coordinates if focal length provided
    if useNormalizedCoords
        x_norm = x_rel / fx;
        y_norm = y_rel / fy;
    else
        % Work directly in pixel coordinates
        x_norm = x_rel;
        y_norm = y_rel;
    end
    
    % Better initial guess: for barrel distortion (negative k1), 
    % undistorted point is closer to center than distorted point
    % Use a simple approximation for initial guess
    r2_initial = x_norm^2 + y_norm^2;
    r4_initial = r2_initial^2;
    initialDistortionFactor = 1 + k1*r2_initial + k2*r4_initial + k3*r2_initial^3;
    
    % Initial guess: if distortion factor > 1, point was pushed outward,
    % so undistorted is closer to center
    if abs(initialDistortionFactor) > 1e-10
        x_undist = x_norm / initialDistortionFactor;
        y_undist = y_norm / initialDistortionFactor;
    else
        x_undist = x_norm;
        y_undist = y_norm;
    end
    
    % Iterative correction using fixed-point iteration with under-relaxation
    % We want to solve: x_dist = x_undist * (1 + k1*r^2 + k2*r^4 + k3*r^6)
    % Rearranging: x_undist = x_dist / (1 + k1*r^2 + k2*r^4 + k3*r^6)
    % Since r^2 depends on x_undist, we iterate
    
    maxIterations = 100;
    tolerance = 1e-8;
    relaxationFactor = 0.8;  % Under-relaxation for stability
    
    for iter = 1:maxIterations
        % Store previous estimate
        x_undist_prev = x_undist;
        y_undist_prev = y_undist;
        
        % Calculate current radius squared (from current undistorted estimate)
        r2 = x_undist^2 + y_undist^2;
        r4 = r2^2;
        r6 = r2^3;
        
        % Calculate distortion factor
        distortionFactor = 1 + k1*r2 + k2*r4 + k3*r6;
        
        % Check for convergence by applying forward distortion
        x_pred = x_undist * distortionFactor;
        y_pred = y_undist * distortionFactor;
        
        % Calculate error
        error_x = x_norm - x_pred;
        error_y = y_norm - y_pred;
        totalError = sqrt(error_x^2 + error_y^2);
        
        % Check convergence
        if totalError < tolerance
            break;
        end
        
        % Update estimate using fixed-point iteration with under-relaxation
        % x_undist_new = x_dist / (1 + k1*r^2 + k2*r^4 + k3*r^6)
        % where r^2 is calculated from x_undist_old
        if abs(distortionFactor) > 1e-12  % Avoid division by zero
            x_undist_new = x_norm / distortionFactor;
            y_undist_new = y_norm / distortionFactor;
            
            % Use under-relaxation for stability: x_new = (1-alpha)*x_old + alpha*x_calc
            x_undist = (1 - relaxationFactor) * x_undist_prev + relaxationFactor * x_undist_new;
            y_undist = (1 - relaxationFactor) * y_undist_prev + relaxationFactor * y_undist_new;
        else
            % Fallback: if distortion factor is too small, use gradient descent
            stepSize = 0.1;
            x_undist = x_undist_prev + stepSize * error_x;
            y_undist = y_undist_prev + stepSize * error_y;
        end
        
        % Check for divergence (shouldn't happen, but safety check)
        if any(isnan([x_undist, y_undist])) || any(isinf([x_undist, y_undist]))
            warning('Iteration diverged, returning original coordinates');
            x_undist = x_norm;
            y_undist = y_norm;
            break;
        end
    end
    
    % Convert back to pixel coordinates
    if useNormalizedCoords
        x_corrected = x_undist * fx + principalPoint(1);
        y_corrected = y_undist * fy + principalPoint(2);
    else
        x_corrected = x_undist + principalPoint(1);
        y_corrected = y_undist + principalPoint(2);
    end
    
end

