function [in_times, in_pos, in_eul] = generateSpiralTrajectory(rpm, numLoops, dr_per_loop, dz_per_loop, startHeight, startRadius, numWaypoints)
%GENERATESPIRALTRAJECTORY Generate Archimedes spiral trajectory waypoints
%   Generates waypoints for a drone following an Archimedes spiral centered
%   at [0 0 0], with camera always pointing at the target.
%
%   Inputs:
%       rpm             - Rotations per minute (default: 1)
%       numLoops        - Number of spiral loops/rotations (default: 3)
%       dr_per_loop     - Horizontal distance increase per loop in meters (default: 0.5)
%       dz_per_loop     - Vertical distance increase per loop in meters (default: 0.3)
%       startHeight     - Starting height above target in meters (default: 0.5)
%       startRadius     - Starting radius from center in meters (default: 0.1)
%       numWaypoints    - Number of waypoints to generate (default: 20)
%
%   Outputs:
%       in_times        - Column vector of time stamps (seconds)
%       in_pos          - 3xN matrix of positions [x; y; z] in NED frame
%       in_eul          - 3xN matrix of Euler angles [roll; pitch; yaw] in degrees (XYZ convention)
%
%   Example:
%       [times, pos, eul] = generateSpiralTrajectory(1, 3, 0.5, 0.3, 0.5, 0.1, 30);

    % Default values
    if nargin < 1 || isempty(rpm), rpm = 1; end
    if nargin < 2 || isempty(numLoops), numLoops = 3; end
    if nargin < 3 || isempty(dr_per_loop), dr_per_loop = 0.5; end
    if nargin < 4 || isempty(dz_per_loop), dz_per_loop = 0.3; end
    if nargin < 5 || isempty(startHeight), startHeight = 0.5; end
    if nargin < 6 || isempty(startRadius), startRadius = 0.1; end
    if nargin < 7 || isempty(numWaypoints), numWaypoints = 20; end
    
    % Calculate total duration
    duration_sec = (numLoops / rpm) * 60; % total duration in seconds
    
    % Generate angles (0 to 2*pi*numLoops)
    theta = linspace(0, 2*pi*numLoops, numWaypoints);
    
    % Generate spiral positions
    % Archimedes spiral: r = r0 + a*theta
    % where a = dr_per_loop / (2*pi)
    a = dr_per_loop / (2*pi);
    r = startRadius + a * theta;
    
    % Horizontal positions (NED: x=North, y=East)
    x = r .* cos(theta);
    y = r .* sin(theta);
    
    % Vertical position (NED: z positive is down, so negative is up)
    % z starts at -startHeight and decreases (goes more negative = higher)
    dz_total = dz_per_loop * theta / (2*pi);
    z = -(startHeight + dz_total);
    
    % Generate times (uniformly distributed)
    in_times = linspace(0.01, duration_sec, numWaypoints)';
    
    % Calculate Euler angles to point camera at target [0 0 0]
    % In NED: x=North, y=East, z=Down (positive z is downward)
    % Using XYZ Euler convention: [roll(X), pitch(Y), yaw(Z)]
    % The drone's z-axis (pointing down in body frame) must align with direction to target
    %
    % To point z-axis at target:
    % - Yaw (Z rotation): Rotate around z-axis to orient body x-axis toward target's projection
    % - Pitch (Y rotation): Tilt down around y-axis to point z-axis at target
    % - Roll (X rotation): Keep level (0)
    
    in_eul = zeros(3, numWaypoints);
    for i = 1:numWaypoints
        % Vector from drone to target (in NED frame)
        % Target is at [0, 0, 0], drone is at [x(i), y(i), z(i)]
        % Direction vector: target - drone = [0-x, 0-y, 0-z] = [-x, -y, -z]
        dx = -x(i);
        dy = -y(i);
        dz = -z(i);
        
        % Horizontal distance in xy-plane
        horizontal_dist = sqrt(dx^2 + dy^2);
        
        % Yaw (Z rotation): angle from North (x-axis) to target's projection in horizontal plane
        % This orients the body so its x-axis points toward the target horizontally
        yaw = atan2d(dy, dx);  % atan2(y, x) gives angle from +x axis
        
        % Pitch (Y rotation): angle to tilt down to point z-axis at target
        % The angle from horizontal down to the target direction
        % If dz is positive (target is below), we tilt down (positive pitch)
        % If dz is negative (target is above - shouldn't happen but handle it), we tilt up
        pitch = atan2d(horizontal_dist, dz);  % Tilt down by this angle
        
        % Roll (X rotation): Keep level
        roll = 0;
        
        % Store in XYZ order: [roll; pitch; yaw] = [X; Y; Z]
        in_eul(:, i) = [roll; pitch; yaw];
    end
    
    % Unwrap Euler angles to prevent discontinuities at ±180° wrap-around
    % Convert to radians, unwrap, then convert back to degrees
    in_eul_rad = deg2rad(in_eul);
    in_eul_rad_unwrapped = unwrap(in_eul_rad, [], 2);  % Unwrap along dimension 2 (across waypoints)
    in_eul = rad2deg(in_eul_rad_unwrapped);
    
    % Combine positions (each column is a waypoint: [x; y; z])
    in_pos = [x; y; z];
end

