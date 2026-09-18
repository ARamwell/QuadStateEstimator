function [in_times, in_pos, in_eul] = generateSpiralTrajectory(rpm, numLoops, dr_per_loop, dz_per_loop, startHeight, startRadius, numWaypoints, numRollRotations)
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
%       numRollRotations - Number of full 360° roll rotations across trajectory (default: 0)
%
%   Outputs:
%       in_times        - Column vector of time stamps (seconds)
%       in_pos          - 3xN matrix of positions [x; y; z] in NED frame
%       in_eul          - 3xN matrix of Euler angles [roll; pitch; yaw] in degrees (XYZ convention)
%
%   Example:
%       [times, pos, eul] = generateSpiralTrajectory(1, 3, 0.5, 0.3, 0.5, 0.1, 30, 2);

    % Default values
    if nargin < 1 || isempty(rpm), rpm = 1; end
    if nargin < 2 || isempty(numLoops), numLoops = 3; end
    if nargin < 3 || isempty(dr_per_loop), dr_per_loop = 0.5; end
    if nargin < 4 || isempty(dz_per_loop), dz_per_loop = 0.3; end
    if nargin < 5 || isempty(startHeight), startHeight = 0.5; end
    if nargin < 6 || isempty(startRadius), startRadius = 0.1; end
    if nargin < 7 || isempty(numWaypoints), numWaypoints = 20; end
    if nargin < 8 || isempty(numRollRotations), numRollRotations = 0; end
    
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
    
    % Calculate roll angles (ramping linearly from 0 to 360*numRollRotations)
    roll_angles = linspace(0, 360 * numRollRotations, numWaypoints);
    
    % Calculate Euler angles to point camera at target [0 0 0]
    % In NED: x=North, y=East, z=Down (positive z is downward)
    % Using XYZ Euler convention: [roll(X), pitch(Y), yaw(Z)]
    % The drone's z-axis (pointing down in body frame) must align with direction to target
    %
    % For XYZ Euler: R = R_z(yaw) * R_y(pitch) * R_x(roll)
    % With roll=0: R = R_z(yaw) * R_y(pitch)
    %
    % To point z-axis at target, we compute what the final z-axis direction should be,
    % then extract the Euler angles that produce this orientation.
    
    in_eul = zeros(3, numWaypoints);
    for i = 1:numWaypoints
        % Vector from drone to target (in NED frame, normalized)
        % Target is at [0, 0, 0], drone is at [x(i), y(i), z(i)]
        vec_to_target = [0; 0; 0] - [x(i); y(i); z(i)];  % [-x, -y, -z]
        vec_to_target = vec_to_target / norm(vec_to_target);
        
        % Get the desired roll angle for this waypoint
        roll = roll_angles(i);
        
        % We want the body's z-axis [0; 0; 1] to align with vec_to_target after rotation
        % For XYZ Euler: R = R_z(yaw) * R_y(pitch) * R_x(roll)
        % We'll build the rotation matrix that first applies roll, then aligns z-axis with target
        %
        % Convert roll to radians
        roll_rad = deg2rad(roll);
        cos_roll = cos(roll_rad);
        sin_roll = sin(roll_rad);
        
        % Define a reference "up" direction (pointing up = negative z in NED)
        up_ref = [0; 0; -1];
        
        % The desired z-axis direction (normalized)
        z_axis = vec_to_target;
        
        % Build x-axis: perpendicular to both up_ref and z_axis
        x_axis = cross(up_ref, z_axis);
        if norm(x_axis) < 1e-6
            % Degenerate case: z_axis is parallel to up_ref
            % Use North (x) as reference instead
            x_axis = cross([1; 0; 0], z_axis);
            if norm(x_axis) < 1e-6
                % Still degenerate, use East (y)
                x_axis = cross([0; 1; 0], z_axis);
            end
        end
        x_axis = x_axis / norm(x_axis);
        
        % Build y-axis to complete right-handed coordinate system
        y_axis = cross(z_axis, x_axis);
        y_axis = y_axis / norm(y_axis);
        
        % Build rotation matrix with desired z-axis pointing at target
        % Then rotate x and y axes around z_axis by the roll angle
        % This ensures z-axis points at target and roll matches desired value
        
        % Start with x and y axes from the no-roll case
        x_base = x_axis;
        y_base = y_axis;
        
        % Rotate x_base and y_base vectors around z_axis by roll angle
        % Using Rodrigues' rotation formula: rotate v around axis k by angle theta
        % v_rot = v*cos(theta) + cross(k,v)*sin(theta) + k*dot(k,v)*(1-cos(theta))
        % Since x_base and y_base are perpendicular to z_axis, dot(z_axis, x_base) = 0
        x_axis_rolled = x_base * cos_roll + cross(z_axis, x_base) * sin_roll;
        y_axis_rolled = y_base * cos_roll + cross(z_axis, y_base) * sin_roll;
        
        % Normalize (should already be unit but ensure)
        x_axis_rolled = x_axis_rolled / norm(x_axis_rolled);
        y_axis_rolled = y_axis_rolled / norm(y_axis_rolled);
        
        % Verify right-handedness: y_axis_rolled should = cross(z_axis, x_axis_rolled)
        % This should be satisfied, but let's ensure
        y_axis_rolled = cross(z_axis, x_axis_rolled);
        y_axis_rolled = y_axis_rolled / norm(y_axis_rolled);
        
        % Build final rotation matrix: columns are x, y, z axes in world frame
        % z-axis points at target, x and y are rotated by roll around z
        R_body2world = [x_axis_rolled, y_axis_rolled, z_axis];
        
        % Extract XYZ Euler angles using MATLAB's built-in function
        % rotm2eul with 'XYZ' returns [X, Y, Z] angles in radians
        eul_rad = rotm2eul(R_body2world, 'XYZ');
        
        % Convert to degrees: [roll, pitch, yaw] = [X, Y, Z]
        in_eul(:, i) = rad2deg(eul_rad);
    end
    
    % Unwrap Euler angles to prevent discontinuities at ±180° wrap-around
    % Convert to radians, unwrap, then convert back to degrees
    in_eul_rad = deg2rad(in_eul);
    in_eul_rad_unwrapped = unwrap(in_eul_rad, [], 2);  % Unwrap along dimension 2 (across waypoints)
    in_eul = rad2deg(in_eul_rad_unwrapped);
    
    % Combine positions (each column is a waypoint: [x; y; z])
    in_pos = [x; y; z];

    
    %Append a stationary phase at the start
    in_times = [0; 0.5;in_times+2];
    in_pos = [[0 0 in_pos(3,1)]', [0 0 in_pos(3,1)]', in_pos];
    in_eul = [[0 0 0]', [0 0 0]', in_eul];
end

