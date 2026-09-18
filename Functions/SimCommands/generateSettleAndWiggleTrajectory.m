function [in_times, in_pos, in_eul] = generateSettleAndWiggleTrajectory(hoverHeight, exploreRadius)
%GENERATESETTLEANDWIGGLETRAJECTORY Waypoints for SettleAndWiggle (~60 s).
%   0-10 s: stationary; 10-30 s: small square motion; 30-60 s: fixed position, wiggle attitude.
%   Use with simset.trajInterp = 'linear'. Layout: in_times 1xN, in_pos/in_eul 3xN.

    if nargin < 1 || isempty(hoverHeight), hoverHeight = 1.8; end
    if nargin < 2 || isempty(exploreRadius), exploreRadius = 0.35; end

    z = -hoverHeight;
    r = exploreRadius;
    target = [0; 0; 0];

    % 0-10 s hover; 10-30 s square; 30-60 s hold position, rotate in place
    in_times = [0, 10, 15, 20, 25, 30, 45, 60];

    in_pos = [ ...
        0,  0,  r,  r,  0,  0,  0,  0; ...
        0,  0,  0,  r,  r,  0,  0,  0; ...
        z,  z,  z,  z,  z,  z,  z,  z];

    in_eul = zeros(3, numel(in_times));
    for k = 1:6
        in_eul(:, k) = eulerLookAtTarget(in_pos(:, k), target, 0);
    end
    e0 = in_eul(:, 6);
    in_eul(:, 7) = e0 + [90; 45; 0];
    in_eul(:, 8) = e0 + [120; 60; 180];
end

function eulDeg = eulerLookAtTarget(pos, target, rollDeg)
    pos = pos(1:3);
    vec_to_target = (target - pos) / norm(target - pos);

    roll_rad = deg2rad(rollDeg);
    z_axis = vec_to_target;
    up_ref = [0; 0; -1];

    x_axis = cross(up_ref, z_axis);
    if norm(x_axis) < 1e-6
        x_axis = cross([1; 0; 0], z_axis);
    end
    x_axis = x_axis / norm(x_axis);

    x_axis_rolled = x_axis * cos(roll_rad) + cross(z_axis, x_axis) * sin(roll_rad);
    y_axis_rolled = cross(z_axis, x_axis_rolled);
    y_axis_rolled = y_axis_rolled / norm(y_axis_rolled);
    x_axis_rolled = x_axis_rolled / norm(x_axis_rolled);

    eulDeg = rad2deg(rotm2eul([x_axis_rolled, y_axis_rolled, z_axis], 'XYZ'))';
end
