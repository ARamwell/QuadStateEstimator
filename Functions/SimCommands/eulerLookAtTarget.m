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
