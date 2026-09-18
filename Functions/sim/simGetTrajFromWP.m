function [trajOut, simset] = simGetTrajFromWP(trajWP, simset, cut)
%SIMGETTRAJFROMWP Waypoints -> dense trajectory at simset.simHz.
%   simset.trajInterp: 'minsnap' (default) or 'linear' (fast, pos/eul separate).

        idx_nan = find(isnan(trajWP(1,:)), 1, "first");
        if isempty(idx_nan)
            idx_nan = size(trajWP, 2)+1;
        end

        wp_times = trajWP(1, 1:idx_nan-1);
        wp_pos = trajWP(2:4, 1:idx_nan-1);
        wp_eul = trajWP(5:7, 1:idx_nan-1);

        n = ceil(simset.duration * simset.simHz);
        useLinear = isfield(simset, 'trajInterp') && strcmpi(simset.trajInterp, 'linear');

        if useLinear
            t_q = linspace(0, simset.duration, n);
            wp_times = wp_times(:)';
            traj_pos = interp1(wp_times, wp_pos', t_q, 'linear')';
            traj_eul = interp1(wp_times, wp_eul', t_q, 'linear')';
            traj_times = t_q;
        else
            [traj_pos, ~, ~, ~, ~, ~, ~, traj_times] = minsnappolytraj(wp_pos, wp_times, n);
            traj_eul = minsnappolytraj(wp_eul, wp_times, n);
        end

        if cut
            cutDur = 2;
            numWp = cutDur * simset.simHz;
            traj_pos = traj_pos(:, 1:end-numWp);
            traj_eul = traj_eul(:, 1:end-numWp);
            traj_times = traj_times(:, 1:end-numWp);
            simset.duration = simset.duration - cutDur;
        end

        trajOut = [traj_times; traj_pos; traj_eul];
end
