function [trajOut, simset] = simGetTrajFromWP(trajWP, simset, cut)
%SIMGETTRAJFROMWP Summary of this function goes here
%   Detailed explanation goes here
        idx_nan = find(isnan(trajWP(1,:)), 1, "first");
        if isempty(idx_nan)
            idx_nan = size(trajWP, 2)+1;
        end

        wp_times = trajWP(1,1:idx_nan-1);
        wp_pos = trajWP(2:4,1:idx_nan-1);
        wp_eul=trajWP(5:7,1:idx_nan-1);

        [traj_pos, ~, ~, ~, ~, ~, ~, traj_times] = minsnappolytraj(wp_pos, wp_times, ceil(simset.duration*simset.simHz));
        traj_eul = minsnappolytraj(wp_eul, wp_times, ceil(simset.duration*simset.simHz));

        if cut
            cutDur = 2; %ut off last 2 seconds
            numWp = cutDur*simset.simHz;
            traj_pos = traj_pos(:,1:end-numWp);
            traj_eul = traj_eul(:,1:end-numWp);
            traj_times = traj_times(:,1:end-numWp);
            simset.duration = simset.duration-cutDur;
        end

        trajOut= [traj_times; traj_pos; traj_eul];
end

