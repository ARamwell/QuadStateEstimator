function [flag_constVelDetected, T_a, T_w] = constVelDetector(newImu, a_thresh, omega_thresh, n_window, iter)

flag_constVelDetected = false;
T_a = nan;
T_w=nan;

if iter==1
    clear buff headPtr
end

persistent buff;
persistent headPtr;

if size(buff,2) < n_window-1
    buff(:, end+1) = newImu; %fill buffer first
    headPtr=1;
else
    if size(buff, 2)== n_window-1
        buff(:, end+1) = newImu; %fill buffer first
        headPtr=1;
    else
        %update buffer, FIFO
        buff(:,headPtr) = newImu;
        headPtr = mod(headPtr, n_window) + 1; 
    end
    %once buffer is full, start calculating metrics
    w_mean = mean(buff(1:3, :), 2);
    a_var=var(buff(4:6,:), 0, 2);
    T_a = sum(a_var);
    T_w = sum(abs(w_mean));

    if ((T_a<=a_thresh) && (T_w<omega_thresh))
        flag_constVelDetected = true;
    end

end

end