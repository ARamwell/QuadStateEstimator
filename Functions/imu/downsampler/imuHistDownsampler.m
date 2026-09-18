function [imuHist_ds, imuTimes_ds] = imuHistDownsampler(imuHist, imuTimes, oldRate, newRate, corrFlag)

    numToCombine = ceil(oldRate/newRate);
    cnt = 0;
    dt = 1/oldRate;
    a_down = createArray(3,0);
    w_down = createArray(3,0);
    
    for t=1:size(imuHist, 2)
        w_new = imuHist(1:3, t);
        a_new = imuHist(4:6,t);
        reset = 0;
        if cnt >= numToCombine
            a_down(:,end+1) = accumVel/(dt*cnt);
            w_down(:,end+1) = accumAngle/(dt*cnt);
            reset = 1;
            cnt = 0;
        end
        [accumVel, accumAngle] = accumImu(a_new, w_new, dt, corrFlag, corrFlag, reset);
        cnt = cnt+1;
    end

    imuHist_ds = [w_down; a_down];
    imuTimes_ds = downsample(imuTimes', numToCombine, 0)';

end