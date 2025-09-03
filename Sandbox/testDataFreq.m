
function [meanDelay, v] = testDataFreq(data_timestamps)
    dataTimes = data_timestamps;
    dt = createArray(1, size(dataTimes, 2), 'double');
    
    for t=1:(size(dataTimes, 2)-1)
        dt(1,t) = milliseconds(dataTimes(1,t+1) - dataTimes(1,t))/1000;
    end
    
    v = var(dt);
    meanDelay = mean(dt);
end
