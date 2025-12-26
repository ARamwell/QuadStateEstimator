function [newTime] = simTimeToRefTime(simTime, refTime, format)
%SIMTIMETOREFTIME Summary of this function goes here
%   Detailed explanation goes here
    
    newTime = createArray(1, 0, 'datetime');
    newTime = datetime(newTime, 'Format', format);

    for t=1:size(simTime,2)
        newTime(t) = datetime(refTime+seconds(simTime(t)), 'Format', format);
    end

end

