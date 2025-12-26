function [nees] = evalNEES_noq(estStateHist, estCov, trueStateHist)
%will need groundtruth to be time-aligned in advance
    
    [ekfStates, numUpdates] = size(estStateHist);
    %calculate NEES
    for t=1:numUpdates
        x_err =  trueStateHist(1:ekfStates,t) - estStateHist(:,t);
        x_err(4,:) = [];
        effCov = estCov(:,:,t);
        effCov(4,:) = [];
        effCov(:,4) = [];
        nees(t) = x_err' * (effCov \ x_err);
    end

end