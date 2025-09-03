function [bestSolns] = findObjBestP3P(p3pSolns, p3pTimes, truePose, trueTimes, Rt_rc2rq)
%FINDBESTP3P Summary of this function goes here
%   Detailed explanation goes here

bestSolns = createArray(7, size(p3pTimes,2));


for t=1:size(p3pTimes, 2) 

    [closestDiff, closestIndex] = min(abs(trueTimes(1,:)-p3pTimes(t))); %find closest true timestamp to current p3p timestamp
    truePose_t = truePose(:,closestIndex);

    p3pPoses_t = p3pSolns(:,:,t);

    [bestPose, bestErr, bestIndex] = chooseMinPoseErr(p3pPoses_t, truePose_t, 1.2, 2);
    
    bestSolns(:,t)=bestPose;
end
   

end

