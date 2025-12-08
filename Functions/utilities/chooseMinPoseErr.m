function [pq_best,outErr, index] = chooseMinPoseErr(pq_arr,pq_comp,p_weight, q_weight)
%CHOOSEMINPOSEERR Summary of this function goes here
%   Detailed explanation goes here

minErr = 10000;
pq_best = nan(7,1);

for i=1:size(pq_arr, 2)
    [posErr, orientErr] = getPoseError(pq_arr(:, i), pq_comp);
    totalErr = abs(posErr*p_weight) + abs(deg2rad(orientErr)*q_weight);

    if i==1 || totalErr<minErr
        minErr = totalErr;
        outErr = [posErr, orientErr, totalErr];
        pq_best = pq_arr(:,i);
        index=i;
    end
    

end

