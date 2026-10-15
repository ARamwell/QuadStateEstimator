function [z_arr, z_timeHist, z_best] = preprocessP3pResult(p3pResult, groundTruth)

        z_timeHist = p3pResult.time;
        z_arr = p3pResult.poseArr;
        z_best = nan(7,size(z_timeHist, 2));
        for t=1:size(z_timeHist,2)
            [closestDiff, closestIndex] = min(abs(groundTruth.quad.time(1,:)-z_timeHist(1,t))); %find index of closest time
            %idx_gt = find(groundTruth.quad.time(1,:)==z_timeHist(1,t), 1, "first");
            z_best(:,t) = chooseMinPoseErr(z_arr(:,:,t), groundTruth.quad.state(1:7, closestIndex), 1.2, 2);
        end
        %z_best = p3pResult.selected;

end