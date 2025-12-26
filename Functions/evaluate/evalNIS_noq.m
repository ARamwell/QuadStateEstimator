function [nis] = evalNIS(meas_resid, meas_cov)
%will need groundtruth to be time-aligned in advance
    
    idx = ~isnan(meas_resid(1,:));
    y_clean = meas_resid(:,idx);   
    cov_clean = meas_cov(:,:, idx);
    numUpdates = size(y_clean, 2);
    
    %calculate NIS 
    for t=1:numUpdates

        %remove q_0
        eff_y = y_clean(:,t); 
        eff_y(4,:) = [];
        effCov = cov_clean(:,:,t);
        effCov(4,:) = [];
        effCov(:,4) = [];
        %will nans be a problem?
        nis(t) = eff_y' * (effCov \ eff_y);
    end
    
end