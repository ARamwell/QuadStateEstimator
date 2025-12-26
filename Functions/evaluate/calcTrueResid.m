function [y_true] = calcTrueResid(meas, state_true)
%CALCTRUERESID Summary of this function goes here
%   Detailed explanation goes here


    %idx = ~isnan(meas(1,:));
    %meas_clean = meas(:,idx);   
    %gt_clean = groundtruth(:,:, idx);
    %numUpdates = size(meas_clean, 2);

    xhowBig = size(state_true, 1);
    xSizeStr = strcat(string(xhowBig), 'el');
    y_true = createArray(size(meas));
        
    for t = 1:size(meas,2)
        x = state_true(:,t);
        z = meas(:,t);

        z_pred = feval(strcat('ekf_measModel_', xSizeStr), x); %get predicted measurement
        
        y_true(:,t) = z - z_pred;
    
    end

