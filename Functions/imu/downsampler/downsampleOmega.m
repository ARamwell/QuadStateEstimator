function [accumAngle_f, accumAngle_c] = downsampleOmega(w_k, dt, coningActive, resetFlag)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
persistent w_prev
persistent dPhi_fir_accum
persistent dPhi_con_accum
persistent dPhi_fir_prev

    if (isempty(w_prev) || resetFlag )
        w_prev = w_k;%zeros(3,1);
        dPhi_fir_accum= zeros(3,1);
        dPhi_con_accum= zeros(3,1);
        dPhi_fir_prev= zeros(3,1);
    end

    dPhi_fir_k = 0.5 * (w_k + w_prev) *dt;    %get first-order rotation element, trapezoidal integration
    R_prev2k = rotvec2mat3d(deg2rad(dPhi_fir_k));
    accumAngle_f = R_prev2k*dPhi_fir_accum + dPhi_fir_k;
    
    
    if coningActive == true
        dPhi_con_k = 0.5 * cross( (dPhi_fir_accum + 1/6 * dPhi_fir_prev), dPhi_fir_k);  % get current coning contribution
        %dPhi_con_k = dt * dPhi_con_k;
        accumAngle_c = R_prev2k*dPhi_con_accum + dPhi_con_k; %update accumulated coning contribution
    else
        accumAngle_c = zeros(3,1);
    end

    w_prev = w_k;
    dPhi_fir_prev = dPhi_fir_k;
    dPhi_fir_accum = accumAngle_f;
    dPhi_con_accum = accumAngle_c;
    
end

