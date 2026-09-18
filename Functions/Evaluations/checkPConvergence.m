function [idx_conv, Pchange] = checkPConvergence(Phist)
%CHECKPCONVERGENCE Summary of this function goes here
%   Detailed explanation goes here



    converged = false;
    conv_cnt = 0;
    Pchange = createArray(1,size(Phist,3));
    Pchange(1) = 0;
    

    %we consider that the filter has converged if P changes by less than 5%
    for i=2:size(Phist,3)
        chnge = norm(Phist(:,:,i)-Phist(:,:,i-1))/norm(Phist(:,:,i-1));
        Pchange(i) = chnge;
        conv_i =false;
        
        if chnge < 0.05
            conv_i = true;

            if conv_prev == true
                conv_cnt = conv_cnt+1;
            else
                conv_cnt = 1;
            end
        end

        if conv_cnt ==1
            idx_conv = i;
        end

        if conv_cnt>=20
            converged = true;
        end

        if converged
            break;
        end

        conv_prev = conv_i;

    end

    if ~converged
        idx_conv = [];
    end

end

