function [vioPercent] = evalPercentDivergence(x_, x, P_, numSDs)
%EVALPERCENTDIVERGENCE Summary of this function goes here
%   Detailed explanation goes here

vio = zeros(size(x_));
diff = createArray(size(x_));

for t=1:size(x_, 2)

    if dot(x_(4:7,:), x((4:7),:)) < 0
        x((4:7),:) = -x((4:7),:);
    end
    diff(:,t) = x(:,t) - x_(:,t);

    for i = 1:size(diff, 1)        
        dev = sqrt(abs(P_(i,i,t)));
        if abs(diff(i,t))>(dev*numSDs)
            vio(i,t) = 1;
        end
    end

end

vioPercent = sum(vio,2)/size(vio,2);

