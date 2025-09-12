function [x_pnts_i] = featureDetector(I, checkerSize)
%FEATUREDETECTOR Summary of this function goes here
%   Function wrapper to enable codegen for detectCheckerboardPoints.
%   Creates predictable-sized output.

x_pnts_i = NaN(10*10,2);

[x_pnts_i_det, checkerSize_detected] = detectCheckerboardPoints(I);

if checkerSize_detected == checkerSize
    for i=1:size(x_pnts_i_det,1)
       x_pnts_i(i,1:2) = x_pnts_i_det(i,1:2);
    end
end

end

