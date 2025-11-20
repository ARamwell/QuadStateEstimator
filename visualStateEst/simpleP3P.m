function [poseArr] = simpleP3P(x_i, X_W, K)
%SIMPLEP3P Streamlined Kneip's P3P for embedded hardware. 
%   Detailed explanation goes here

%variables
inlierThreshold = 1;
poseArr = nan(7, 4);
%soln = struct('T_arr', nan(4,4,4), 'poseArr', nan(7,4,4), 'mostInliers', struct('T', nan(4,4), 'Num', []), 'minReproj', struct('T', nan, 'Err', []));

if ~isnan(x_i(1,1))
   
    soln = p3pRun.KneipN(x_i, X_W, K, inlierThreshold); %can be streamlined, currently much overhead: outputs struct with poses, Rt matrices, most inliers, least reproj...
    
    for j = 1:size(poseArr,2)
        poseArr(1:7,j) = soln.poseArr(1:7,j); %all rc2rw
        poseArr(1:3,j)= poseArr(1:3,j); %convert to m
    end
end

end

