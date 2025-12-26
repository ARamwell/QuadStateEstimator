function [eul, eulCov] = quatEst2eulEst(ekfResult)
    %EKF estimates
    numLoops = size(ekfResult.time, 2);
    eulCov = zeros(3,3,numLoops);
    eul = zeros(3,numLoops);
    for j=1:numLoops
    
        %convert quaternion estimate to euler
        eul(:,j) = rad2deg(quat2eul(ekfResult.x_(4:7, j)', 'XYZ'))'; %convert quaternion to euler
        q = ekfResult.x_(4:7, j);
    
        %propagate covariance
        orientCov_quat = ekfResult.P(4:7, 4:7, j);
        jacob = jacobianEultoQuat(q);
        eulCov(:,:,j) = rad2deg(jacob * orientCov_quat * jacob');
    
    end
    
end
