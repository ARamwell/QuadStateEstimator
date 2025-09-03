clear all
% q_est q_true ekfData p3pData

q_est = createArray(4,0);
q_true = createArray(4,0);

%t_est = 
%t_true

%% Find all subfolders
parentFolder = 'C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\';
allSubFolders = genpath(parentFolder);
% Parse into a cell array.
remain = allSubFolders;
listOfFolderNames = {};
while true
	[singleSubFolder, remain] = strtok(remain, ';');
	if isempty(singleSubFolder)
		break;
    end
    if ~contains(singleSubFolder, 'arb')  && contains(singleSubFolder, 'static') && ~contains(singleSubFolder, 'sim')
	    listOfFolderNames = [listOfFolderNames singleSubFolder];
    end
end
numberOfSubFolders = length(listOfFolderNames);
% 
  % for k=2:numberOfSubFolders
  %    currentFolder = string(listOfFolderNames(k));

    currentFolder = fullfile('C:\Users\Alyssa\Documents\QuadStateEstimator\Tests\RobMech\Dynamic\TestSeries_3\pitchforward_high1\sim_16Hz');
    
    basename = 'ekfResult_10el_rect';
    filename = strcat('\', basename, '.mat');
    %ekfData = load(fullfile(currentFolder, filename));
    p3pData = load(fullfile(currentFolder, '\p3pResult_realigned.mat'));
    
    %USE WAHBA's PROBLEM TO GET AVERAGE ROTATION BETWEEN THESE FRAMES

    %q_est = [q_est ekfData.ekfResult.stateEst(4:7,:)];
    %q_true = [q_true ekfData.ekfResult.truePose(4:7,:)];

    q_est = [q_est p3pData.p3pResult.KneipN.best.quadPose(4:7,:)];
    q_true = [q_true p3pData.p3pResult.KneipN.best.truePose(4:7,:)];
% end

%% First, change to rotation matrices
n = size(q_est, 2); % number of samples
R_gt = zeros(3, 3, n);
R_est = zeros(3, 3, n);

for i = 1:n
    R_est(:,:,i) = quat2rotm(q_est(:, i)');
    R_gt(:,:,i) = quat2rotm(q_true(:, i)');

    q_est2gt(:,i) = quatmultiply(quatconj(q_true(:, i)'), q_est(:, i)')';
    
    %Enforce quaternion constraints - closest quaternions
    if i>1 && dot(q_est2gt(:,i-1), q_est2gt(:,i)) < 0
        q_est2gt(:,i) = - q_est2gt(:,i);
    end
    angle_est2gt(:,i) = 2*acosd(q_est2gt(1,i)); 
    if angle_est2gt(:,i) > 180
        angle_est2gt(:,i) = angle_est2gt(:,i) -360;
    end
    axis_est2gt(:,i) = q_est2gt(2:4,i)/sind(angle_est2gt(:,i)/2);

end

%% Find optimal rotation matrix using SVD
A = zeros(3,3);
for i = 1:n
    A = A + R_gt(:,:,i) * R_est(:,:,i)';
end

[U, ~, V] = svd(A);
R_align = U * V';

% Ensure it's a proper rotation matrix (det = +1)
if det(R_align) < 0
    U(:,3) = -U(:,3);
    R_align = U * V';
end

%% Convert to quat
q_align1 = rotm2quat(R_align);

%% Use axis angle representation instead
%angle_est2gt = unwrap(angle_est2gt, [], 2);
angle_mean = mean(angle_est2gt, 2);
angle_std = std(angle_est2gt, [], 2);
axis_mean = mean(axis_est2gt, 2);
axis_std = std(axis_est2gt, [], 2);

q_av_est2gt = [cosd(angle_mean/2); (axis_mean/norm(axis_mean)) * sind(angle_mean/2)];
q_av_est2gt = q_av_est2gt/norm(q_av_est2gt);
R_align2 = quat2rotm(q_av_est2gt');
