function [x_pnts_i, X_pnts_W] = featureDetectMatch(I, featureMap)
%FEATUREDETECTOR Summary of this function goes here
%   Function wrapper to enable codegen for detectCheckerboardPoints.
%   Creates predictable-sized output.

    %initialise variables
    x_pnts_i = NaN(2, 4);
    X_pnts_W = NaN(3, 4);
    
    %run detection
    [x_pnts_i(1:2, 1:4), id] = detectAruco(I, x_pnts_i); 
    %[x_i, X_W] = detectChecker(I, featureMap);
    %x_pnts_i = x_i';
    %X_pnts_W = X_W';
    

    %if any tags detected, match them
    if ~isnan(x_pnts_i(1,1))
        %img_m = insertMarker(I, x_pnts_i(:,3)');
        %imshow(img_m)
       X_pnts_W(1:3, 1:4) = matchAruco(featureMap.arucos, id);
       if isnan(X_pnts_W(1,1))
           x_pnts_i = NaN(2,4);
       end
    end



end

function [x_pnts_i, X_pnts_W] = detectChecker(I, featureMap)

    checkerSize = [5, 8];
    x_pnts_i = NaN(1,2);
    X_pnts_W = NaN(1,3);

    [x_pnts_i_det, checkerSize_detected] = detectCheckerboardPoints(I);

    if checkerSize_detected == checkerSize
        
        x_pnts_i = x_pnts_i_det;
        % for i=1:size(x_pnts_i_det,1)
        %    x_pnts_i(end+1,:) = x_pnts_i_det(i,:);
        % end
        X_pnts_W  = featureMap.checkers(1).corners';
    end
    
end

function [x_pnts_i, tagID] = detectAruco(I, x_pnts_i)

    tagID = NaN;
    [id,loc] = readArucoMarker(I,"DICT_4X4_50");

    %extract centre points
    % loc_c = createArray(2, size(loc, 3));
    % for i=1:size(loc, 3)
    %     loc_c(:,i) = sum(loc(:,:, i), 2)/4;
    % end
    % %loc_flat = reshape(loc(1,1:2,:), 2, []);

    
    if ~isempty(loc)
          x_pnts_i(1:2, 1:4) = loc(1:4,1:2)';
          tagID = id;
    end
    
end

function X_pnts_W = matchAruco(map_arucoList, tagID)

    X_pnts_W = NaN(3, 4);

    %Loop through list
    for i = 1:size(map_arucoList,2)
        %check if tag is there
        if map_arucoList(i).id == tagID
            X_pnts_W(1:3, 1:4) = (map_arucoList(i).corners);
            return; % Field found, exit
        end
    end

end

