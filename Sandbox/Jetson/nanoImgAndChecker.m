function nanoImgAndChecker()
%NANOIMGANDCHECKER Summary of this function goes here
%   Detailed explanation goes here
%#codegen

hwobj = jetson;
%camlist = getCameraList(hwobj);
camName = 'vi-output, imx219 6-0010';
%camRes = [1280 720];
camRes = [640 360];


camObj = camera(hwobj,camName,camRes);
dispObj = imageDisplay(hwobj); %for troubleshooting

for i = 1:100
    img = rot90((snapshot(camObj)));
    
    %img_g = (rgb2gray(snapshot(camObj)));
     
    %img_g = rgb2gray(img);
    %[x_pnts_i, checkerSize_detected] = detectCheckerboardPoints(img_g);
    [id,loc] = readArucoMarker(img,"DICT_4X4_50");
    
    if ~isempty(loc)

        %extract centre points
        loc_c = createArray(2, size(loc, 3));
        for i=1:size(loc, 3)
            loc_c(:,i) = sum(loc(:,:, i), 2)/4;
        end
        %loc_flat = reshape(loc(1,1:2,:), 2, []);


        img_m = insertMarker(img, loc_c);
        %img_m = rgb2gray(img_m);
    else
        img_m = img;
    end

    image(dispObj, img_m);
  
end

            % %Display checkerboard image for this iteration - for troubleshooting
            % if (checkerSize_detected(1) <= checkerSize(1)) || (checkerSize_detected(2) <= checkerSize(1)) 
            %     if i==1
            %         checkerFig = figure();
            %         checkerAx = axes('Parent', checkerFig);
            %         checkerImg = imshow(imgFuncs.markDetectedCheckers(I, x_pnts_i), 'Parent', checkerAx);
            %     else
            %         checkerImg.CData = (imgFuncs.markDetectedCheckers(I, x_pnts_i));
            %         drawnow;
            %     end 
            % end