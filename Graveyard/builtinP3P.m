function A = builtinP3P(imagePts_vert, worldPts_vert, focalLngth, principalPnt, imageSize, estTranslationVec, estRotationMat, camPlot)
    
    intr = cameraIntrinsics(focalLngth, principalPnt, imageSize);

    worldPose = estworldpose(imagePts_vert, worldPts_vert, intr, 'MaxReprojectionError', 0.2);
    A = worldPose.A;

    if camPlot == 1
        figure;
        pcshow(worldPts_vert,VerticalAxis="Y",VerticalAxisDir="down", ...
        MarkerSize=30);
        
        hold on
        plotCamera(Size=10,Orientation=worldPose.R', ...
            Location=worldPose.Translation);
        hold on
        plotCamera(Size=10,Orientation=estRotationMat', ...
            Location=estTranslationVec,Color=[0 1 0]);
        hold off
    end 

end