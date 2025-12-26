function p3pResult_rq2rw = runP3pOnFile(file, camParams_d, K_p3p, T_rq2rc)
    
    refTime = datetime(2000, 01, 01); %if importing simulation data
    featMap = load('C:/Users/Alyssa/Documents/QuadStateEstimator/Resources/featureMap.mat');

       
    [imageStream, imageTime] = imgFuncs.importImageSeq(file, 0, refTime); %returns grayscale
    totalFrames = size(imageTime,2);

    p3pResult_rc2rw = struct('poseArr', createArray(7,4, totalFrames), 'time', createArray(1,totalFrames));
    err=createArray(3,1);
    %p3pResult_rc2rw.time = createArray(1, 0, 'datetime');
    %p3pResult_rc2rw.time = datetime(p3pResult_rc2rw.time, 'Format', 'yyyyMMdd_HHmmss_SSS');
    
    for i=1:totalFrames
        %disp(strcat("Working on frame ", string(i)));
        
        %Get current frame
        img=imageStream(:,:,i);
        img_g = img;
        %img_g = rgb2gray(img);
        %img_g = undistortImage(img, camParams_d);
        %imshow(img_g);

        %Find Aruco corners
        [x_pnts_i, X_pnts_W] = featureDetectMatch(img_g, featMap.featureMap);
        % if ~isnan(x_pnts_i(1,1))
        %     colors = {"red","white","green","magenta"};
        %     img_s = insertMarker(img_g,x_pnts_i',"o",MarkerColor=colors,Size=10);
        %     imshow(img_s);
        % end
        
        %run Kneip's P3P
        p3pResult_rc2rw.time(i) = refTimeToElapsedTimeDouble(imageTime(i), refTime);
        if ~isnan(x_pnts_i(1,1)) 
            soln = p3pRun.KneipN(x_pnts_i, X_pnts_W, K_p3p, 4); %can be streamlined, currently much overhead: outputs struct with poses, Rt matrices, most inliers, least reproj...
            p3pResult_rc2rw.poseArr(:,:,i) = soln.poseArr;   
        end
        
    end
    
    p3pResult_rq2rw = p3pFuncs.convSolnFrame(p3pResult_rc2rw, T_rq2rc);  

    %import groundtruth
    simout = load(strcat(file, '\simout.mat'));
    simout = simout.simout;
    groundTruth = processSimGroundTruth(simout);
    indices = selectClosestTimeIndices(p3pResult_rq2rw.time, groundTruth.quad.time);
    p3pResult_rq2rw.truePose = groundTruth.quad.state(1:7,indices);
    %p3pResult_rc2rw.truePose = groundTruth.cam.state(1:7,:);

    %find best solution
    for i = 1: size(p3pResult_rq2rw.time, 2)
        [p3pResult_rq2rw.selected(:,i), err(:,i)] = chooseMinPoseErr(p3pResult_rq2rw.poseArr(:,:,i), p3pResult_rq2rw.truePose(:,i), 1.2, 3);
    %    [p3pResult_rc2rw.selected(:,i), err_c(:,i)] = chooseMinPoseErr(p3pResult_rc2rw.poseArr(:,:,i), p3pResult_rc2rw.truePose(:,i), 1.2, 0);
    end
    %disp(err);


end

 