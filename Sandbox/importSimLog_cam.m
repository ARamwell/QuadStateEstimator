        function [rtHist, cam_timeHist, cam_stateHist] = importSimLog_gt(fullFile, refTime)
        %Basic function to import all logged trajectory data (as .m file). 
        %Does not consider time alignment with other data (i.e., imports 
        %all logged points, does not skip any)
            %refTime = datetime(2000, 01, 01);

            simData = load(fullFile);
           
            %% Initialise variables
            %for camera
            numCamLogPoints = size((simData.out.camState_GT.signals.values), 3);
            cam_timeHist = zeros(1, numCamLogPoints);
            %cam_stateHist = zeros(1, numCamLogPoints);

            %% Import Camera Ground Truth data
            for i=1:numCamLogPoints

                %import time
                cam_time = simData.out.camState_GT.time(i,1);
                cam_timeHist(1,(i)) = datetime(refTime+seconds(cam_time), 'Format', 'yyyyMMdd_HHmmss_SSS');
            
                % Import position log
                trans_cam = transpose(simData.out.camState_GT.signals.values(1,1:3,i)); %in m

                %If in quaternions (new implementation)
                    %import orientation log
                    q = simData.out.camState_GT.signals.values(1, 4:7, i);
                    j = 8;
                    orient = transpose(q);

                    %convert to rotation matrix
                    R_cam = quat2rotm(q);

                % Import velocity log
                x_dot = simData.out.camState_GT.signals.values(1,j,i);
                y_dot = simData.out.camState_GT.signals.values(1,j+1,i);
                z_dot = simData.out.camState_GT.signals.values(1,j+2,i);

                %Buld ground truth Rt history
                rtHist(1:3,1:3,i) = R_cam;
                rtHist(1:3,4,i) = trans_cam;
                cam_stateHist(:,i) = ([trans_cam; orient; x_dot; y_dot; z_dot]);
            end

        end