        function [rtHist, quadStateHist_times, quadStateHist] = importSimLog_gt(fullFile, refTime)
        %Basic function to import all logged trajectory data (as .m file). 
        %Does not consider time alignment with other data (i.e., imports 
        %all logged points, does not skip any)
            %refTime = datetime(2000, 01, 01);

            simData = load(fullFile);
            
            %% Initialise variables
            %Get data sizes
            numQuadLogPoints = size((simData.out.quadState.signals.values), 1);
            quadStateDimensions = size((simData.out.quadState.signals.values), 2);
            %create arrays
            rtHist = zeros(3,4,numQuadLogPoints);%-1 to skip t0
            quadStateHist =zeros(quadStateDimensions, (numQuadLogPoints));
            quadStateHist_times = createArray(1, numQuadLogPoints, 'datetime');
            quadStateHist_times = datetime(quadStateHist_times, 'Format', 'yyyyMMdd_HHmmss_SSS');

            %% Import Quad Ground Truth data
            for i=1:numQuadLogPoints
                 %Import time history
                state_time = simData.out.quadState.time(i,1);
                quadStateHist_times(1,(i)) = datetime(refTime+seconds(state_time), 'Format', 'yyyyMMdd_HHmmss_SSS');

                % Import position log
                trans_quad = transpose(simData.out.quadState.signals.values(i,1:3)); %in m
             
                %If in quaternions (new implementation)
                %import orientation log
                q = simData.out.quadState.signals.values(i, 4:7);
                j = 8;
                orient = transpose(q);

                %convert to rotation matrix
                R_quad =(quat2rotm(q));

                % Import velocity log
                x_dot = simData.out.quadState.signals.values(i,j);
                y_dot = simData.out.quadState.signals.values(i,j+1);
                z_dot = simData.out.quadState.signals.values(i,j+2);

                % Import bias log
                j=11;
                b_a = simData.out.quadState.signals.values(i,j:j+2);
                b_g = simData.out.quadState.signals.values(i,j+3:j+5);
                
                %Buld ground truth Rt history
                rtHist(1:3,1:3,i) = R_quad;
                rtHist(1:3,4,i) = trans_quad;
                quadStateHist(:,i) = ([trans_quad; orient; x_dot; y_dot; z_dot; b_a'; b_g']);
            end      
        
        end