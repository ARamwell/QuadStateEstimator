        function [imu_readHist, imu_timeHist] = importSimLog_imu(fullFile, refTime)
        %Basic function to import all logged trajectory data (as .m file). 
        %Does not consider time alignment with other data (i.e., imports 
        %all logged points, does not skip any)
            %refTime = datetime(2000, 01, 01);

            simData = load(fullFile);
           
            %% Initialise variables
            numImuReadPoints = size(simData.out.IMU.signals.values, 1);
            imu_readHist =zeros(6, numImuReadPoints);
            imu_timeHist = createArray(1, numImuReadPoints, 'datetime');
            imu_timeHist = datetime(imu_timeHist, 'Format', 'yyyyMMdd_HHmmss_SSSSSS');
                                         
            %% Import IMU readings
            for i=1:numImuReadPoints
                % Import imu log
                imuState = simData.out.IMU.signals.values(i, :);
                imuState_time = simData.out.IMU.time(i, 1);
                %build history
                imu_timeHist(1,i)= datetime(refTime+seconds(imuState_time), 'Format', 'yyyyMMdd_HHmmss_SSSSSS');
                imu_readHist(1:end, i)=transpose(imuState); 

            end
        end
