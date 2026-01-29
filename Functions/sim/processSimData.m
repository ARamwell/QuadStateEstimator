function [groundTruth, imuData, ekfResult, p3pResult] = processSimData(out, runEKF, runP3P, diffHz)

    groundTruth = processSimGroundTruth(out);

    imuData(1) = processSimImu(out.IMU);
    if diffHz ==  true
        imuData(2) = processSimImu(out.IMU_2kHz);
        %fix some things that were wrong in the first version
        imuData(2).rawdata = downsample(imuData(2).rawdata',4)';
        imuData(2).time = downsample(imuData(2).time',4)';
        imuData(2).rawdata = [imuData(2).rawdata(4:6,:); imuData(2).rawdata(1:3,:)] ;
    end

    [ekfResult, p3pResult] = processSimEstimatorData(out, runEKF, runP3P, groundTruth);

end




