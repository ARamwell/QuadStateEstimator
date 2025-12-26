function [groundTruth, imuData, ekfResult, p3pResult] = processSimData(out, runEKF, runP3P)

    groundTruth = processSimGroundTruth(out);

    imuData= processSimImu(out);

    [ekfResult, p3pResult] = processSimEstimatorData(out, runEKF, runP3P, groundTruth);

end




