function [ekfResult, p3pResult, groundTruth] = processSimData(out, simset, estset)

    groundTruth = processSimGroundTruth(out);

    [ekfResult, p3pResult] = processSimEstimatorData(out, estset, groundTruth);

end




