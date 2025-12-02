function [ekfResult, p3pResult, groundTruth] = processSimData(out, simset)

    groundTruth = processSimGroundTruth(out);

    [ekfResult, p3pResult] = processSimEstimatorData(out, simset, groundTruth);

end




