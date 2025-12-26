function [outputArg1,outputArg2] = runEKFCoreMetrics(ekfResult)
%RUNEKFCOREMETRICS Summary of this function goes here
%   Detailed explanation goes here

trajErr = evaluateTrackingPerformance(ekfResult.x_, ekfResult.trueState, 'none');
nees = evalNEES_noq(ekfResult.x_, ekfResult.PHat, ekfResult.trueState);



end

