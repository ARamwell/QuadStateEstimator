function resultStruct = calcRtError(resultStruct, Rt_hist_true, time_hist_true)

    methodsUsed = fieldnames(resultStruct);

    for m = 1: length(methodsUsed)

        methodName = methodsUsed{m};
        
        currentTime = resultStruct.(methodName).time(1,end);

        [~, closestIndex] = min(abs(time_hist_true-currentTime));

        Rt_true = Rt_hist_true(1:3, 1:4, closestIndex);
        Rt_calc = resultStruct.(methodName).Rt(1:3, 1:4, end);

        errorCalc = p3pFuncs.getOrientErrArr(Rt_true, Rt_calc);

        resultStruct.(methodName).error(:, end+1) = errorCalc;


    end



end