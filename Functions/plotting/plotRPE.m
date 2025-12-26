function [rpeAx] = plotRPE(figObj, trajErrMetrics, qualifier)
    
    figure(figObj);
    rpeAx = plot(trajErrMetrics, "absolute-translation");
    set(rpeAx, 'Ydir', 'reverse');
    set(rpeAx, 'Zdir', 'reverse');
    view(rpeAx, [-57 30]);
    %title(strcat('Relative Pose Error - ', qualifier));

end
