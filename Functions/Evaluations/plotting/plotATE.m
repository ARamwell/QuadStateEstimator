function [ateAx] = plotATE(figObj, trajErrMetrics, qualifier)
    
    figure(figObj); %make current figure
    ateAx = plot(trajErrMetrics, "absolute-translation");
    set(ateAx, 'Ydir', 'reverse');
    set(ateAx, 'Zdir', 'reverse');
    view(ateAx, [-57 30]);
    %title(strcat('Absolute Translation Error - ', qualifier));
    hcb = colorbar;
    hcb.Title.String = "Absolute translation error (m)";
    hcb.Title.Rotation = 90;
    hcb.Title.Position = [-3, 0];
    hcb.Title.VerticalAlignment = 'bottom';
    hcb.Title.HorizontalAlignment = 'left';

end