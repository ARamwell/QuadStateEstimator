function [areAx] = plotARE(figObj, trajErrMetrics, qualifier)
    
    figure(figObj);
    areAx = plot(trajErrMetrics, "absolute-rotation");
    set(areAx, 'Ydir', 'reverse');
    set(areAx, 'Zdir', 'reverse');
    view(areAx, [-57 30]);
    %title(strcat('Absolute Rotation Error - ', qualifier));
    hcb = colorbar;
    hcb.Title.String = "Absolute rotation error (degrees)";
    hcb.Title.Rotation = 90;
    hcb.Title.Position = [-3, 0];
    hcb.Title.VerticalAlignment = 'bottom';
    hcb.Title.HorizontalAlignment = 'left';

end