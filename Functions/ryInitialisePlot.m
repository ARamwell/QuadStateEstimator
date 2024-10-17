function [lineObj, textObj] = ryInitialisePlot(ax)
%RYINITIALISEPLOT Summary of this function goes here
%   Detailed explanation goes here
    
% Plot line
lineObj = plot(ax, NaN, NaN, 'k*');
textObj = text(ax, NaN, NaN, "This");

end

% [lineA, textA] = ryInitialisePlot(trajPlotAx) this is how to call it. 
