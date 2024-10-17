
newFig = figure();

newAx = p3pPlotting.initPlot(newFig, [0,500], [0,500], [0 500]);

[trajLine, axesTextArr, axesLineArr] = p3pPlotting.addTraj(newAx, 'plot1', 'or', '-r');

checkerPlot = p3pPlotting.addCheckerboard(newAx, X_pnts_W);

Rt = [1 0 0 300; 0 1 0 300; 0 0 1 300];

p3pPlotting.updateTraj(trajLine, axesTextArr, axesLineArr, Rt);


