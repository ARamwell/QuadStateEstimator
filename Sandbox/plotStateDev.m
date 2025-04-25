function [outputArg1,outputArg2] = plotStateDev(figHandle, estStateVariableHist, estStateCovHist, corrHist, trueStateVariableHist)
%PLOTSTATEDEV Function to plot state variable over time, including
%covariance and correction steps where they occur
%   Detailed explanation goes here

figObj = figure();
axObj = axes('Parent', figObj);
%axis equal;
%xlim(axObj, xLim);

%plot changing state
x = estStateVariableHist;
y = 

%plot variance
x = linspace(0,1,20)';
y = sin(x);
dy = .1*(1+rand(size(y))).*y;  % made-up error values
fill([x;flipud(x)],[y-dy;flipud(y+dy)],[.9 .9 .9],'linestyle','none');
line(x,y)



end

