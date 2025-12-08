function [meanData, varData, figObj] = plotGaussNoise(data, figObj, labelX, labelY, tit)
%PLOTGAUSSNOISE Summary of this function goes here
%   Detailed explanation goes here

meanData = mean(data);
stdData = std(data);

%figObj = figure();
plot(data, Color='#FFB14E');
ylabel(labelX);
xlabel(labelY);
title(tit);

plotMeanAndStd(figObj, meanData, stdData, '#0000e3', '#9D02D7')


end

