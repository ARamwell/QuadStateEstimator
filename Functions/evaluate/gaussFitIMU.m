function [figObj2, stats] = gaussFitIMU(data)
%GAUSSFITP3P Summary of this function goes here
%   Detailed explanation goes here

    figObj2 = figure();
    tileLayout_cust = tiledlayout(figObj2, 1, 3);

    statevar= {"$x$", "$y$", "$z$"};

    for i=1:3
        
        nexttile();
        
        %tit = statevar{i};
        histfit(data(i,:));
        title(statevar{i});

        pdFit = fitdist(data(i,:)', 'Normal');

        stats(i) = pdFit;

   end
        
end

