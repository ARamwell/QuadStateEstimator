function [figObj2, stats] = gaussFitIMU(data)
%GAUSSFITP3P Summary of this function goes here
%   Detailed explanation goes here

    figObj2 = figure();
    tileLayout_cust = tiledlayout(figObj2, 1, 3);

    statevar= {"$x$", "$y$", "$z$"};

    for i=1:3
        
        nexttile();
        
        %tit = statevar{i};
        
        h=histfit(data(:,i), 20, 'normal');
        hold on;
        title(statevar{i});

        pdFit = fitdist(data(:,i), 'normal');
        stats(i) = pdFit;

   end
        
end

