function [figObj2, stats] = gaussFitIMU(data)
%GAUSSFITP3P Summary of this function goes here
%   Detailed explanation goes here

    figObj2 = figure();
    tileLayout_cust = tiledlayout(figObj2, 1, 3);

    statevar= {"$x$-channel", "$y$-channel", "$z$-channel"};

    for i=1:3        
        nexttile();        
        %tit = statevar{i};       
        h=histfit(data(:,i), 20, 'normal');
        hold on;
        title(statevar{i});

        pdFit = fitdist(data(:,i), 'normal')
        stats(i) = pdFit;

        %insert mean and sd textbox
        % str = sprintf('$\mu = %.4f$', pdFit.mu);
        % str = str + newline;
        % str = str + sprintf('$\sigma = %.4f', pdFit.sigma);
        % dim = [.05 .05 .3 .3];
        % annotation('textbox', dim,'String',str,'FitBoxToText','on','VerticalAlignment','bottom');

        formatFigForLatex(figObj2);
   end
        
end

