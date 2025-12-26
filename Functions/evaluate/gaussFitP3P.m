function [figObj, stats] = gaussFitP3P(data)
%GAUSSFITP3P Summary of this function goes here
%   Detailed explanation goes here

    figObj2 = figure();
    numVertTiles = 3;
    tileLayout_cust = tiledlayout(figObj2, 3, 12);

    statevar= {"$p_x$", "$p_y$", "$p_z$", "$q_w$", "$q_x$", "$q_y$", "$q_z$"};

    %q to angle
    

    for i=1:7
            
        %PLOT CHANGING STATE
        if i>=4 && i<=7
            nexttile([1 6]);
            
        else
            nexttile([1 4]);
            
        end
        
        histfit(data(i,:));
        title(statevar{i});

        pdFit = fitdist(data(i,:)', 'Normal');

        stats(i) = pdFit;


    end
        
end

