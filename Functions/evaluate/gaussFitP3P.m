function [figObj, stats] = gaussFitP3P(data)
%GAUSSFITP3P Summary of this function goes here
%   Detailed explanation goes here

    figObj1 = figure();
    figObj2 = figure();
   
    sizeState = size(data,1);
    
    if sizeState > 7
         numVertTiles = 4;
    else
         numVertTiles = 3;
    end
    tileLayout_cust = tiledlayout(figObj1, numVertTiles, 12);

    if sizeState > 10
        tileLayout_bias =tiledlayout(figObj2, 3, 3);
    end

    statevar= {"$p_x$", "$p_y$", "$p_z$", "$q_w$", "$q_x$", "$q_y$", "$q_z$", "$v_x$", "$v_y$", "$v_z$","$ba_x$", "$ba_y$", "$ba_z$", "$bg_x$", "$bg_y$", "$bg_z$"};

    for i=1:sizeState
        
        if i <= 10
            figure(figObj1);
            %PLOT CHANGING STATE
            if i>=4 && i<=7
                nexttile([1 6]);
            else
                nexttile([1 4]);
               
            end
        else
            figure(figObj2);
            nexttile;
        end
        
        histfit(data(i,:));
        title(statevar{i});

        pdFit = fitdist(data(i,:)', 'Normal');

        stats(i) = pdFit;
    end
        
end

