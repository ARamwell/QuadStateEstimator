function fig_stateEvol = plotStateEvolution(ekfResult, trueState, trueState_times, p3pResult)
    
    numLoops = size(ekfResult.x_, 2);
    numEl = size(ekfResult.x_, 1);
    
    fig_stateEvol = figure();
    if numEl == 10
        numVertTiles = 3;
    elseif numEl == 16
        numVertTiles = 5;
    end
    tileLayout_cust = tiledlayout(fig_stateEvol, numVertTiles, 12);
    
    for i=1:numEl
        
    
        %PLOT CHANGING STATE
        if i>=4 && i<=7
            nexttile([1 3]);
        else
            nexttile([1 4]);
        end
        x = ekfResult.elapsedTime(1,:);
        y = ekfResult.x_(i,:);
        plot(x, y, Color='#d7191c',  DisplayName='estimated state variable');  
        hold on;
        
        %PLOT VARIANCE
        stateVariance = zeros(1, numLoops); %initialise array
        for t=1:numLoops
            stateVariance(1,t)=(ekfResult.P(i,i,t));
        end
        stateVariance = sqrt(stateVariance);
        lowerCurve = y - 1*abs(stateVariance);
        upperCurve = y + 1*abs(stateVariance);
        plot(x, lowerCurve, Color='#fdae61',  HandleVisibility='off');
        hold on;
        plot(x, upperCurve, Color='#fdae61', DisplayName='2-std dev bounds');
        hold on;
        % fill(x, [y; upperCurve],[.9 .9 .9],'linestyle','none');
        % hold on;
        % 
        % fill(x, [lowerCurve; y],[.9 .9 .9],'linestyle','none');
        % hold on;
        %line(x,y)
        %errorbar(x, y, -dy, dy);
        % 
        % drawnow;
        % hold on;
    
    
        %%PLOT GROUND TRUTH
        if i <= 7
            if exist("p3pResult", "var")
                x = p3pResult.time(1,:);
                y = p3pResult.selected(i, :);
                plot(x, y, Color='#3288bd',  DisplayName='visual state estimate');  
                hold on;
            end

        end

        if i<= 10
            if exist("trueState", 'var')
                %figure()
                x = trueState_times(1,:);
                y = trueState(i,:);
                plot(x, y, Color='#1a9641',  DisplayName='true state variable');
                hold on;
            end
        end

    
        %plot corrections, if any (i.e., measurements in state space)
        % if i<=7
        %     %plot(x(isfinite(y)),y(isfinite(y)),'*-')
        %     x = ekfResult.elapsedTime(1,:);
        %     y = ekfResult.z(i,:);
        %     scatter(x(isfinite(y)),y(isfinite(y)),10, "filled");
        %     hold on;
        %     plot(x,y);
        %     hold on;
        % 
        %     % %mark estimate that was corrected
        %     % correctedEst = zeros(1, size(ekfResult.zHist, 2));
        %     % for t=1:size(ekfResult.zHist, 2)
        %     %     t_k =  ekfResult.zHist(1, t);
        %     %     [closestDiff, closestIndex] = min(abs(ekfResult.elapsedTime(1,:)-ekfResult.zHist(1,t)));
        %     %     correctedEst(1, t) = ekfResult.stateEst(i, closestIndex);
        %     % end
        %     % y = correctedEst;
        %     % quiver(x, correctedEst, zeros(1, size(correctedEst, 2)), ((ekfResult.zHist(1+i,:)-correctedEst)), 0);
        % 
        % end
        
        title(strcat('State variable evolution: ', string(i)));
        %legend('State estimate', 'Std deviation lower bound', 'Std deviation upper bound', 'Ground truth', 'Camera estimate (correction)')
        legend;
        hold off;
    
       end
end