function plot_updated = updatePlot(plotIn, data)

    %Define a list of valid method names
    validPlots = {'trajectory', 'verification'}; %if you add to this, you must also add a switch case
    
    %Initialise output struct
    plot_updated = plotIn;

    %% MAIN
  
    %Get name of plot type
    plotType = plotIn.Type;

    if any(strcmp(plotType, validPlots))

        %Apply suitable update method
        switch plotType
            case 'trajectory'
                %'data' should be a struct of new Rt values, indexed by
                % p3p methodName
                linesToUpdate = fieldnames(plotIn.plotLines);
               

                %for each method
                for l = 1:length(linesToUpdate)
                    currentLine = linesToUpdate{l};
                    
                    if any(strcmp(currentLine, 'Verification'))
                        continue;
                    end
                    if any(strcmp(currentLine, 'EKF'))
                        continue;
                    end
                    
                    %update trajectory data
                    %oldDataSeries = plotIn.plotLines.(currentLine).Data;
                    %newDataPnt = data.(currentLine).Rt;
                    %newDataSeries = cat(3, oldDataSeries, newDataPnt);
                    newDataSeries = data.(currentLine).Rt;
                    plotIn.plotLines.(currentLine).Data = newDataSeries;

                    p3pPlotting.updateTraj(plotIn.plotLines.(currentLine).line, plotIn.plotLines.(currentLine).frameText, plotIn.plotLines.(currentLine).frameLines, newDataSeries);

                end
                               
        end

    end

plot_updated = plotIn;

end