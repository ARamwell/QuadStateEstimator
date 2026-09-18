function [axObj] = plotNEES(figObj, x, nees, stateSize, numMonteCarloRuns, title)
    
    figure(figObj);
    %axObj = axes('Parent', figObj);
    alpha = 0.05; %confidence
    chiSquareLimits = [chi2inv(alpha/2, numMonteCarloRuns*stateSize), chi2inv(1-alpha/2, numMonteCarloRuns*stateSize)]/numMonteCarloRuns;
    
    axObj = plot(x, nees, 'DisplayName', title);
    xlabel('Elapsed time (s)');
    ylabel('NEES');
    ylim([0, round(chiSquareLimits(2) * 1.5)])
    
    chiLine_lower = yline(chiSquareLimits(1), '--g', 'chi-squared lower',  HandleVisibility='off');
    chiLine_upper = yline(chiSquareLimits(2), '--g', 'chi-squared upper',  HandleVisibility='off');

  

end