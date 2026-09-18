function plotMeanAndStd(figObj, meanVal, stdVal, colMean, colStd)


    lowerLine = meanVal - stdVal;
    upperLine = meanVal + stdVal;

    if isempty(colStd)
        colStd = '#fdae61';
    end

    figure(figObj);
    hold on;
    yline(meanVal, Color=colMean, DisplayName='mean', LineWidth=2);
    hold on;
    yline(lowerLine, Color=colStd, HandleVisibility='off', LineWidth=1.5);
    hold on;
    yline(upperLine, Color=colStd, DisplayName='1-std dev bounds', LineWidth=1.5)


    xlims = xlim;
    xLabelPosLeft = xlims(2) * 0.05;   % near the left edge
    xLabelPosRight = xlims(2) * 0.5;   % near the left edge
    %text(xLabelPos, meanVal, 'mean', Color=colMean, VerticalAlignment='bottom');
    %text(xLabelPos, meanVal+stdVal, '+1 standard deviation', Color=colStd, VerticalAlignment='bottom');
    %text(xLabelPos, meanVal-stdVal, '-1 standard deviation', Color=colStd, VerticalAlignment='top');

    text(xLabelPosLeft, meanVal, ...
        sprintf('mean = %.4f', meanVal), ...
        'Color', colMean, 'VerticalAlignment', 'bottom', 'Interpreter','latex');

    text(xLabelPosRight, upperLine, ...
        sprintf('+1 s.d. = %.4f', upperLine), ...
        'Color', colStd, 'VerticalAlignment', 'bottom', 'Interpreter','latex');

    text(xLabelPosRight, lowerLine, ...
        sprintf('-1 s.d. = %.4f', lowerLine), ...
        'Color', colStd, 'VerticalAlignment', 'top', 'Interpreter','latex');

    hold off;

    % plot(x, lowerLine, Color=col,  HandleVisibility='off');
    % hold on;
    % plot(x, upperCurve, Color=col, DisplayName='1-std dev bounds');
    % hold on;

end