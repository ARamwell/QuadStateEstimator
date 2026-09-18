function [figObj, stats] = gaussFitP3P(data)
%GAUSSFITP3P Summary of this function goes here
%   Detailed explanation goes here

    figObj1 = figure();
    figObj2 = figure();
   
    sizeState = size(data,1);
    
    if sizeState > 7
        numVertTiles = 3;
    else
        numVertTiles = 2;
    end
    tileLayout_cust = tiledlayout(figObj1, numVertTiles, 12);

    if sizeState > 10
        tileLayout_bias =tiledlayout(figObj2, 3, 3);
    end

    statevar= {"$p_x$", "$p_y$", "$p_z$", "$q_w$", "$q_x$", "$q_y$", "$q_z$", "$v_x$", "$v_y$", "$v_z$","$ba_x$", "$ba_y$", "$ba_z$", "$bg_x$", "$bg_y$", "$bg_z$"};

    for i=1:sizeState
        
        if i <= 10
            figure(figObj1);
        
            if i <= 3 || (i >= 8 && i <= 10)
                % p_x--p_z and v_x--v_z: three plots across.
                nexttile([1, 4]);
        
            elseif i >= 4 && i <= 7
                % q_w--q_z: four plots across on one line.
                nexttile([1, 3]);
            end
        else
            figure(figObj2);
            nexttile;
        end
        
        numBins = reasonableBins(data(i,:));
        histfit(data(i,:),numBins);
        title(statevar{i});

        pdFit = fitdist(data(i,:)', 'Normal');

        stats(i) = pdFit;
    end

    function numBins = reasonableBins(x)
% Calculate a reasonable histogram-bin count for one state vector.

    x = abs(x(:));
    x = x(isfinite(x) & x ~= 0);

    n = numel(x);

    if n < 2
        numBins = 1;
        return
    end

    binWidth = 2 * iqr(x) / n^(1/3);

    if binWidth <= 0 || ~isfinite(binWidth)
        numBins = 20;
    else
        numBins = ceil((max(x) - min(x)) / binWidth);
    end

    numBins = max(20, min(numBins, 60));
    end

  function restyleExistingHistograms(figObj, data, stats)
%RESTYLEEXISTINGHISTOGRAMS Replot the first ten state-error histograms.
% Uses the NormalDistribution objects in stats from gaussFitP3P's main loop.

    useAbsolute = false;   % Keep false to match stats(i).mu and stats(i).sigma.
    maxStates = min([10, size(data, 1), numel(stats)]);

    signedLabels = { ...
        '$\varepsilon_{p_x}~(\mathrm{m})$', ...
        '$\varepsilon_{p_y}~(\mathrm{m})$', ...
        '$\varepsilon_{p_z}~(\mathrm{m})$', ...
        '$\varepsilon_{q_w}~(-)$', ...
        '$\varepsilon_{q_x}~(-)$', ...
        '$\varepsilon_{q_y}~(-)$', ...
        '$\varepsilon_{q_z}~(-)$', ...
        '$\varepsilon_{v_x}~(\mathrm{m\,s^{-1}})$', ...
        '$\varepsilon_{v_y}~(\mathrm{m\,s^{-1}})$', ...
        '$\varepsilon_{v_z}~(\mathrm{m\,s^{-1}})$'};

    absoluteLabels = { ...
        '$|\varepsilon_{p_x}|~(\mathrm{m})$', ...
        '$|\varepsilon_{p_y}|~(\mathrm{m})$', ...
        '$|\varepsilon_{p_z}|~(\mathrm{m})$', ...
        '$|\varepsilon_{q_w}|~(-)$', ...
        '$|\varepsilon_{q_x}|~(-)$', ...
        '$|\varepsilon_{q_y}|~(-)$', ...
        '$|\varepsilon_{q_z}|~(-)$', ...
        '$|\varepsilon_{v_x}|~(\mathrm{m\,s^{-1}})$', ...
        '$|\varepsilon_{v_y}|~(\mathrm{m\,s^{-1}})$', ...
        '$|\varepsilon_{v_z}|~(\mathrm{m\,s^{-1}})$'};

    if useAbsolute
        xAxisLabels = absoluteLabels;
    else
        xAxisLabels = signedLabels;
    end

    clf(figObj);

    set(figObj, ...
        'Color', 'w', ...
        'Units', 'pixels', ...
        'Position', [100, 100, 1450, 700]);

    tileLayout = tiledlayout(figObj, 3, 12, ...
        'Padding', 'compact', ...
        'TileSpacing', 'compact');

    for i = 1:maxStates

        % Position row: 3 plots. Quaternion row: 4 plots.
        % Velocity row: 3 plots.
        if i <= 3 || i >= 8
            nexttile(tileLayout, [1, 4]);
        else
            nexttile(tileLayout, [1, 3]);
        end

        % Zero, NaN, and Inf values represent missing data.
        stateData = data(i, :);
        stateData = stateData(isfinite(stateData) & stateData ~= 0);

        if useAbsolute
            stateData = abs(stateData);
            displayLimits = [0, prctile(stateData, 99.5)];
        else
            % Keep isolated outliers from setting an unhelpful x-axis range.
            displayLimits = prctile(stateData, [0.25, 99.75]);
        end

        displayData = stateData( ...
            stateData >= displayLimits(1) & ...
            stateData <= displayLimits(2));

        % Freedman--Diaconis bin count, constrained for readability.
        binWidth = 2 * iqr(displayData) / numel(displayData)^(1/3);

        if binWidth <= 0 || ~isfinite(binWidth)
            numBins = 30;
        else
            numBins = ceil(range(displayData) / binWidth);
            numBins = max(20, min(numBins, 60));
        end

        h = histfit(displayData, numBins);

        h(1).FaceColor = [0, 0.4470, 0.7410];
        h(1).EdgeColor = 'k';
        h(1).LineWidth = 0.25;

        h(2).Color = [0.8500, 0.3250, 0.0980];
        h(2).LineWidth = 1.5;

        ax = gca;
        xlim(displayLimits);
        grid on;
        box off;

        ax.FontName = 'Times New Roman';
        ax.FontSize = 10;
        ax.GridAlpha = 0.18;
        ax.MinorGridAlpha = 0.10;
        ax.XMinorGrid = 'on';
        ax.YMinorGrid = 'on';

        % No plot titles: identify each state through its x-axis label.
        xlabel(xAxisLabels{i}, ...
            'Interpreter', 'latex', ...
            'FontSize', 11);

        % One frequency label per row.
        if i == 1 || i == 4 || i == 8
            ylabel('Frequency');
        end

        % Use the original fitted values from gaussFitP3P's main loop.
        meanValue = stats(i).mu;
        stdValue = stats(i).sigma;

        % Position, velocity, accel bias: 3 decimals.
        % Quaternion components and gyro bias: 4 decimals.
        useFourDecimals = (i >= 4 && i <= 7) || (i >= 14 && i <= 16);

        if useFourDecimals
            annotationText = sprintf( ...
                '\\mu = %.4f\\newline\\sigma = %.4f', ...
                meanValue, stdValue);
        else
            annotationText = sprintf( ...
                '\\mu = %.3f\\newline\\sigma = %.3f', ...
                meanValue, stdValue);
        end

        text(0.06, 0.92, annotationText, ...
            'Units', 'normalized', ...
            'VerticalAlignment', 'top', ...
            'FontName', 'Times New Roman', ...
            'FontSize', 10);
    end
  end

function restyleBiasHistograms(figObj2, data, stats)
%RESTYLEBIASHISTOGRAMS Replot accelerometer- and gyro-bias histograms.
% Uses the fitted NormalDistribution objects from gaussFitP3P's main loop.

    useAbsolute = false;   % Keep false to match stats(i).mu and stats(i).sigma.
    stateIndices = 11:min(16, min(size(data, 1), numel(stats)));

    signedLabels = { ...
        '$\varepsilon_{b_{a,x}}~(\mathrm{m\,s^{-2}})$', ...
        '$\varepsilon_{b_{a,y}}~(\mathrm{m\,s^{-2}})$', ...
        '$\varepsilon_{b_{a,z}}~(\mathrm{m\,s^{-2}})$', ...
        '$\varepsilon_{b_{g,x}}~(\mathrm{rad\,s^{-1}})$', ...
        '$\varepsilon_{b_{g,y}}~(\mathrm{rad\,s^{-1}})$', ...
        '$\varepsilon_{b_{g,z}}~(\mathrm{rad\,s^{-1}})$'};

    absoluteLabels = { ...
        '$|\varepsilon_{b_{a,x}}|~(\mathrm{m\,s^{-2}})$', ...
        '$|\varepsilon_{b_{a,y}}|~(\mathrm{m\,s^{-2}})$', ...
        '$|\varepsilon_{b_{a,z}}|~(\mathrm{m\,s^{-2}})$', ...
        '$|\varepsilon_{b_{g,x}}|~(\mathrm{rad\,s^{-1}})$', ...
        '$|\varepsilon_{b_{g,y}}|~(\mathrm{rad\,s^{-1}})$', ...
        '$|\varepsilon_{b_{g,z}}|~(\mathrm{rad\,s^{-1}})$'};

    if useAbsolute
        xAxisLabels = absoluteLabels;
    else
        xAxisLabels = signedLabels;
    end

    clf(figObj2);

    set(figObj2, ...
        'Color', 'w', ...
        'Units', 'pixels', ...
        'Position', [100, 100, 1250, 520]);

    tileLayout = tiledlayout(figObj2, 2, 3, ...
        'Padding', 'compact', ...
        'TileSpacing', 'compact');

    for component = 1:numel(stateIndices)

        stateIndex = stateIndices(component);
        nexttile(tileLayout);

        stateData = data(stateIndex, :);
        stateData = stateData(isfinite(stateData) & stateData ~= 0);

        if useAbsolute
            stateData = abs(stateData);
            displayLimits = [0, prctile(stateData, 99.5)];
        else
            displayLimits = prctile(stateData, [0.25, 99.75]);
        end

        displayData = stateData( ...
            stateData >= displayLimits(1) & ...
            stateData <= displayLimits(2));

        binWidth = 2 * iqr(displayData) / numel(displayData)^(1/3);

        if binWidth <= 0 || ~isfinite(binWidth)
            numBins = 30;
        else
            numBins = ceil(range(displayData) / binWidth);
            numBins = max(20, min(numBins, 60));
        end

        h = histfit(displayData, numBins);

        h(1).FaceColor = [0, 0.4470, 0.7410];
        h(1).EdgeColor = 'k';
        h(1).LineWidth = 0.25;

        h(2).Color = [0.8500, 0.3250, 0.0980];
        h(2).LineWidth = 1.5;

        ax = gca;
        xlim(displayLimits);
        grid on;
        box off;

        ax.FontName = 'Times New Roman';
        ax.FontSize = 10;
        ax.GridAlpha = 0.18;
        ax.MinorGridAlpha = 0.10;
        ax.XMinorGrid = 'on';
        ax.YMinorGrid = 'on';

        xlabel(xAxisLabels{component}, ...
            'Interpreter', 'latex', ...
            'FontSize', 11);

        if component == 1 || component == 4
            ylabel('Frequency');
        end

        meanValue = stats(stateIndex).mu;
        stdValue = stats(stateIndex).sigma;

        % Accelerometer bias: 3 decimals. Gyro bias: 4 decimals.
        if stateIndex <= 13
            annotationText = sprintf( ...
                '\\mu = %.3f\\newline\\sigma = %.3f', ...
                meanValue, stdValue);
        else
            annotationText = sprintf( ...
                '\\mu = %.4f\\newline\\sigma = %.4f', ...
                meanValue, stdValue);
        end

        text(0.06, 0.92, annotationText, ...
            'Units', 'normalized', ...
            'VerticalAlignment', 'top', ...
            'FontName', 'Times New Roman', ...
            'FontSize', 10);
    end
end
end

