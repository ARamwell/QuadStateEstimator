function plotViolin(varargin)
    
    % This function generates a violin plot for each dataset provided as input.
    % Violin plots are obtained from histcounts.     
    % The thick line in the box represents the mean, while the thin line represents the median.
    %
    % Inputs:
    %   - Data vectors (each data vector should be followed by its name)
    %   - numbins: number of bins for the histogram
    %   - data label: a string for the ylabel of the plot 
    %
    % Example usage:
    %   plotViolin(V1, 'nameV1', V2, 'nameV2', ..., 150, 'delay')
    %   Where V1, V2, ... are vectors of data, 'nameV1', 'nameV2', ... are their names,
    %   150 is the number of bins, and 'delay' is the data label for the plot

    if nargin < 4
        error('You must pass at least 4 arguments: data, names, numbins, and ylabel text.');
    end
    
    numArgs = length(varargin); 
    numbins = varargin{numArgs-1};  
    data_label = varargin{numArgs};    
    
    if ~isnumeric(numbins) || ~ischar(data_label)
        error('numbins must be numeric and ylabel_text must be a char.');
    end
    
    Spacing = 2.5;  % Fixed spacing between violins (I do not suggest to change it)
    
    numStructures = (numArgs - 2) / 2;  % Each structure consists of 2 arguments (data + name)
    
    figure;
    hold on;
    
    xticksPos = zeros(1, numStructures);  % X tick positions
    xticklabelsPos = cell(1, numStructures);  % X tick labels
    
    allLowerWhiskers = [];  
    allUpperWhiskers = [];  
    
    for i = 1:numStructures

        data = varargin{2*i - 1};   % Data vector
        name = varargin{2*i};       % Name for the X-axis label
        
        % Compute distribution
        [counts, edges] = histcounts(data, numbins, 'Normalization', 'pdf'); % Normalized count as PDF
        centers = edges(1:end-1) + diff(edges) / 2; % Bin centers
        maxCount = max(counts); 
        counts = counts / maxCount; % Normalize to 1
              
        % violin plot
        pos = (i-1) * Spacing;
        fill([pos + counts, pos - flip(counts)], [centers, flip(centers)], [0 0 0.7451], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        
        % Mean and median
        meanVal = mean(data);
        medianVal = median(data);
        
        % Interquartile Range
        Q1 = prctile(data, 25);  % First quartile
        Q3 = prctile(data, 75);  % Third quartile
        IQR = Q3 - Q1;
        
        % Whisker boundaries
        upperWhiskerLimit = Q3 + 1.5 * IQR;
        lowerWhiskerLimit = Q1 - 1.5 * IQR;
        upperWhisker = max(data(data <= upperWhiskerLimit));
        lowerWhisker = min(data(data >= lowerWhiskerLimit));
        allLowerWhiskers = [allLowerWhiskers, lowerWhisker];
        allUpperWhiskers = [allUpperWhiskers, upperWhisker];
        
        box_width = 0.15 * max(counts);  % Narrow box width relative to violin
        rectangle('Position', [pos - box_width, Q1, 2*box_width, Q3 - Q1], ...
                  'EdgeColor', [0 0 0.7451], 'LineWidth', 1.5); % Box for IQR
                  
        plot([pos - box_width, pos + box_width], [meanVal, meanVal], 'LineWidth', 3, ...
             'Color', [0 0 0.7451], 'LineStyle','-');
        
        plot([pos - box_width, pos + box_width], [medianVal, medianVal], 'LineWidth', 1.5, ...
             'Color', [0 0 0.7451], 'LineStyle', '-');
        
        % Whiskers following the 1.5*IQR rule
        plot([pos, pos], [lowerWhisker, Q1], 'Color', [0 0 0.7451], 'LineWidth', 1.2,'LineStyle','-');  
        plot([pos, pos], [Q3, upperWhisker], 'Color', [0 0 0.7451], 'LineWidth', 1.2,'LineStyle','-');  
        
        xticksPos(i) = pos;
        xticklabelsPos{i} = name;
    end
    
    minLowerWhisker = min(allLowerWhiskers);
    maxUpperWhisker = max(allUpperWhiskers);
    ylim([minLowerWhisker-0.1*abs(maxUpperWhisker-minLowerWhisker), maxUpperWhisker+0.1*abs(maxUpperWhisker-minLowerWhisker)]);

    xlim([0 - Spacing / 2, (numStructures - 1) * Spacing + Spacing / 2]); 
    xticks(xticksPos); 
    xticklabels(xticklabelsPos); 
    ylabel(data_label, "FontSize", 13);  
    hold off;
    grid on;
    box on;
    view([90 90]);

end

