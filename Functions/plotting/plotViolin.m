function plotViolin(varargin)
    
    % This function generates a violin plot for each dataset provided as input.
    % Violin plots are obtained from histcounts.     
    % The thick line in the box represents the mean, while the thin line represents the median.
    %
    % Inputs:
    %   - Data vectors (each data vector should be followed by its name)
    %   - 'TailBinPct', p (optional, plot-wide): cap equal-width bins at the p-th
    %     percentile and place all larger values in one overflow bin per violin.
    %     Each violin uses its own p-th percentile of its own data; only the
    %     percentile level p is shared across violins on the same figure.
    %   - binSize: fixed width of histogram bins (same units as the data)
    %   - data label: a string for the ylabel of the plot 
    %
    % Example usage:
    %   plotViolin(V1, 'nameV1', V2, 'nameV2', 'TailBinPct', 95, 0.05, 'delay (s)')

    if nargin < 4
        error('You must pass at least 4 arguments: data, names, binSize, and ylabel text.');
    end
    
    binSize = varargin{end-1};  
    data_label = varargin{end};    
    args = varargin(1:end-2);
    
    if ~isnumeric(binSize) || ~isscalar(binSize) || binSize <= 0 || ~ischar(data_label)
        error('binSize must be a positive scalar and ylabel_text must be a char.');
    end

    tailBinPct = [];
    k = 1;
    while k < numel(args)
        if ischar(args{k}) && strcmpi(args{k}, 'TailBinPct')
            if k >= numel(args)
                error('plotViolin:TailBinPct', 'TailBinPct requires a numeric percentile value.');
            end
            tailBinPct = args{k+1};
            args(k:k+1) = [];
        else
            k = k + 1;
        end
    end

    if ~isempty(tailBinPct)
        validateattributes(tailBinPct, {'numeric'}, {'scalar', 'real', '>', 0, '<', 100});
    end

    if mod(numel(args), 2) ~= 0
        error('plotViolin:InvalidArgs', 'Each data vector must be followed by its name.');
    end
    
    Spacing = 2.5;  % Fixed spacing between violins (I do not suggest to change it)
    
    numStructures = numel(args) / 2;
    
    figure;
    hold on;
    
    xticksPos = zeros(1, numStructures);
    xticklabelsPos = cell(1, numStructures);
    
    allLowerWhiskers = [];  
    allUpperWhiskers = [];  
    
    for i = 1:numStructures

        data = args{2*i - 1};
        name = args{2*i};
        
        idx_validData = ~isnan(data);
        d = data(idx_validData);

        edges = buildViolinEdges(d, binSize, tailBinPct);
        [counts, edges] = histcounts(d, edges, 'Normalization', 'pdf');
        centers = edges(1:end-1) + diff(edges) / 2;
        maxCount = max(counts); 
        counts = counts / maxCount;
              
        pos = (i-1) * Spacing;
        fill([pos + counts, pos - flip(counts)], [centers, flip(centers)], [0 0 0.7451], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        
        meanVal = mean(d);
        medianVal = median(d);
        
        Q1 = prctile(d, 25);
        Q3 = prctile(d, 75);
        IQR = Q3 - Q1;
        
        upperWhiskerLimit = Q3 + 1.5 * IQR;
        lowerWhiskerLimit = Q1 - 1.5 * IQR;
        upperWhisker = max(data(data <= upperWhiskerLimit));
        lowerWhisker = min(data(data >= lowerWhiskerLimit));
        allLowerWhiskers = [allLowerWhiskers, lowerWhisker];
        allUpperWhiskers = [allUpperWhiskers, upperWhisker];
        
        box_width = 0.35 * max(counts);
        rectangle('Position', [pos - box_width, Q1, 2*box_width, Q3 - Q1], ...
                  'EdgeColor', [0 0 0.7451], 'LineWidth', 1.5);
                  
        plot([pos - box_width, pos + box_width], [meanVal, meanVal], 'LineWidth', 3, ...
             'Color', [0 0 0.7451], 'LineStyle','-');
        
        plot([pos - box_width, pos + box_width], [medianVal, medianVal], 'LineWidth', 1.5, ...
             'Color', [0 0 0.7451], 'LineStyle', '-');
        
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

function edges = buildViolinEdges(d, binSize, tailBinPct)
%BUILDVIOLINEDGES  Fixed-width bins from min(d) to cap, optional single tail bin.

    lo = min(d);
    hi = max(d);

    if isempty(tailBinPct)
        cap = hi;
    else
        cap = prctile(d, tailBinPct);
    end

    if hi <= lo
        edges = [lo, hi];
        return;
    end

    edges = lo;
    while edges(end) + binSize < cap
        edges(end+1) = edges(end) + binSize; %#ok<AGROW>
    end
    if edges(end) < cap
        edges(end+1) = cap;
    end

    if ~isempty(tailBinPct) && hi > cap
        if edges(end) < hi
            edges(end+1) = hi;
        end
    elseif edges(end) < hi
        edges(end+1) = hi;
    end
end
