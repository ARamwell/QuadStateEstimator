function figHandle = formatFigForLatex_v2(figHandle, picturewidth, hw_ratio)
%FORMATFIGFORLATEX Format a figure nicely for LaTeX export.
%
%   figHandle = FORMATFIGFORLATEX(figHandle)
%   figHandle = FORMATFIGFORLATEX(figHandle, picturewidth_cm, hw_ratio)
%
%   - Works with normal axes and tiled layouts.
%   - Sets LaTeX interpreters.
%   - Ensures legends have a readable white background.

    if nargin < 1 || ~ishandle(figHandle) || ~strcmp(get(figHandle,'Type'),'figure')
        error('First argument must be a valid figure handle.');
    end
    if nargin < 2 || isempty(picturewidth)
        picturewidth = 40;      % [cm]
    end
    if nargin < 3 || isempty(hw_ratio)
        hw_ratio = 0.65;        % height/width
    end

    % --- Global figure settings ---
    set(figHandle, 'Units','centimeters', ...
                   'Position',[3 3 picturewidth hw_ratio*picturewidth]);

    % Default interpreters (applied to new text/axes created under this figure)
    set(figHandle, 'DefaultTextInterpreter','latex', ...
                   'DefaultAxesTickLabelInterpreter','latex', ...
                   'DefaultLegendInterpreter','latex');

    % --- Collect graphics objects of interest ---
    % This finds axes inside normal plots AND tiled layouts
    ax = findall(figHandle, 'Type','axes');

    % Exclude colorbars if desired (uncomment if you want different settings)
    % ax = ax(~strcmp(get(ax,'Tag'),'Colorbar'));

    % Lines and markers
    ln = findall(figHandle, 'Type','line');
    % Text objects (titles, labels, etc. are usually handled via axes properties,
    % but we can still normalize here)
    tx = findall(figHandle, 'Type','text');
    % Legends
    lg = findall(figHandle, 'Type','legend');

    % --- Line width & fonts ---
    desiredLineWidth = 1.5;
    desiredFontSize  = 12;

    % Lines
    if ~isempty(ln)
        set(ln, 'LineWidth', desiredLineWidth);
    end

    % Axes (including those in tiled layouts)
    if ~isempty(ax)
        set(ax, 'LineWidth', desiredLineWidth, ...
                'FontSize', desiredFontSize, ...
                'Box','off');  % or 'on' if you prefer boxed axes
    end

    % Text
    if ~isempty(tx)
        set(tx, 'FontSize', desiredFontSize);
    end

    % --- Legends: make readable with white background ---
    for k = 1:numel(lg)
        set(lg(k), 'Interpreter','latex', ...
                   'FontSize', desiredFontSize, ...
                   'Box','on', ...      % draw legend border
                   'Color','w');        % solid white background
        % If you prefer a semi-transparent background, you can instead use
        % set(lg(k), 'Color','w', 'EdgeColor',[0 0 0], 'NumColumns',get(lg(k),'NumColumns'));
    end

    % If you want to auto-adjust paper size for printing/export, you can add:
    % pos = get(figHandle,'Position');
    % set(figHandle, 'PaperUnits','centimeters', ...
    %                'PaperPosition',[0 0 pos(3) pos(4)], ...
    %                'PaperSize',[pos(3) pos(4)]);

end