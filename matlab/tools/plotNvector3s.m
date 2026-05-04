function fig = plotNvector3s(t, yCellArray, labels, plotTitle, opts)
%PLOTNVECTOR3S Overlay N 3-component vector signals on stacked subplots.
%
%   fig = plotNvector3s(t, yCellArray, labels, plotTitle)
%   fig = plotNvector3s(t, yCellArray, labels, plotTitle, opts)
%
% Expects yCellArray to contain N-by-3 arrays with columns x, y, z.

arguments
    t
    yCellArray
    labels
    plotTitle
    opts struct = struct()
end

if ~iscell(yCellArray)
    error('yCellArray must be a cell array.');
end

if numel(labels) ~= numel(yCellArray)
    error('labels must have the same number of elements as yCellArray.');
end

if ~isfield(opts, 'axes')
    % Keep opts robust so callers can pass other option fields later.
    opts.axes = [];
end

for responseIdx = 1:numel(yCellArray)
    y = yCellArray{responseIdx};

    if size(y, 2) ~= 3
        error('Each element of yCellArray must be an N-by-3 array.');
    end

    if numel(t) ~= size(y, 1)
        error('t must have the same number of samples as the rows of each yCellArray element.');
    end
end

componentLabels = {'x', 'y', 'z'};

if isempty(opts.axes)
    % Create a standalone 3-row plot when no axes are supplied.
    fig = figure('Color', 'w');
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    axesArray = gobjects(3, 1);

    for componentIdx = 1:3
        axesArray(componentIdx) = nexttile;
    end
else
    % Reuse caller-provided axes so this helper can draw inside dashboards.
    axesArray = opts.axes(:);
    if numel(axesArray) ~= 3 || ~all(isgraphics(axesArray, 'axes'))
        error('opts.axes must contain 3 axes handles.');
    end
    fig = ancestor(axesArray(1), 'figure');
end

for componentIdx = 1:3
    axes(axesArray(componentIdx));
    hold(axesArray(componentIdx), 'on');
    % Overlay each response on the matching vector component axis.
    for responseIdx = 1:numel(yCellArray)
        y = yCellArray{responseIdx};
        plot(axesArray(componentIdx), t, y(:, componentIdx), ...
            'DisplayName', labels{responseIdx}, ...
            'LineWidth', 1.5);
    end
    hold(axesArray(componentIdx), 'off');
    grid(axesArray(componentIdx), 'on');
    ylabel(axesArray(componentIdx), componentLabels{componentIdx});
    legend(axesArray(componentIdx), 'Location', 'best');
    if componentIdx == 1 && ~isempty(opts.axes)
        title(axesArray(componentIdx), plotTitle);
    end
    if componentIdx == 3
        xlabel(axesArray(componentIdx), 't');
    end
end

if isempty(opts.axes)
    % Preserve the standalone title behavior used by plotVector3.
    sgtitle(plotTitle);
end

end
