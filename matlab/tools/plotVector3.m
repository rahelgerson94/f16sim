function fig = plotVector3(t, y, plotTitle, opts)
%PLOTVECTOR3 Plot a 3-component vector signal on stacked subplots.
%
%   fig = plotVector3(t, y, plotTitle)
%   fig = plotVector3(t, y, plotTitle, opts)
%
% Expects y to be an N-by-3 array with columns corresponding to x, y, z.

arguments
    t
    y
    plotTitle
    opts struct = struct()
end

if ~isfield(opts, 'axes')
    % Keep opts robust so callers can pass other option fields later.
    opts.axes = [];
end

if size(y, 2) ~= 3
    error('y must be an N-by-3 array.');
end

if numel(t) ~= size(y, 1)
    error('t must have the same number of samples as the rows of y.');
end

labels = {'x', 'y', 'z'};

if isempty(opts.axes)
    % Create the original standalone 3-row plot when no axes are supplied.
    fig = figure('Color', 'w');
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    axesArray = gobjects(3, 1);

    for i = 1:3
        axesArray(i) = nexttile;
    end
else
    % Reuse caller-provided axes so this helper can draw inside dashboards.
    axesArray = opts.axes(:);
    if numel(axesArray) ~= 3 || ~all(isgraphics(axesArray, 'axes'))
        error('opts.axes must contain 3 axes handles.');
    end
    fig = ancestor(axesArray(1), 'figure');
end

for i = 1:3
    axes(axesArray(i));
    plot(axesArray(i), t, y(:, i), 'LineWidth', 1.5);
    grid(axesArray(i), 'on');
    ylabel(axesArray(i), labels{i});
    if i == 1 && ~isempty(opts.axes)
        title(axesArray(i), plotTitle);
    end
    if i == 3
        xlabel(axesArray(i), 't');
        grid(axesArray(i), 'on');
        ylabel(axesArray(i), labels{i});
    end
end

if isempty(opts.axes)
    % Preserve the original standalone title behavior.
    sgtitle(plotTitle);
end

end
