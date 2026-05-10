function plotForceMomentDashboard(time, FinB, MinB, opts)
% Plot body-axis forces in the first column and moments in the second column.
arguments
    time
    FinB
    MinB
    opts struct = struct()
end

if ~isfield(opts, 'responseNames')
    % Keep the options struct robust for non-overlay callers.
    opts.responseNames = {};
end

figure('Color', 'w');
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

forceAxes = gobjects(3, 1);
momentAxes = gobjects(3, 1);

for rowIdx = 1:3
    % Create paired force and moment axes in each row.
    forceAxes(rowIdx) = nexttile((rowIdx - 1)*2 + 1);
    momentAxes(rowIdx) = nexttile((rowIdx - 1)*2 + 2);
end

if iscell(FinB) || iscell(MinB)
    % Plot overlayed response histories when callers pass cell arrays.
    plotOverlayedVectorPair(time, FinB, MinB, forceAxes, momentAxes, opts.responseNames);
else
    % Reuse plotVector3 so the non-overlay dashboard matches plotStates.
    plotVector3(time, FinB, "forces", struct("axes", forceAxes));
    plotVector3(time, MinB, "moments", struct("axes", momentAxes));
end

% Link the dashboard time axes so zoom and pan stay aligned.
linkaxes([forceAxes(:); momentAxes(:)], 'x');

end

function plotOverlayedVectorPair(time, FinB, MinB, forceAxes, momentAxes, responseNames)
% Plot overlayed force and moment responses on matching component axes.
componentLabels = {'x', 'y', 'z'};
numResponses = numel(FinB);

if isempty(responseNames)
    % Build default labels when callers do not provide response names.
    responseNames = cell(1, numResponses);
    for responseIdx = 1:numResponses
        responseNames{responseIdx} = sprintf("response %d", responseIdx);
    end
end

for componentIdx = 1:3
    % Draw each body-axis force component with all response traces.
    plotOverlayedComponent(time, FinB, forceAxes(componentIdx), componentIdx, componentLabels, "forces", responseNames);

    % Draw each body-axis moment component with all response traces.
    plotOverlayedComponent(time, MinB, momentAxes(componentIdx), componentIdx, componentLabels, "moments", responseNames);
end

end

function plotOverlayedComponent(time, vectorHistories, targetAxes, componentIdx, componentLabels, plotTitle, responseNames)
% Plot one component from each response history on a shared axis.
axes(targetAxes);
hold(targetAxes, 'on');

for responseIdx = 1:numel(vectorHistories)
    plot(targetAxes, time, vectorHistories{responseIdx}(:, componentIdx), ...
        "DisplayName", responseNames{responseIdx}, ...
        'LineWidth', 1.5);
end

hold(targetAxes, 'off');
grid(targetAxes, 'on');
ylabel(targetAxes, componentLabels{componentIdx});

if componentIdx == 1
    title(targetAxes, plotTitle);
end

if componentIdx == 3
    xlabel(targetAxes, 't');
end

legend(targetAxes, 'Location', 'best');

end
