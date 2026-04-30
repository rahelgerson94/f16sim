function []= plotStates(time, uvw, pqr, forces, moments)
close all;

c = getConstants();
figure('Color', 'w');

% Build each vector group as three stacked axes inside a 6-by-6 dashboard.
numVectorRows = 3;
numGridCols = 6;
numGroupCols = 3;
forceAxes = createVectorAxes(1, 1, numVectorRows, numGridCols, numGroupCols);
momentAxes = createVectorAxes(1, 4, numVectorRows, numGridCols, numGroupCols);
uvwAxes = createVectorAxes(4, 1, numVectorRows, numGridCols, numGroupCols);
pqrAxes = createVectorAxes(4, 4, numVectorRows, numGridCols, numGroupCols);

plotVector3(time, forces, "forces", struct("axes", forceAxes));
plotVector3(time, moments, "moments", struct("axes", momentAxes));
plotVector3(time, uvw*c.M2FT, "uvw (ft/s)", struct("axes", uvwAxes));
plotVector3(time, pqr*c.RAD2DEG, "w (deg/s)", struct("axes", pqrAxes));
linkDashboardXAxes(forceAxes, momentAxes, uvwAxes, pqrAxes);

% Override subplot's wide default margins with a compact dashboard layout.
layoutOpts = struct( ...
    "leftMargin", 0.055, ...
    "rightMargin", 0.025, ...
    "topMargin", 0.055, ...
    "bottomMargin", 0.055, ...
    "colGap", 0.035, ...
    "rowGap", 0.030, ...
    "middleRowGap", 0.110);
applyTightDashboardLayout(forceAxes, momentAxes, uvwAxes, pqrAxes, layoutOpts);

   V = vecnorm(uvw, 2, 2);
    alpha = atan2(uvw(:,3), uvw(:,1));
    beta = asin(uvw(:,2) ./ V);
    plot3Scalars(time, alpha, beta , V*c.M2FT , ["α  (deg)", "β  (deg)" , "V (ft/s)"], "")
end

function linkDashboardXAxes(forceAxes, momentAxes, uvwAxes, pqrAxes)
% Link the dashboard time axes so zooming one plot keeps all plots aligned.
allAxes = [forceAxes(:); momentAxes(:); uvwAxes(:); pqrAxes(:)];
linkaxes(allAxes, 'x');

end

function applyTightDashboardLayout(forceAxes, momentAxes, uvwAxes, pqrAxes, opts)
% Place the four vector groups in a tight 6-row by 2-column dashboard.
axesGrid = [forceAxes(:), momentAxes(:); uvwAxes(:), pqrAxes(:)];
numRows = size(axesGrid, 1);
numCols = size(axesGrid, 2);
rowGaps = opts.rowGap*ones(numRows - 1, 1);
middleGapIndex = numel(forceAxes);
rowGaps(middleGapIndex) = opts.middleRowGap;

axesWidth = (1 - opts.leftMargin - opts.rightMargin - (numCols - 1)*opts.colGap)/numCols;
axesHeight = (1 - opts.topMargin - opts.bottomMargin - sum(rowGaps))/numRows;

for row = 1:numRows
    for col = 1:numCols
        left = opts.leftMargin + (col - 1)*(axesWidth + opts.colGap);
        % Use a larger center gap between the top and bottom vector groups.
        bottom = 1 - opts.topMargin - row*axesHeight - sum(rowGaps(1:(row - 1)));
        set(axesGrid(row, col), "Units", "normalized", "Position", [left, bottom, axesWidth, axesHeight]);
    end
end

end

function axesArray = createVectorAxes(startRow, startCol, numVectorRows, numGridCols, numGroupCols)
% Create a three-row subplot stack that spans three columns per component.
axesArray = gobjects(numVectorRows, 1);

for i = 1:numVectorRows
    row = startRow + i - 1;
    firstIndex = (row - 1)*numGridCols + startCol;
    subplotIndices = firstIndex:(firstIndex + numGroupCols - 1);
    axesArray(i) = subplot(numGridCols, numGridCols, subplotIndices);
end

end
