function []= plotStatesLon(time, U,W,Q,THETA, N,D)
close all;

c = getConstants();
figure('Color', 'w');


tiledlayout(3,2);
toPlot = {U,W,Q,THETA, N,D};
labels = {"u", "w", "q", "theta", "n", "d"};
units = {c.M2FT, c.M2FT, c.RAD2DEG, c.RAD2DEG, c.M2FT, c.M2FT};
unitNames = {"ft/s", "ft/s", "deg/s", "deg",  "ft", "ft"};
plotIndex = 1;
axesArray = gobjects(6, 1);
for i = 1:3
    for j = 1:2
        axesArray(plotIndex) = nexttile;
        % Advance through each longitudinal state once across the 3-by-2 layout.
        plot(time, ...
            toPlot{plotIndex}*units{plotIndex}, ...
            "DisplayName", sprintf("%s (%s)", labels{plotIndex}, unitNames{plotIndex}), ...
            'LineWidth', 1.5);
        grid on;
        % Title each tile with the state name and plotted unit.
        title(sprintf("%s (%s)", labels{plotIndex}, unitNames{plotIndex}));
        plotIndex = plotIndex + 1;
    end
end
% Link the time axes so zooming or panning one state plot keeps all states aligned.
linkaxes(axesArray, 'x');
