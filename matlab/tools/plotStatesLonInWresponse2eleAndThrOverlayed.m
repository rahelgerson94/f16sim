function []= plotStatesLonInWresponse2eleAndThrOverlayed(time, VT, ALPHA, Q, THETA, N,D)

c = getConstants();
figure('Color', 'w');


tiledlayout(3,2);
toPlot = {VT, ALPHA, Q, THETA, N,D};
labels = {"Vt", "α", "q", "θ", "n", "d"};
units = {c.M2FT, c.RAD2DEG, c.RAD2DEG, c.RAD2DEG, c.M2FT, c.M2FT};
unitNames = {"ft/s", "deg", "deg/s", "deg",  "ft", "ft"};
responseNames = {"elevator", "throttle"};

plotIndex = 1;
axesArray = gobjects(6, 1);
for rowIdx = 1:3
    for colIdx = 1:2
        axesArray(plotIndex) = nexttile;
        hold on;
        % Plot the elevator and throttle response traces on the same state axes.
        for responseIdx = 1:2
            plot(time, ...
                toPlot{plotIndex}{responseIdx}*units{plotIndex}, ...
                "DisplayName", responseNames{responseIdx}, ...
                'LineWidth', 1.5);
        end
        hold off;
        grid on;
        % Title each tile with the state name and plotted unit.
        title(sprintf("%s (%s)", labels{plotIndex}, unitNames{plotIndex}));
        legend('Location', 'best');
        plotIndex = plotIndex + 1;
    end
end
% Link the time axes so zooming or panning one state plot keeps all states aligned.
linkaxes(axesArray, 'x');
