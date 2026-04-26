function []= plotStatesLon(time, U,W,Q,THETA, N,D)
close all;

c = getConstants();
figure('Color', 'w');


tiledlayout(3,2);
toPlot = {U,W,Q,THETA, N,D};
labels = {"u", "w", "q", "theta", "n", "d"};
units = {c.M2FT, c.M2FT, c.RAD2DEG, c.RAD2DEG, c.M2FT, c.M2FT};
unitNames = {"ft/s", "ft/s", "deg/s", "deg",  "ft", "ft"};
for i = 1:3
    for j = 1:2
        nexttile;
        plot(time, ...
            toPlot{i}*units{i}, ...
            "DisplayName", sprintf("%s (%s)", labels{i}, unitNames{i}));
    end
end