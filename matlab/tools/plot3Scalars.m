function fig = plot3Scalars(t, y1, y2,y3, titles, supTitle)
%PLOTVECTOR3 Plot a 3-component vector signal on stacked subplots.
%
%   fig = plotVector3(t, y, title)
%
% Expects y to be an N-by-3 array with columns corresponding to x, y, z.

if numel(t) ~= size(y1, 1)
    error('t must have the same number of samples as the rows of y.');
end

labels = {'x', 'y', 'z'};

fig = figure('Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
ys = {y1,y2,y3};
for i = 1:3
    nexttile;
    plot(t, ys{i}, 'LineWidth', 1.5, "DisplayName", titles(i));
    grid on;
    legend;
    ylabel(labels{i});
    
    if i == 3
        xlabel('t');
    end
end

%sgtitle(supTitle);

end
