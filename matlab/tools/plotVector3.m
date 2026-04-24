function fig = plotVector3(t, y, title)
%PLOTVECTOR3 Plot a 3-component vector signal on stacked subplots.
%
%   fig = plotVector3(t, y, title)
%
% Expects y to be an N-by-3 array with columns corresponding to x, y, z.

if size(y, 2) ~= 3
    error('y must be an N-by-3 array.');
end

if numel(t) ~= size(y, 1)
    error('t must have the same number of samples as the rows of y.');
end

labels = {'x', 'y', 'z'};

fig = figure('Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:3
    nexttile;
    plot(t, y(:, i), 'LineWidth', 1.5);
    grid on;
    ylabel(labels{i});
    if i == 3
        xlabel('t');
    end
end

sgtitle(title);

end
