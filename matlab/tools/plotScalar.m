function fig = plotScalar(t, y, title)
%PLOTSCALAR Plot a scalar signal versus time.
%
%   fig = plotScalar(t, y, title)

if numel(t) ~= numel(y)
    error('t and y must have the same number of samples.');
end

fig = figure('Color', 'w');
plot(t, y, 'LineWidth', 1.5);
grid on;
xlabel('t');
ylabel('y');
sgtitle(title);

end
