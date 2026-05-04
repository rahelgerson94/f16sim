t = (0:0.1:1)';
y1 = [t t.^2 t.^3];
y2 = 2*y1;
y3 = 3*y1;

% Plot three overlaid vector responses to verify N-response support.
fig = plotNvector3s(t, {y1, y2, y3}, {"one", "two", "three"}, "test");
axesArray = findall(fig, 'Type', 'axes');
lineCounts = arrayfun(@(axisHandle) numel(findall(axisHandle, 'Type', 'line')), axesArray);
assert(all(lineCounts == 3), 'Each vector component axis should contain three traces.');

% Verify labels are required to match the number of response traces.
try
    plotNvector3s(t, {y1, y2}, {"one"}, "bad labels");
    error('plotNvector3s_test:ExpectedLabelMismatch', ...
        'plotNvector3s should reject mismatched label and response counts.');
catch err
    assert(contains(err.message, 'labels must have the same number of elements as yCellArray'), ...
        'Unexpected label mismatch error message.');
end
