t = (0:0.1:1)';
y1 = [t t.^2 t.^3];
y2 = 2*y1;
y3 = 3*y1;
plotNvector3s(t, {y1, y2, y3}, {"one", "two", "three"}, "test");
