function [Vt, a, b] = uvw2mab(vInB)
if abs(vecnorm(vInB)) < 0.001
    a = 0; b = 0;
else

    u = vInB(1); v = vInB(2); w = vInB(3); Vt = vecnorm(vInB);
    a = atan(w/u);
    b = asin(v/Vt);
    
end
end