%TABLE 3.6-3 Trimmed Flight Conditions for the F-1

Vt = 502*c.FT2M; %ft/s
alpha = 0.03691; %rad
theta = 0.03691;%rad
q = 0;
h = 0;
thtl = 0.1385;
ele = -0.7588;



ue = [ele; thtl];

u = Vt*cos(alpha);
w = Vt*sin(alpha);
xe = [u; w; q; theta; 0;0];
fprintf("alpha (deg): %.2f \n", atan2(w,u)*c.RAD2DEG);
fprintf("(u, w) = %.2f , %.2f\n", u*c.M2FT, w*c.M2FT );