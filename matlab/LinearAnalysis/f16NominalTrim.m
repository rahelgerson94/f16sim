%TABLE 3.6-3 Trimmed Flight Conditions for the F-1

VtFt = 502; %ft/s
alpha = 0.03691; %rad
theta = 0.03691;%rad
alphaDeg = alpha*c.RAD2DEG;
thetaDeg = theta*c.RAD2DEG;
q = 0; %rad
h = 10000*c.FT2M; % ft
thtl = 0.1385; % (0,1)
ele = -0.7588; %deg



ue = [ele; thtl];

u = VtFt*cos(alpha);
w = VtFt*sin(alpha);
xe = [u; w; q; theta; 0;-h];

xeWind = [VtFt alphaDeg  q thetaDeg 0 -h]';
fprintf("α (deg): %.2f \n", atan2(w,u)*c.RAD2DEG);
fprintf("(u, w) = %.2f , %.2f\n", u*c.M2FT, w*c.M2FT );
fprintf("ue : [δ_e, δ_t] = [%.2f   %.2f]\n", ue)
fprintf("[Vt (ft),  α (deg), θ (deg), q (deg)]  = [%.2f    %.2f     %.2f     %.2f ]\n", VtFt, alphaDeg, q, thetaDeg);
%fprintf("α = %.2f, β = %.2f\n", alpha, beta)