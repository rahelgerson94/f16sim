
TOL = 0.01;
u = 4; w = -2; v = 0;
vInB = [u; v; w];
alpha = atan(w/u);
c= getConstants();
alphaDeg = alpha*c.RAD2DEG;

vInW = body2wind(vInB, alpha, 0);

if abs(vInW(2) - v ) > 0.01
    fprintf("v ≠ 0, v = %.2f\n",  vInW(2));
end 

if abs(vInW(3) - w)  > 0.1
    fprintf("w ≠ 0, w = %.2f\n",  vInW(3));
end 

if abs(sqrt(u^2 + w^2)  - vInW(1))> 0.1
    fprintf("Vt ≠ norm(u*u + w*w), Vt = %.2f\n",  vInW(1));
end 

[u_,v_,w_] = mab2uvw( vecnorm(vInB), 0, alpha);
if abs(u_ - u)  > 0.01
    fprintf("error in u_: u = %.2f\n",  u_);
end 

if abs(w_ - w) > 0.01
    fprintf("error in w_: w = %.2f\n",  w_);
end 
%% test wind2body
Vt = [vecnorm(vInB); 0; 0];
vInB_act = wind2body(Vt, alpha, 0);
labels = {"u", "v", "w"};
for i = 1:3
    if abs(vInB(i) - vInB_act(i) ) > 0.01
    fprintf("%s error: u = %.2f ft/s\n", labels{i},  vInB_act(i)*c.FT2M);
    end
end 

%% test uvw2mab with non zero beta 
V = 100*c.FT2M;
vinB =V* [0.5; 0.86; 0];
[Vt,a, b] = uvw2mab(vinB);

bExp = 60*c.DEG2RAD;
alfExp = 0;

Vexp = V;
if abs(bExp - b) > TOL
    fprintf("β error: β = %.2f deg\n", b*c.RAD2DEG);
end

if abs(alfExp - a) > TOL
    fprintf("α error: α  = %.2f deg\n", a*c.RAD2DEG);
end

if abs(Vexp - V) > TOL
    fprintf("Vt error: Vt  = %.2f deg\n", Vt);
end
