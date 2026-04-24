function rho = rhoFromAlt(h)
% h in meters, rho in kg/m^3

T0 = 288.15;      % sea level temp (K)
P0 = 101325;      % sea level pressure (Pa)
L  = 0.0065;      % lapse rate (K/m)
R  = 287.058;     % gas constant (J/kg/K)
g  = 9.80665;     % gravity (m/s^2)

if h <= 11000  % Troposphere
    T = T0 - L*h;
    P = P0 * (T/T0)^(g/(R*L));
else           % Lower stratosphere (isothermal)
    T = 216.65;
    P11 = P0 * ( (T/T0)^(g/(R*L)) );
    P = P11 * exp(-g*(h-11000)/(R*T));
end

rho = P / (R*T);
end