
function params = getVehicleParams()
% Inertia and geometry data for F16.
% All parameters are in SI Units.
% c: the returned value from getConstants
c = getConstants();
params.geometry.Sref = 300*c.FT2M^2;      % Reference Area, m^2
params.geometry.bref =  30*c.FT2M;        % Wing Span, m
params.geometry.cref =  11.32*c.FT2M;     % Aerodynamic Mean Chord, m
params.rho = 1.293;         % Air Density, kg/m^3
params.geometry.xcgr = 0.35*params.geometry.cref;         % reference center of gravity as a fraction of cbar
params.geometry.xcg  = 0.30*params.geometry.cref;         % center of gravity as a fraction of cbar.

%% MASS PROPERTIES
Ixx = 9496.0;            % Principle Moment of Intertia around X-axis, slugs*ft^2
Iyy = 55814.0;           % Principle Moment of Intertia around Y-axis, slugs*ft^2
Izz = 63100.0;           % Principle Moment of Intertia around Z-axis, slugs*ft^2 
Ixz = 982.0;           % Principle Moment of Intertia around XZ-axis,slugs*ft^2
params.geometry.Ixx = Ixx*c.SLUG2KG*(c.FT2M)^2;
params.geometry.Iyy = Iyy*c.SLUG2KG*(c.FT2M)^2;
params.geometry.Izz = Izz*c.SLUG2KG*(c.FT2M)^2;
params.geometry.Ixz = Ixz*c.SLUG2KG*(c.FT2M)^2;
params.geometry.I = [Ixx 0 Ixz;
             0   Iyy 0
             Ixz 0   Izz]*c.SLUG2KG*(c.FT2M)^2; % Convert to Kg*m^2

params.mass = 636.94*c.SLUG2KG;     % kg


%% CONTROL SURFACE DEFLECTION LIMITS
params.limits.cs.eSym = 25*c.DEG2RAD;  % (±) symmetric deflection
params.limits.cs.eDif = 5.375*c.DEG2RAD; % (±) differential deflection
params.limits.cs.a = 21.5; % (±)
params.limits.cs.r = 30; % (±)
params.limits.cs.lef = 25; % ONLY +
params.limits.cs.sb = 60; % ONLY +
end
