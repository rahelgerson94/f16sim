
function params = getVehicleParams(c)
% Inertia and geometry data for F16.
% All parameters are in SI Units.
% c: the returned value from getConstants
params.Sref = 300*c.FT2M^2;      % Reference Area, m^2
params.bref =  30*c.FT2M;        % Wing Span, m
params.cref =  11.32*c.FT2M;     % Aerodynamic Mean Chord, m
params.rho = 1.293;         % Air Density, kg/m^3
params.xcgr = 0.35;         % reference center of gravity as a fraction of cbar
params.xcg  = 0.30;         % center of gravity as a fraction of cbar.
params.g = 9.866;

% Moment of Inertial
Ixx = 9496.0;            % Principle Moment of Intertia around X-axis, slugs*ft^2
Iyy = 55814.0;           % Principle Moment of Intertia around Y-axis, slugs*ft^2
Izz = 63100.0;           % Principle Moment of Intertia around Z-axis, slugs*ft^2 
Ixz = 982.0;           % Principle Moment of Intertia around XZ-axis,slugs*ft^2
params.Ixx = Ixx*c.SLUG2KG*(c.FT2M)^2;
params.Iyy = Iyy*c.SLUG2KG*(c.FT2M)^2;
params.Izz = Izz*c.SLUG2KG*(c.FT2M)^2;
params.Ixz = Ixz*c.SLUG2KG*(c.FT2M)^2;
params.I = [Ixx 0 Ixz;
             0   Iyy 0
             Ixz 0   Izz]*c.SLUG2KG*(c.FT2M)^2; % Convert to Kg*m^2

params.mass = 636.94*c.SLUG2KG;     % kg

end
