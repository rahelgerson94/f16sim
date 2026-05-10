function c = getConstants()
% ================================
% Conversions and Constants
% ================================

c = struct();

% ---------- Math ----------
c.PI = 3.141592653589793;

% ---------- constants ----------
c.g = 9.80665;  % m/s^2
c.MACH1_FT_S = 1125.328; % speed of sound = 1125.328 feet per second 
c.MACH1_M_S = 343; % speed of sound = 343 meters per second 
% ---------- Mass ----------
c.SLUG2KG = 14.5939;
c.KG2SLUG = 1.0 / c.SLUG2KG;

c.LBM2KG = 0.45359237;
c.KG2LBM = 1.0 / c.LBM2KG;

% ---------- Length ----------
c.FT2M = 0.3048;
c.M2FT = 1.0 / c.FT2M;

c.IN2FT = 1.0 / 12.0;
c.FT2IN = 12.0;

c.IN2M = 0.0254;
c.M2IN = 1.0 / c.IN2M;

% ---------- Force ----------
c.LBF2N = 4.4482216152605;
c.N2LBF = 1.0 / c.LBF2N;

% ---------- Angles ----------
c.RAD2DEG = 180.0 / c.PI;
c.DEG2RAD = c.PI / 180.0;

% ---------- Area ----------
c.FT22M2 = c.FT2M ^ 2;
c.M22FT2 = c.M2FT ^ 2;

c.IN22M2 = c.IN2M ^ 2;
c.M22IN2 = 1.0 / c.IN22M2;

% ---------- Volume ----------
c.FT32M3 = c.FT2M ^ 3;
c.M32FT3 = c.M2FT ^ 3;

c.IN32M3 = c.IN2M ^ 3;
c.M32IN3 = 1.0 / c.IN32M3;

% ---------- Pressure ----------
c.PSF2PA = 47.88025898;
c.PA2PSF = 1.0 / c.PSF2PA;

c.PSI2PA = 6894.757293168;
c.PA2PSI = 1.0 / c.PSI2PA;

% ---------- Density ----------
c.SLUGFT32KGPM3 = c.SLUG2KG / c.FT32M3;
c.KGPM32SLUGFT3 = 1.0 / c.SLUGFT32KGPM3;

% ---------- Convenience ----------
c.ZERO_VEC = zeros(3,1);
c.ZERO_Q = [1,0,0,0];
c.nStates = 13;
c.dt = 0.01;

% ---------- Indexing ----------
c.i_vB = 1;
c.i_wB = c.i_vB + 1 + 3;
c.i_q  = c.i_wB + 1 + 3;
c.i_rI = c.i_q +  1 + 4;

c.DRAG_IDX = 1; c.LIFT_IDX = 3; c.Y_IDX=2;
c.lon.VT_IDX = 1; 
c.lon.TH_IDX =2;
c.lon.ALF_IDX = 3; 
c.lon.Q_IDX=4;
c.lon.ELE_IDX = 1; 
c.lon.THTL_IDX=2;
end