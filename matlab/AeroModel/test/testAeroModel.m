%close all;
c= getConstants();
matlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dataDir = fullfile(matlabRoot, 'AeroModel', 'data');
addpath(matlabRoot);
addpath(fullfile(matlabRoot, 'AeroModel'));
addpath(fullfile(matlabRoot, 'tools'));
addpath(fullfile(matlabRoot, 'KinematicMath'));
addpath(fullfile(matlabRoot, 'LinearAnalysis'));
f16NominalTrim; %xe = [u; w; q; theta; 0;-h];  xeWind= [VtFt alphaDeg  q thetaDeg 0 -h]';
Vt = xeWind(1)*c.FT2M;
wB_rad = [0 ;  xeWind(3); 0];
aert = [0; ue(1); 0; ue(2)];
hM = -xeWind(6)*c.FT2M;

c = getConstants();
params = getVehicleParams(c);
aeromodel = AeroModel(dataDir, '', params.Sref, params.bref, params.cref);

N = 100;
alphaVec = linspace(-25, 25, N);
betaVec = linspace(-30, 30, N);
dhVec = linspace(-25, 25, N);
 

N = length(alphaVec);
CfinB = zeros(N,3);
CmInB = zeros(N,3);
CfinW = zeros(N,3);
CmInW = zeros(N,3);
CmDampingInB = zeros(N,3);
for i = 1:N
    alpha = alphaVec(i);  dh = dhVec(i);
    rho =  rhoFromAlt(hM);
    [cxStruct, cmStruct, cmDampingStruct] = aeromodel.getCoeffs(alpha, 0, Vt, wB_rad, rho, aert);

    Cf=[cxStruct.Cx; cxStruct.Cy; cxStruct.Cz];
    Cm = [cmStruct.Cl; cmStruct.Cm; cmStruct.Cn];

    CfinB(i,:) = Cf;
    CmInB(i,:)=Cm;
    % Convert the coefficient vectors with radians because body2wind uses sin/cos.
    alphaRad = alpha*c.DEG2RAD;
       CfinW(i,:) = body2wind(Cf, alphaRad, 0);
    % CmInW(i,:)= body2wind(Cm, alphaRad, 0);
    % Cd = -cos(alphaRad)*cxStruct.Cx + sin(alphaRad)*cxStruct.Cz;
    % Cl= sin(alphaRad)*cxStruct.Cx      + cos(alphaRad)*cxStruct.Cz;
    % CfinW(i,:) = [Cd; 0; Cl];
end

tiledlayout(1,2); L = 1.5;
nexttile;
% Plot against the full alpha sweep, not the last scalar alpha from the loop.
plot(alphaVec, CfinB(:,c.DRAG_IDX), "DisplayName", "C_x", "Color","blue",LineWidth=L); hold on;
plot(alphaVec, CfinW(:,c.DRAG_IDX), "DisplayName", "C_D","Color","black",LineWidth=L); hold off;
xlabel("α (deg)"); grid on; legend;

nexttile;
% Compare body z-force coefficient with the wind-frame lift-axis coefficient.
plot(alphaVec, CfinB(:,c.LIFT_IDX), "DisplayName", "C_z", "Color","blue",LineWidth=L); hold on;
plot(alphaVec, CfinW(:,c.LIFT_IDX), "DisplayName", "C_L","Color","black", LineWidth=L); hold off;
xlabel("α (deg)"); grid on; legend;

tiledlayout(2,1);
