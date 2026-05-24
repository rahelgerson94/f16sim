%close all;
c= getConstants();
matlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dataDir = fullfile(matlabRoot, 'AeroModel', 'data');
paramsPath = fullfile(matlabRoot, 'common', 'getVehicleParams.m');
addpath(matlabRoot);
addpath(fullfile(matlabRoot, 'AeroModel'));
addpath(fullfile(matlabRoot, 'tools'));
addpath(fullfile(matlabRoot, 'KinematicMath'));
addpath(fullfile(matlabRoot, 'LinearAnalysis'));

xeInW = load([matlabRoot '/generated/xeInW.mat']).xeInW;
ue = load([matlabRoot '/generated/ue.mat']).ue;
fprintf("ue : [δ_e, δ_t] = [%.2f   %.2f]\n", ue)
fprintf("[Vt (ft),   θ (deg), α (deg), q (deg)]  = [%.2f    %.2f     %.2f     %.2f ]\n", xeInW(1), xeInW(2), xeInW(3), xeInW(4) );
Vt = xeInW(1)*c.FT2M;
wB_rad = [0 ;  xeInW(3); 0];
aert = [0; ue(1); 0; ue(2)];
hM = -xeInW(6)*c.FT2M;

c = getConstants();
params = getVehicleParams();
% Pass only paths; AeroModel owns params construction.
aeromodel = AeroModel(dataDir, paramsPath);

N = 400;
alphaVec = linspace(-20, 80, N);
betaVec = linspace(-30, 30, N);
dhVec = linspace(-25, 25, N);
 
cm = zeros(N,3);
CmDampingInB = zeros(N,3);
deltaCdmDampingInB =  zeros(N,3);
CmTotInB = zeros(N,3);
CfTotInW = zeros(N,3);
CmTotInW = zeros(N,3);
CmInB =  zeros(N,3);
DCL = zeros(N,3);
for i = 1:N
    betaDeg = betaVec(i);  %dh = dhVec(i); 
    dh=0;
    rho =  rhoFromAlt(hM);
    [cxStruct, cmStruct, cmDampingStruct] = aeromodel.updateCoeffs(25, betaDeg, Vt, wB_rad, rho, aert);
    CmTotInB(i,:)=CmTot;

    % Convert the coefficient vectors with radians because body2wind uses sin/cos.
    alphaRad = 0*c.DEG2RAD;
    betaRad=betaDeg*c.DEG2RAD;
    % multiply by -1 because D is along -xb,
    % C along -yb, L along -zb
    CmInB(i,:)= aeromodel.Cm(25, betaDeg,dh);
    % Cx = aeromodel.Cx(0,betaDeg,dh);
    % Cz = aeromodel.Cz(0,betaDeg,dh);
    % Cy=aeromodel.Cy(0,betaDeg);
    %DCL(i,:)=body2windCoeffs([Cx Cy Cz], alphaRad, betaRad);
    %CfTotInW(i,:) = body2windCoeffs(Cf, alphaRad, betaRad);
    CmTotInW(i,:)= body2windCoeffs(CmTot, alphaRad, betaRad);
    CmDampingInB(i,:) =  [cmDampingStruct.Cl; ...
                                        cmDampingStruct.Cm; ...
                                        cmStruct.Cn];...
    deltaCmDampinginB(i,:) = aeromodel.Cm(0, betaRad,dh);

end 
 
% tiledlayout(1,3); L =
% tiledlayout(1,3); L = 1.5;
% nexttile;
% % Plot against the full alpha sweep, not the last scalar alpha from the loop.
% plot(alphaVec, CfinB(:,c.DRAG_IDX), "DisplayName", "C_x", "Color","blue",LineWidth=L); hold on;
% plot(alphaVec, CfinW(:,c.DRAG_IDX), "DisplayName", "C_D","Color","black",LineWidth=L); hold off;
% xlabel("α (deg)"); grid on; legend;
% 
% nexttile;
% % Compare body z-force coefficient with the wind-frame lift-axis coefficient.
% plot(alphaVec, CfinB(:,c.LIFT_IDX), "DisplayName", "C_z", "Color","blue",LineWidth=L); hold on;
% plot(alphaVec, CfinW(:,c.LIFT_IDX), "DisplayName", "C_L","Color","black", LineWidth=L); hold off;
% xlabel("α (deg)"); grid on; legend;
% 
% nexttile;
% plot(alphaVec, CmInB(:,2), "DisplayName", "C_m (B)", "Color","black",LineWidth=L); hold on 
% plot(alphaVec, CmInW(:,2), "DisplayName", "C_m (W)", "Color","blue",LineWidth=L); hold on 
% 
% xlabel("α (deg)"); grid on; legend;
