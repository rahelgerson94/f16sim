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
Vt = xeInW(c.lon.VT_IDX)*c.FT2M;
wB_rad = [0 ;  xeInW(c.lon.Q_IDX); 0];
aert = [0; ue(c.lon.ELE_IDX); 0; ue(c.lon.THTL_IDX)];
hM = -xeInW(6)*c.FT2M;

c = getConstants();
params = getVehicleParams();
% Pass only paths; AeroModel owns params construction.
aeromodel = AeroModel(dataDir, paramsPath);

N = 400;
alphaVec = linspace(-20, 80, N);
betaVec = linspace(-30, 30, N);
dhVec = linspace(-25, 25, N);
 
CfTotinB = zeros(N,3);
CmTotInB = zeros(N,3);
CfTotInW = zeros(N,3);
CmTotInW = zeros(N,3);
CmDampingInB = zeros(N,3);
CmInB =  zeros(N,3);
DCL = zeros(N,3);
deltaCmDampingInB= zeros(N,3);
for i = 1:N
    alphaDeg = alphaVec(i);  dh = dhVec(i);
    rho =  rhoFromAlt(hM);
    [cxStruct, cmStruct, cmDampingStruct] = aeromodel.updateCoeffs(alphaDeg, 0, Vt, wB_rad, rho, aert);

    Cf=[cxStruct.Cx; cxStruct.Cy; cxStruct.Cz];
    CmTot = [cmStruct.Cl; cmStruct.Cm; cmStruct.Cn];
    
    CfTotinB(i,:) = Cf;
    CmTotInB(i,:)=CmTot;

    % Convert the coefficient vectors with radians because body2wind uses sin/cos.
    alphaRad = alphaDeg*c.DEG2RAD;
    % multiply by -1 because D is along -xb,
    % C along -yb, L along -zb
    CmInB(i,:)= aeromodel.Cm(alphaDeg, 0,0);
    Cx = aeromodel.Cx(alphaDeg,0,0);
    Cz = aeromodel.Cz(alphaDeg,0,0);
    Cy=aeromodel.Cy(alphaDeg,0);
    DCL(i,:)=body2windCoeffs([Cx Cy Cz], alphaRad, 0);
    CfTotInW(i,:) = body2windCoeffs(Cf, alphaRad, 0);
    CmTotInW(i,:)= body2windCoeffs(CmTot, alphaRad, 0);
    CmDampingInB(i,:) =  [cmDampingStruct.Cl; ...
                                        cmDampingStruct.Cm; ...
                                        cmStruct.Cn];...
    deltaCmDampinginB(i,:) = aeromodel.Cm(alphaDeg, 0,0);

end 
%plotForceMomentDashboard(alphaVec, CfinB, CmInB);
plotVector3(alphaVec, DCL, "C_d, C_y, C_L vs α (deg)");

figure; plot(alphaVec,CmInB);
plotForceMomentDashboard(alphaVec, CfTotInW, CmTotInW);
figure; labels ={"l", "m", "n"}; colors = {"r", "b", "g"};
for i = 1:3
    plot(alphaVec, CmDampingInB(:,i), "DisplayName", sprintf("C_%s", labels{i}), ...
        "LineWidth",1.5,...
    "Color" , colors{i}); hold on;
end
legend; grid on; xlabel("α (deg)");
hold off;
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
