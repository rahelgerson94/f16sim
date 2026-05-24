%close all;
matlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dataDir = fullfile(matlabRoot, 'AeroModel', 'data');
paramsPath = fullfile(matlabRoot, 'common', 'getVehicleParams.m');
addpath(matlabRoot);
addpath(fullfile(matlabRoot, 'AeroModel'));
addpath(fullfile(matlabRoot, 'tools'));
addpath(fullfile(matlabRoot, 'KinematicMath'));
addpath(fullfile(matlabRoot, 'LinearAnalysis'));
addpath(fullfile(matlabRoot, 'common'));

c = getConstants();
xeInW = load([matlabRoot '/generated/xeInW.mat']).xeInW;
ue = load([matlabRoot '/generated/ue.mat']).ue;
fprintf("ue : [δ_e, δ_t] = [%.2f   %.2f]\n", ue);
fprintf("[Vt (ft),   θ (deg), α (deg), q (deg)]  = [%.2f    %.2f     %.2f     %.2f ]\n", xeInW(1), xeInW(2), xeInW(3), xeInW(4));
Vt = xeInW(c.lon.VT_IDX)*c.FT2M;
wB_rad = [0; xeInW(c.lon.Q_IDX); 0];
aert = [0; ue(c.lon.ELE_IDX); 0; ue(c.lon.THTL_IDX)];
hM = -xeInW(6)*c.FT2M;
alphaDeg = xeInW(c.lon.ALF_IDX);
betaDeg = 0;

% Pass only paths; AeroModel owns params construction.
aeromodel = AeroModel(dataDir, paramsPath);

% Control deflection limits.
ailLims = [-30, 30];
eleLims = [-25, 25];
rudLims = [-30, 30];
N = 80;
ailRange = linspace(ailLims(1), ailLims(2), N);
eleRange = linspace(eleLims(1), eleLims(2), N);
rudRange = linspace(rudLims(1), rudLims(2), N);

% Sweep each control with one shared helper to avoid duplicated loop logic.
FMvsAileron = getFMwrtDeflection(aeromodel, Vt, alphaDeg, betaDeg, wB_rad, hM, aert, ailRange, 1, c);
FMvsElevator = getFMwrtDeflection(aeromodel, Vt, alphaDeg, betaDeg, wB_rad, hM, aert, eleRange, 2, c);
FMvsRudder = getFMwrtDeflection(aeromodel, Vt, alphaDeg, betaDeg, wB_rad, hM, aert, rudRange, 3, c);

% Check that each sweep generated finite force and moment coefficient arrays.
validateFMwrtDeflection(FMvsAileron, N);
validateFMwrtDeflection(FMvsElevator, N);
validateFMwrtDeflection(FMvsRudder, N);

plotForceMomentDashboard(ailRange, FMvsAileron(:,:,1), FMvsAileron(:,:,2));
sgtitle("Control Derivatives, Aileron");
plotForceMomentDashboard(eleRange, FMvsElevator(:,:,1), FMvsElevator(:,:,2));
sgtitle("Control Derivatives, Elevator");
plotForceMomentDashboard(rudRange, FMvsRudder(:,:,1), FMvsRudder(:,:,2));
sgtitle("Control Derivatives, Rudder");

function FMwrtDeflection = getFMwrtDeflection(aeromodel, Vt, alphaDeg, betaDeg, wB_rad, hM, baseAert, deflectionRange, ctrlIdx, c)
% Sweep one control deflection and return force and moment coefficient histories.
numDeflections = numel(deflectionRange);
FMwrtDeflection = zeros(numDeflections, 3, 2);
rho = rhoFromAlt(hM);
alphaRad = alphaDeg*c.DEG2RAD;
betaRad = betaDeg*c.DEG2RAD;

for deflectionIdx = 1:numDeflections
    % Copy the trim controls so only the selected control surface changes.
    aert = baseAert;
    aert(ctrlIdx) = deflectionRange(deflectionIdx);

    [cfStruct, cmStruct, ~] = aeromodel.updateCoeffs(alphaDeg, betaDeg, Vt, wB_rad, rho, aert);

    % Store force coefficients in wind axes and moment coefficients in body axes.
    cfInB = [cfStruct.Cx; cfStruct.Cy; cfStruct.Cz];
    cmInB = [cmStruct.Cl; cmStruct.Cm; cmStruct.Cn];
    FMwrtDeflection(deflectionIdx,:,1) = body2windCoeffs(cfInB, alphaRad, betaRad);
    FMwrtDeflection(deflectionIdx,:,2) = cmInB;
end
end

function validateFMwrtDeflection(FMwrtDeflection, numDeflections)
% Keep script failures obvious if the sweep helper changes shape or returns bad data.
assert(isequal(size(FMwrtDeflection), [numDeflections, 3, 2]), "FMwrtDeflection must be sized [N, 3, 2].");
assert(all(isfinite(FMwrtDeflection), "all"), "FMwrtDeflection must contain only finite values.");
end
