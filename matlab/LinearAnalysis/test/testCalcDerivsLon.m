
% Run project setup from the matlab root so this test works from any folder.
testDir = fileparts(mfilename('fullpath'));
matlabRoot = fullfile(testDir, '..', '..');
currentDir = pwd;
cd(matlabRoot); setupProject; cd(fullfile(matlabRoot, 'LinearAnalysis'));
c = getConstants();
f16NominalTrim; % get xe, ue into the workspace
cd(currentDir);
test = CalcDerivsLon(xe, ue);
dt = c.dt;


%% simulation parameters
SIM_DURATION = 4;
SIM_DURATION_SAMPLES = ceil(SIM_DURATION/dt);
% Build the time vector used by the longitudinal state plot.
time = linspace(0, SIM_DURATION, SIM_DURATION_SAMPLES);

STEP_DURATION_SAMPLES = 1/c.dt;
X_LON = zeros(SIM_DURATION_SAMPLES,6);
FORCES = zeros(SIM_DURATION_SAMPLES,3);
MOMENTS = zeros(SIM_DURATION_SAMPLES,3);
test.setInitialState(xe);

for i = 1:SIM_DURATION_SAMPLES
    if i <= STEP_DURATION_SAMPLES
        u = ue;
    else
        u = zeros(2,1);
    end
    test.updateWithF(u);
    % Store the column state vector as one row in the simulation history.
    X_LON(i,:) = test.getState()';
    [f,m] = test.getAeroFM(test.x, u);
    FORCES(i,:) = f;
    MOMENTS(i,:) = m;
end

 [U,W,Q,TH, N,D] = unpackXlon(X_LON);
 plotStatesLon(time, U,W,Q,TH, N,D);
 plotVector3(time, FORCES, "Faero");
 plotVector3(time, MOMENTS, "Maero");
 function [U,W,Q,TH, N,D] = unpackXlon(Xlon)
    % Split the longitudinal state history into scalar traces for plotting.
    U = Xlon(:, 1); W = Xlon(:, 2);
    Q = Xlon(:, 3); TH = Xlon(:, 4);
    N = Xlon(:, 5); D = Xlon(:, 6);
    end
