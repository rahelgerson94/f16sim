
% Run project setup from the matlab root so this test works from any folder.
testDir = fileparts(mfilename('fullpath'));
matlabRoot = fullfile(testDir, '..', '..');
currentDir = pwd;
cd(matlabRoot); setupProject; cd(fullfile(matlabRoot, 'LinearAnalysis'));
c = getConstants();
f16NominalTrim; % get xe, ue into the workspace

cd(currentDir);
paramsPath = fullfile(matlabRoot, 'common', 'getVehicleParams.m');
% Pass the params function path so CalcDerivsLon builds params internally.
test = CalcDerivsLon(xe, ue, paramsPath);
dt = c.dt;
plotInB = false;
plotInW = true;
%% simulation parameters
SIM_DURATION = 8;
APPLICATION_START_IDX = 1/dt;
SIM_DURATION_SAMPLES = ceil(SIM_DURATION/dt);
% Build the time vector used by the longitudinal state plot.
time = linspace(0, SIM_DURATION, SIM_DURATION_SAMPLES);

STEP_DURATION_SAMPLES = 1/c.dt;
X_LON_IN_W = zeros(SIM_DURATION_SAMPLES,6);
X_LON_IN_B = zeros(SIM_DURATION_SAMPLES,6);
FORCES_IN_B = zeros(SIM_DURATION_SAMPLES,3);
MOMENTS_IN_B = zeros(SIM_DURATION_SAMPLES,3);


for i = 1:SIM_DURATION_SAMPLES
    if i < APPLICATION_START_IDX
        u = [0 0];
    elseif i >=APPLICATION_START_IDX &&  i < APPLICATION_START_IDX  + STEP_DURATION_SAMPLES
        %u = ue;
        u = [15 0];
    else 
        u = [0 0];
    end
    test.updateWithFinW(u);
    xInW = test.getStateInW()';
    % Store the column state vector as one row in the simulation history.
    X_LON_IN_W(i,:) = xInW;
    
    
    X_LON_IN_B(i,:) = test.getStateInB();
    [f,m] = test.getAeroFMInW( xInW, u);
    FORCES_IN_B(i,:) = wind2body(f, test.alpha, 0);
    MOMENTS_IN_B(i,:) = wind2body(m,test.alpha, 0);
end


 
 if plotInW
     [VT, ALPHA, Q, THETA, N,D] = unpackXlonInW(X_LON_IN_W, test);
        plotStatesLonInW(time, VT, ALPHA, Q, THETA, N,D);
 end
 if plotInB
     %[U,W,Q,TH, N,D]  = unpackXlonInB(X_LON_IN_B); 
      [U,W,Q,TH, N,D]  = unpackXlonInWAndConvert2B(X_LON_IN_W, test);
     plotStatesLonInB(time, U,W,Q,TH, N,D);
 end

 plotVector3(time, FORCES_IN_B, "Faero");
 plotVector3(time, MOMENTS_IN_B, "Maero");
 function [VT, ALPHA, Q, THETA, N,D] = unpackXlonInW(XlonInW, calcDerivObj)
    % Split the longitudinal state history into scalar traces for plotting.
    VT = XlonInW(:, calcDerivObj.VT_IDX);        ALPHA = XlonInW(:, calcDerivObj.ALF_IDX);
    Q = XlonInW(:, calcDerivObj.Q_IDX);           THETA = XlonInW(:, calcDerivObj.TH_IDX);
    N = XlonInW(:, 5);                          D = XlonInW(:, 6);
    end
function [U,W,Q,TH, N,D] = unpackXlonInB(XlonInB)
% Split the longitudinal state history into scalar traces for plotting.
U = XlonInB(:, 1); W = XlonInB(:, 2);
Q = XlonInB(:, 3); TH = XlonInB(:, 4);
N = XlonInB(:, 5); D = XlonInB(:, 6);
end

function [U,W,Q,THETA, N,D] = unpackXlonInWAndConvert2B(XlonInW, calcDerivObj)
% Split the longitudinal state history into scalar traces for plotting.
    VT = XlonInW(:, calcDerivObj.VT_IDX);        ALPHA = XlonInW(:, calcDerivObj.ALF_IDX);
    Q = XlonInW(:, calcDerivObj.Q_IDX);           THETA = XlonInW(:, calcDerivObj.TH_IDX);
    N = XlonInW(:, 5);                          D = XlonInW(:, 6);

    U = VT.*cos(ALPHA);
    W = VT.*sin(ALPHA);
end
