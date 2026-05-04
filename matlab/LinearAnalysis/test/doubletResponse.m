close all;
% Run project setup from the matlab root so this test works from any folder.
testDir = fileparts(mfilename('fullpath'));
matlabRoot = fullfile(testDir, '..', '..');
currentDir = pwd;
cd(matlabRoot); setupProject; cd(fullfile(matlabRoot, 'LinearAnalysis'));
c = getConstants();
f16NominalTrim; % get xe, ue into the workspace
cd(currentDir);
test = CalcDerivsLon(xe, ue);
CTRL_INP_IDX = 1;
CTRL_DEFLECTION_AMPLITUDES = [5, 0.6];
dt = c.dt;
plotInB = false;
plotInW = true;
%% simulation parameters
SIM_DURATION = 4;
SIM_DURATION_SAMPLES = ceil(SIM_DURATION/dt);
% Build the time vector used by the longitudinal state plot.
time = linspace(0, SIM_DURATION, SIM_DURATION_SAMPLES);

STEP_DURATION_SAMPLES = 0.5/c.dt;
X_LON_IN_W = zeros(SIM_DURATION_SAMPLES,6);
X_LON_IN_B = zeros(SIM_DURATION_SAMPLES,6);
FORCES_IN_B = zeros(SIM_DURATION_SAMPLES,3);
MOMENTS_IN_B = zeros(SIM_DURATION_SAMPLES,3);

ET = zeros(SIM_DURATION_SAMPLES, test.nControls);

ET = getDoublet(SIM_DURATION_SAMPLES, ...
    STEP_DURATION_SAMPLES, ...
    CTRL_INP_IDX, ...
    ue , ...
    CTRL_DEFLECTION_AMPLITUDES(CTRL_INP_IDX), ...
    test);
for i = 1:SIM_DURATION_SAMPLES
    u=ET(i,:);
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
     [VT,  THETA,ALPHA, Q, N,D] = unpackXlonInW(X_LON_IN_W, test);
        plotStatesLonInW(time, VT, ALPHA, Q, THETA, N,D);
 end
 if plotInB
     %[U,W,Q,TH, N,D]  = unpackXlonInB(X_LON_IN_B); 
      [U,W,Q,TH, N,D]  = unpackXlonInWAndConvert2B(X_LON_IN_W, test);
     plotStatesLonInB(time, U,W,Q,TH, N,D);
 end

 plotVector3(time, FORCES_IN_B, "Faero");
 plotVector3(time, MOMENTS_IN_B, "Maero");
 subscripts = {'e', 't'};
 plotScalar(time, ET(:,CTRL_INP_IDX), sprintf("δ_%s", subscripts{CTRL_INP_IDX}));
 function [VT, THETA, ALPHA, Q, N,D] = unpackXlonInW(XlonInW, calcDerivObj)
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

function ET = getDoublet(simDurationSamples, ...
    stepDurationSamples, ...
    inputIdx, ...
    eleThrottleTrim , ...
    inputAmp, ...
    calcDerivsObj)
%eleThrottleTrim: (2,1) vector representing delE, delT at trim
% inputAmp (1,1)  the amount of amplitude away from trim
trimThrottle  = eleThrottleTrim(calcDerivsObj.THROTTLE_IDX);
trimEle =  eleThrottleTrim(calcDerivsObj.ELEVATOR_IDX);

ET(:, inputIdx) =  generateDoublet(simDurationSamples, inputAmp, ...
    eleThrottleTrim(inputIdx), ...
    stepDurationSamples);

if inputIdx == calcDerivsObj.THROTTLE_IDX
    ET(:,calcDerivsObj.ELEVATOR_IDX) = ones(simDurationSamples,1)*trimEle;
else
    ET(:,calcDerivsObj.THROTTLE_IDX) = ones(simDurationSamples,1)*trimThrottle;
end
end