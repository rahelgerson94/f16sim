close all;
% Run project setup from the matlab root so this test works from any folder.
testDir = fileparts(mfilename('fullpath'));
matlabRoot = fullfile(testDir, '..', '..');
currentDir = pwd;
cd(matlabRoot); setupProject; cd(fullfile(matlabRoot, 'LinearAnalysis'));
c = getConstants();
%f16NominalTrim; % get xe, ue into the workspace
xeInW = load([matlabRoot '/generated/xeInW.mat']).xeInW;
ue = load([matlabRoot '/generated/ue.mat']).ue;
cd(currentDir);
paramsPath = fullfile(matlabRoot, 'common', 'getVehicleParams.m');
% Pass the params function path so CalcDerivsLon builds params internally.
test = CalcDerivsLon(xeInW, ue, paramsPath);
CTRL_DEFLECTION_AMPLITUDES = [5, 0.6];
dt = c.dt;
plotInB = false;
plotInW = true;
%% simulation parameters
SIM_DURATION = 8;
SIM_DURATION_SAMPLES = ceil(SIM_DURATION/dt);
% Build the time vector used by the longitudinal state plot.
time = linspace(0, SIM_DURATION, SIM_DURATION_SAMPLES);

STEP_DURATION_SAMPLES = 0.5/c.dt;
X_LON_IN_W = zeros(SIM_DURATION_SAMPLES,6);
X_LON_IN_B = zeros(SIM_DURATION_SAMPLES,6);
FORCES_IN_B = zeros(SIM_DURATION_SAMPLES,3);
MOMENTS_IN_B = zeros(SIM_DURATION_SAMPLES,3);

ET = zeros(SIM_DURATION_SAMPLES, test.nControls);
VTs = {}; ALPHAS = {}; Qs={}; THETAS = {}; Ns={}; Ds={};
M_AEROs={}; F_AEROs={};responseLabels = {"elevator", "throttle"};

%begin loop
subscripts = {'e', 't'};
for controlIdx = 1:2
    % Start each input response from trim so the overlay compares independent doublets.
    test = CalcDerivsLon(xeInW, ue, paramsPath);
    CTRL_INP_IDX = controlIdx;
    X_LON_IN_W = zeros(SIM_DURATION_SAMPLES,6);
    X_LON_IN_B = zeros(SIM_DURATION_SAMPLES,6);
    FORCES_IN_B = zeros(SIM_DURATION_SAMPLES,3);
    MOMENTS_IN_B = zeros(SIM_DURATION_SAMPLES,3);
    ET = getDoublet(SIM_DURATION_SAMPLES, ...
        STEP_DURATION_SAMPLES, ...
        CTRL_INP_IDX, ...
        ue , ...
        CTRL_DEFLECTION_AMPLITUDES(CTRL_INP_IDX), ...
        test);
    for sampleIdx = 1:SIM_DURATION_SAMPLES
        u=ET(sampleIdx,:);
        test.updateWithFinW(u);
        xInW = test.getStateInW()';
        % Store the column state vector as one row in the simulation history.
        X_LON_IN_W(sampleIdx,:) = xInW;
        
        
        %X_LON_IN_B(sampleIdx,:) = test.getStateInB();
        [f,m] = test.getAeroFMInW( xInW, u);
        FORCES_IN_B(sampleIdx,:) = -1*wind2body(f, test.alpha, 0);
        MOMENTS_IN_B(sampleIdx,:) = -1*wind2body(m,test.alpha, 0);
    end
    F_AEROs{controlIdx}=FORCES_IN_B; M_AEROs{controlIdx} = MOMENTS_IN_B;
    [VT, THETA, ALPHA, Q,N,D] = unpackXlonInW(X_LON_IN_W, test);
    VTs{controlIdx}=VT; ALPHAS{controlIdx} = ALPHA; Qs{controlIdx} = Q; THETAS{controlIdx}=THETA; Ns{controlIdx} = N; Ds{controlIdx} = D;
    plotScalar(time, ET(:,CTRL_INP_IDX), sprintf("δ_%s", subscripts{CTRL_INP_IDX}));

end


 plotStatesLonInWresponse2eleAndThrOverlayed(time, VTs, ALPHAS, Qs, THETAS, Ns,Ds, F_AEROs, M_AEROs);
 
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
