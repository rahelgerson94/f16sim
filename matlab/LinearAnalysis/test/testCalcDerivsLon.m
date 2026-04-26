
cd ..; setupProject; cd LinearAnalysis;
f16NominalTrim; % get xe, ue into the workspace
test = CalcDerivsLon(xe, ue);
dt = c.dt;
SIM_DURATION = 5;

AIL_IDX = 1;
ELE_IDX = 2;
RUD_IDX = 3;
%% simulation parameters
SIM_DURATION = 5;
SIM_DURATION_SAMPLES = ceil(SIM_DURATION/dt);
APPLICATION_START_TIME = 2; 
APPLICATION_START_IDX = ceil(APPLICATION_START_TIME/dt); 

STEP_DURATION = 1;
STEP_DURATION_SAMPLES = 1*c.dt;

X_LON = zeros(SIM_DURATION_SAMPLES,6);
U = zeros(2,SIM_DURATION_SAMPLES);
test.setInitialState(xe);

for i = 1:SIM_DURATION_SAMPLES
    test.update(ue);
    X_LON(i,:) = test.getState();
    
end

 [UVW, PQR, Q, NED] = unpackX(X_LON);
 plotStatesLon(time, U,W,Q,TH);

 function [U,W,Q,TH, N,D] = unpackXlon(Xlon)
    U = X(:, 1); W = X(:, 2);
    Q = X(:, 3); TH = X(:, 4);
    N = X(:, 5); D = X(:, 6);
    end
