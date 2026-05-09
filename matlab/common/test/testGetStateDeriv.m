global V_FT_S; global ALT_FT; global params; global c;
c=getConstants();
params = getVehicleParams();
paramsPath = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'getVehicleParams.m');
matlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dataDir = fullfile(matlabRoot, 'AeroModel', 'data');
global aeroModel;
%declarations
dt = c.dt;
SIM_DURATION = 5;
SIM_DURATION_SAMPLES = ceil(SIM_DURATION/dt);
APPLICATION_START_TIME = 2; 
APPLICATION_START_IDX = ceil(APPLICATION_START_TIME/dt); 

STEP_DURATION = 1;
STEP_DURATION_SAMPLES = ceil(STEP_DURATION/dt);

AIL_IDX = 1;
ELEVATOR_IDX = 2;
RUD_IDX = 3;
aeroModel =  AeroModel(dataDir, ...
    paramsPath);
%% simulation parameters

%% initial values
x0 = [1000; 0; 50;
     0; 0; 0; 
     1; 0; 0; 0;
     0; 0; -1000*c.FT2M
     ];
 x=x0;
time = linspace(0,SIM_DURATION ,SIM_DURATION_SAMPLES );
X = zeros(SIM_DURATION_SAMPLES,13);
X(1,:) = x0;
FORCES=zeros(SIM_DURATION_SAMPLES,3);
MOMENTS=zeros(SIM_DURATION_SAMPLES,3);
AERT = zeros(SIM_DURATION_SAMPLES,4);
AERT( APPLICATION_START_IDX:APPLICATION_START_IDX+STEP_DURATION_SAMPLES, ELEVATOR_IDX) = 15; %step for rudder
for i = 2:SIM_DURATION_SAMPLES
    u = AERT(i,:)';
    x = X(i-1,:)';
    [xDot,FaeroInB, MaeroInB] = getXdotFromZ([x ; u], aeroModel);
    
    X(i,:) =  x' + xDot'*dt;
    
    FORCES(i,:)=FaeroInB';
    MOMENTS(i,:)=MaeroInB';
    
   
end

 [UVW, PQR, Q, NED] = unpackX(X);
 plotStates(time, UVW, PQR, FORCES, MOMENTS);

function [UVW, PQR, Q, NED] = unpackX(X)
UVW = X(:, 1:3);
PQR = X(:, 4:6);
Q = X(:, 7:10);
NED=X(:,11:13);

end
