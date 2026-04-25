global V_FT_S; global ALT_FT; global params; global c;
c=getConstants();
params = getVehicleParams(c);
global aeroModel;
%declarations
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
STEP_DURATION_SAMPLES = ceil(STEP_DURATION/dt);

aert = zeros(4,SIM_DURATION_SAMPLES);aert(ELE_IDX, ...
    APPLICATION_START_IDX:APPLICATION_START_IDX+STEP_DURATION_SAMPLES) = 15; %step for rudder
aeroModel =  AeroModel(dataDir, ...
    '', ... %cfgDir
    params.Sref, ...
    params.bref, ...
    params.cref);

%% initial values
x0 = [1000; 0; 50;
     0; 0; 0; 
     1; 0; 0; 0;
     0; 0; -1000*c.FT2M
     ];
     x=x0;
time = linspace(0,SIM_DURATION ,SIM_DURATION_SAMPLES );
X = zeros(SIM_DURATION_SAMPLES,13);
FORCES=zeros(SIM_DURATION_SAMPLES,3);
MOMENTS=zeros(SIM_DURATION_SAMPLES,3);
for i = 2:SIM_DURATION_SAMPLES
    u = aert(:,i);
    [xDot,FaeroInB, MaeroInB] = getXdotFromZ([x;u;], aeroModel);
    X(i,:) =  X(i-1,:) + xDot'*dt;
    FORCES(i,:)=FaeroInB;
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
