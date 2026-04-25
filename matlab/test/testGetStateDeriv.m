global V_FT_S; global ALT_FT; global params; global c;
params = getVehicleParams(c);

%declarations
dt = 0.01;
SIM_DURATION = 5;
time = linspace(0,ceil(SIM_DURATION/c.dt), N);
X = zeros(N,13);
FORCES=zeros(N,3);
MOMENTS=zeros(N,3);
AIL_IDX = 1;
ELE_IDX = 2;
RUD_IDX = 3;
tApply = 2;
STEP_DURATION = 1;
%% initial values
x0 = [1000; 0; 50;
     0; 0; 0; 
     1; 0; 0; 0;
     0; 0; -1000*c.FT2M
     ];
     x=x0;
aert = zeros(N,4);
aert(ELE_IDX, tApply:tApply+STEP_DURATION) = 1; %step for rudder

for i = 2:N
    u =aert(i,:)';
    [xDot,FaeroInB, MaeroInB] = getXdotFromZ([x;u;]);
    X(i,:) =  X(i-1,:) + xDot'*c.dt;
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
