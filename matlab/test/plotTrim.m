global V_FT_S; global ALT_FT; global params; global c;
params = getVehicleParams(c);

%declarations
dt = 0.01;
SIM_DURATION = 3;
time = linspace(0,ceil(SIM_DURATION/c.dt), N);
X = zeros(N,13);
FORCES=zeros(N,3);
MOMENTS=zeros(N,3);
%% initial values
S = load('zStar.mat'); zStar = S.zStar;
xStar=zStar(1:c.nStates);
uStar=zStar(c.nStates+1:end);
x0 = xStar; x = x0;
u = uStar;
z = zStar;

for i = 2:N
    
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
