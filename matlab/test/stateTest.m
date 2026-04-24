prp = fileparts(fileparts(mfilename('fullpath')));
dataDir = fullfile(prp, 'AeroModel', 'data');
addpath(prp);
addpath(fullfile(prp, 'AeroModel'));
addpath(fullfile(prp, 'tools'));
addpath(fullfile(prp, 'KinematicMath'));
c=getConstants();
params = getVehicleParams(c);
c = getConstants();
dt = 0.01;
N = 400;
s = State(dt, getVehicleParams(c));
s.setInitialState(ned0 = [0; 0; -1000*c.FT2M], ...
                  vInB0 = [1000; 0; 50]*c.FT2M, ...
                  qI2B0 = [1; 0; 0; 0]);
STEP_DURATION = 10;
time = linspace(0,dt*N, N);
aert = zeros(4,N);
AIL_IDX = 1;
ELE_IDX = 2;
RUD_IDX = 3;
tApply = 10;
aert(RUD_IDX, tApply:tApply+STEP_DURATION) = 15; %step for rudder


X = zeros(N,13);
aeromodel =  AeroModel(dataDir, ...
    '', ... %cfgDir
    params.Sref, ...
    params.bref, ...
    params.cref);


x0 = s.toVector();
MpropInB = zeros(3,1);
FpropInB =  zeros(3,1);
W = params.mass*params.g;
x = x0;
forces=zeros(N,3);
moments=zeros(N,3);
for i = 1:N
    
    %compute the aerodynamic F&M due to atmosphere, and control surface
    %deflections
    [vInB, wInBrad, qI2B, ned] = s.unpack();
    [a,b ]= uvwToab(vInB);
    alphaDeg = a*c.RAD2DEG; betaDeg  = b*c.RAD2DEG;
    V = vecnorm(vInB);
    
    rho = rhoFromAlt(-s.d);
   [ FaeroInB, MaeroInB] = aeromodel.getAeroFM( ...
       alphaDeg, ...
        betaDeg, ...
        V,  ...
        wInBrad, ...
        rho,...
        aert(:,i));

    if any(isnan(FaeroInB))
        fprintf("Faero == NaN at %d\n", i);
        return;
    end

   if any(isnan(MaeroInB))
        fprintf("MaeroInB == NaN at %d\n", i);
        return;
   end 
   if i == 72
        fprintf("Faero == NaN at %d\n", i);
   end
   forces(i,:)=FaeroInB;
   moments(i,:)=MaeroInB;
    FaeroInB = FaeroInB + Quaternion.rotateVectorByQuaternion(qI2B, [0,0,W]);
    s.step(  FaeroInB, FpropInB, MaeroInB, MpropInB);
    X(i,:) = s.toVector();
   

  
end

 [UVW, PQR, Q, NED] = unpackX(X);
 plotStates(time, UVW, PQR, forces, moments);

function [UVW, PQR, Q, NED] = unpackX(X)
UVW = X(:, 1:3);
PQR = X(:, 4:6);
Q = X(:, 7:10);
NED=X(:,11:13);

end
