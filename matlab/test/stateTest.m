c = getConstants();
params = getVehicleParams(c);
dt = c.dt;
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


% initialize State and time, force, moment vectors
s = State(dt, getVehicleParams(c));
s.setInitialState(ned0 = [0; 0; -1000*c.FT2M], ...
                  vInB0 = [1000; 0; 50]*c.FT2M, ...
                  qI2B0 = [1; 0; 0; 0]);
time = linspace(0,SIM_DURATION ,SIM_DURATION_SAMPLES );
X = zeros(SIM_DURATION_SAMPLES,13);
AERT = zeros(4,SIM_DURATION_SAMPLES);
AERT(ELE_IDX, ...
    APPLICATION_START_IDX:APPLICATION_START_IDX+STEP_DURATION_SAMPLES) = 15; %degrees 

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
forces=zeros(SIM_DURATION_SAMPLES,3);
moments=zeros(SIM_DURATION_SAMPLES,3);
for i = 1:SIM_DURATION_SAMPLES
    
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
        AERT(:,i));

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
