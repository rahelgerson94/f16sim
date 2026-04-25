%initializations

%all functions will have access to global variables. 
%not just variables in this script

%% --- GLOBALS ---
global aeroModel;
global params;
global c;
global V_FT_S;
global ALT_FT;
c=getConstants();
params = getVehicleParams(c);
V_FT_S = 502;
ALT_FT = 1000;
aeroModel =  AeroModel(dataDir, ...
    '', ... %cfgDir
    params.Sref, ...
    params.bref, ...
    params.cref);
%% initial values

initialGuess = 1;
s = State(c.dt, getVehicleParams(c));


%% ------------ BEGIN ------------
if initialGuess
    s.setInitialState(ned0 = [0; 0; -1000*c.FT2M], ...
                  vInB0 = [1000; 0; 0]*c.FT2M, ...
                  qI2B0 = [1; 0; 0; 0]);
    aert = [0 0 0 0]'; 
    Zguess = [s.toVector(); aert];
else
    S = load('zStar.mat');
    zStar = S.zStar;
    Zguess =zStar;
end

opts = optimset('TolX', 1e-10, 'MaxFunEvals', 5000,'MaxIter',10000);
%# Pass fixed trim targets while fminsearch only changes the vector Z.
[zStar, J] = fminsearch(@(Z) costStraighLevel(Z, V_FT_S, ALT_FT), Zguess, opts);
save('zStar.mat', 'zStar');


function J = costStraighLevel(Z, Vt_ft_s, alt_ft)
    global c;

    %# Split the optimization vector back into state and control vectors.
    x = Z(1:c.nStates);
    u = Z(c.nStates+1: end);
    assert(length(u)==4)

    [vInB, wInB, qI2B, ned] = unpackStateVector(x);
    V_FT_S = Vt_ft_s;  
    ALT_FT=alt_ft;
    
    xDot = getXdotFromZ(Z);
    
    [vInBdot, wInBdot, qI2Bdot, nedDot] = unpackStateVector(xDot);
    [a,beta]=uvwToab(vInB);
    eulerAngles = Quaternion.eulerAngles321FromQuaternion(qI2B);
    % define the constraints vector Q 
    Q = [vInBdot;
            wInBdot;
            qI2Bdot(2:3);
            V_FT_S - (vInB(1)^2 + vInB(3)^2)^0.5*c.M2FT ;
            vInB(2);
            eulerAngles(1); %no roll
            wInB(1) ;
            eulerAngles(3); %flying north
            ned(3)*c.M2FT + ALT_FT;
            %u(4); %assume we dont control throttle for now
            ];
    H = diag(ones(1,size(Q,1)));
    J = Q'*H*Q;
end


function xDot = getXdotFromZ(Z, params)
    global c; 
    global params;
    global aeroModel;
   
    x = Z(1:c.nStates); 
    u = Z(c.nStates+1: end);
    assert(length(u) == 4);
    [vInB, wInB, qI2B, ned] = unpackStateVector(x);
    
    [a,b ]= uvwToab(vInB);
    V = vecnorm(vInB);
    rho = rhoFromAlt(ned(3));
    alphaDeg = a*c.RAD2DEG; betaDeg  = b*c.RAD2DEG;

    [ FaeroInB, MaeroInB] = aeroModel.getAeroFM( ...
        alphaDeg, ...
        betaDeg, ...
        V,  ...
        wInB, ...
        rho,...
        u);
    W = c.g * params.mass;
    %generate an artificial lift force opposing gravity  
    FaeroInB = FaeroInB + Quaternion.rotateVectorByQuaternion(qI2B, [0,0,-W]);
    xDot = getStateDeriv(x,FaeroInB, zeros(3,1), MaeroInB, zeros(3,1), params);
end
