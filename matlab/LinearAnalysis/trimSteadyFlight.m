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
V_FT_S = 375.11;
ALT_FT = 1000;
aeroModel =  AeroModel(dataDir, ...
    '', ... %cfgDir
    params.Sref, ...
    params.bref, ...
    params.cref);
%% initial values

initialGuess = 1;

%% ------------ BEGIN ------------
if initialGuess
    X = zeros(c.nStates,1);
    X(1)=V_FT_S*c.FT2M;
    X(7) = 1; %qw = 1
    
    X(c.nStates) = -ALT_FT*c.FT2M;
    aert = [0 0 0 0]'; 
    Zguess = [X; aert];
else
    S = load('zStar.mat');
    zStar = S.zStar;
    Zguess =zStar;
end

[zStar, J] = fminsearch(@costStraighLevel, Zguess,...
    optimset('TolX', 1e-10, 'MaxFunEvals', 5000,'MaxIter',10000));
save('zStar.mat', 'zStar');


function [J ]= costStraighLevel(Z)
    global c;
    global V_FT_S; global ALT_FT;
    x = Z(1:c.nStates);
    u = Z(c.nStates+1: end);
    assert(length(u)==4)
    [~,~,xDot] = getXdotFromZ(Z);
    [vInB, wInB, qI2B, ned] = unpackStateVector(x);
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
            %wInB(1) ;
            eulerAngles(3); %flying north
            %ned(3)*c.M2FT + ALT_FT;
            %u(4); %assume we dont control throttle for now
            ];
    H = diag(ones(1,size(Q,1)));
    J = Q'*H*Q;
end



