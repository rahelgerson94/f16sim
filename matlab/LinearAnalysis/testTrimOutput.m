%verify the solution matches our conditions...
global V_FT_S; global ALT_FT; global params; global c;

c = getConstants();
params = getVehicleParams();
paramsPath = fullfile(fileparts(mfilename('fullpath')), '..', 'common', 'getVehicleParams.m');
S = load('zStar.mat');
zStar = S.zStar;
xStar=zStar(1:c.nStates);
uStar=zStar(c.nStates+1:end);
[vInB, wInBrad, qI2B, ned] = unpackStateVector(xStar);
[Vt,a,b ]= uvw2mab(vInB);
alphaDeg = a*c.RAD2DEG; betaDeg  = b*c.RAD2DEG;
rho = rhoFromAlt(-xStar(13));
aeroModel =  AeroModel(dataDir, ...
    paramsPath);
%% initial values


%verify the model satisfied the constraints
[ FaeroInB, MaeroInB] = aeroModel.getAeroFM( ...
       alphaDeg, ...
        betaDeg, ...
        Vt,  ...
        wInBrad, ...
        rho,...
        uStar);
%generate an artifical lift force to counteract gravity
W = c.g*params.mass;
FgInB =  Quaternion.rotateVectorByQuaternion(qI2B, [0,0,-W]); 
xDotStar = getStateDeriv(xStar,FaeroInB+FgInB, zeros(3,1), MaeroInB, zeros(3,1), params);
[vInB, pqr, qI2b, ned]= unpackStateVector(zStar);

[vInBdot, pqrDot, qI2BDot, nedDot] = unpackStateVector(xDotStar);
if vecnorm(vInBdot)*c.M2FT  >= 0.1
    fprintf( "vInBdot failed\n");  
    printVecWithName(vInBdot*c.M2FT, "vInBdot");
end
if vecnorm(pqrDot)*c.RAD2DEG > .1
    fprintf( "pqrDot failed\n");
    printVecWithName(pqrDot*c.RAD2DEG, "pqrDot");
end

    eaRates = Quaternion.eulerAngles321FromQuaternion(qI2BDot) * c.RAD2DEG;
    if vecnorm(eaRates) > 0.01
    fprintf( "eulerRates failed\n");
    printVecWithName(eaRates, "euler rates");
end
if (abs(vecnorm(nedDot*c.M2FT - [V_FT_S 0 0] )))  > + 0.5 
    printVecWithName(nedDot*c.M2FT, "nedDot");
    fprintf( "u failed\n");
end
eas = Quaternion.eulerAngles321FromQuaternion(qI2B) * c.RAD2DEG;
    
printVecWithName(vInB*c.M2FT, "vInB (ft/s)");
printVecWithName(pqr*c.RAD2DEG, "pqr (deg/s)");
printVecWithName(eas, "euler angles");
printVecWithName(ned*c.M2FT, "ned (ft)");
printVecWithName(uStar, "aert");
printVecWithName([vInB(1) vInB(3) pqr(2) eas(2) ned(1)  ned(3)], "xLon");

ue = zeros(2,1);
ue(c.lon.ELE_IDX) = uStar(2);
ue(c.lon.THTL_IDX) = uStar(4);
save('generated/ue.mat', 'ue');
xeInW = zeros(4,1) ;%Vt Theta alpha q 
xeInW(c.lon.VT_IDX) = Vt;
xeInW(c.lon.ALF_IDX) = alphaDeg;
xeInW(c.lon.TH_IDX) = eas(2);
xeInW(c.lon.Q_IDX) = pqr(2);
xeInW(c.lon.N_IDX) = ned(1);
xeInW(c.lon.D_IDX) = ned(3);
save('generated/xeInW.mat', 'xeInW');