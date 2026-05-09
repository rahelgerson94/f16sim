function [xDot,FaeroInB, MaeroInB] = getXdotFromZ(Z, params, aeroModel)
    global c; 
    global params;
    global aeroModel;
 
    x = Z(1:c.nStates); 
    u = Z(c.nStates+1: end);
    assert(length(u) == 4);
    [vInB, wInB, qI2B, ned] = unpackStateVector(x);
    
    [Vt,a,b ]= uvw2mab(vInB);
    V = vecnorm(vInB);
    rho = rhoFromAlt(-ned(3));
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
