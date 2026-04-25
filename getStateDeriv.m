       function xDot = getStateDeriv(X,FaeroInB, FpropInB, MaeroInB, MpropInB, params)
        global c;
        xDot = zeros(c.nStates,1);
        [vInB, wInB, qI2B, ned] = unpackStateVector(X);
        W = c.g*params.mass;
        %gravity acts in the dwon direction, so the 3rd component is
        %positive
        FgInB =  Quaternion.rotateVectorByQuaternion(qI2B, [0,0,W])'; 
        HinB = params.I*wInB;
        xDot(1:3)= (FaeroInB(:) + FgInB(:) + FpropInB(:) )/params.mass - cross(wInB,vInB);
        xDot(4:6) = params.I \ (MaeroInB(:) + MpropInB(:) - cross(wInB, HinB));


        xDot(7:10) = Quaternion.computeDerivative(qI2B, wInB );

        qB2I = Quaternion.conjugate(qI2B)
        xDot(11:13) = Quaternion.rotateVectorByQuaternion(qB2I, ...
                vInB);
     
       end
