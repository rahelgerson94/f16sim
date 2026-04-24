function [vInB, wInB, qI2B, ned] = unpackStateVectorLat(X)
    vInB = X(1:3);
    wInB = X(4:6);
    
    qI2B = X(7:10);
    ned = X(11:13);
end