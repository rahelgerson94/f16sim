function [Vt, alpha, q, theta, ] = unpackStateVectorLonInB(X)
    Vt = X(1);
    alpha = X(2);
    q = X(3);
    theta = X(4);
    n = X(5);
    d = X(6);
    
end