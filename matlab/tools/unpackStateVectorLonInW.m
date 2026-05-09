function [Vt, alpha, q, theta, n,d] = unpackStateVectorLonInW(X)
    Vt = X(1);
    theta = X(2);
    alpha = X(3);
    q = X(4);
    n = X(5);
    d = X(6);
    
end