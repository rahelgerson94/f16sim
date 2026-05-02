function [vInW] = body2wind(vInB, alpha, beta)

    ca = cos(alpha);
    sa = sin(alpha);
    B2S = [ ca 0   sa;
                          0   1 0 
                         -sa  0 ca];

     cb = cos(beta);
    sb= sin(beta);


    %TODO: VERIFY THIS IS CORRECT
    S2W =  [ cb 0   -sb;
                  0   1 0 
                  sb 0 cb];

vInW = S2W * B2S * vInB;
end
