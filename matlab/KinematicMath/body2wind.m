function [vInW] = body2wind(vInB, alpha, beta)

    ca = cos(alpha);
    sa = sin(alpha);
    B2S = [ ca 0   sa;
                  0   1 0 
                -sa  0 ca];

     cb = cos(beta);
    sb= sin(beta);


    %TODO: VERIFY THIS IS CORRECT
    S2W =  [ cb     sb     0;
            -sb     cb     0
             0       0    1 ];

vInW = S2W * B2S * vInB;

end
