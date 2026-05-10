function [vInW] = wind2body(vInW, alpha, beta)
%% 
% vInW: some vector in the wind frame
%%
    ca = cos(alpha);
    sa = sin(alpha);
    B2S = [ ca 0   sa;
            0   1 0 
            -sa  0 ca];
    S2B = B2S';
    cb = cos(beta);
    sb= sin(beta);


    %TODO: VERIFY THIS IS CORRECT
    S2W =  [ cb     sb     0;
            -sb     cb     0
             0       0    1 ];
    W2S = S2W';
vInW = S2B * W2S * vInW;
end
