function [u,v,w] = mab2uvw(Vt,a,b)
    
   uvw = body2wind([Vt; 0;0], a,b);

   [u,v,w] = unpackVector3(uvw);
end
