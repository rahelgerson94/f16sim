function []= plotStates(time, uvw, pqr, forces, moments)
close all;

c = getConstants();
figure;
 plotVector3(time, moments, "moments")

  plotVector3(time, forces, "forces");
  
   plotVector3(time, pqr*c.RAD2DEG, "w (deg/s)");

   plotVector3(time, uvw*c.M2FT, "uvw (ft/s)");

   V = vecnorm(uvw, 2, 2);
    alpha = atan2(uvw(:,3), uvw(:,1));
    beta = asin(uvw(:,2) ./ V);
    plot3Scalars(time, alpha, beta , V*c.M2FT , ["alpha  (deg)", "beta  (deg)" , "V (ft/s)"], "")

end
