classdef AeroBFC
    properties
        Cx;
        Cy;
        Cz;
    end

    methods
        function self = AeroBFC()
            self.Cx = 0;
            self.Cy = 0;
            self.Cz = 0;
        end

        function aeroFinB = dimensionalize(self, sref, Vmps, rho)
            %aeroBF = AeroBF();
            aeroFinB = zeros(3,1);
            qBar = 0.5 * rho * Vmps^2;
            aeroFinB(1) = qBar * sref * self.Cx;
            aeroFinB(2) = qBar * sref * self.Cy;
            aeroFinB(3) = qBar * sref * self.Cz;
        end
    
          

    end
end
