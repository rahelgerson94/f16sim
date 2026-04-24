classdef AeroCSC %constorl surface coefficients
    properties
        Cnf %normal force
        Chm %hinge moment
        Cbm %bending moment
    end

    methods
        function self = AeroCSC()
            self.Cnf = 0;
            self.Chm = 0;
            self.Cbm = 0;
        end

        function aeroCSFM = dimensionalize(self, sref, Vmps, rho, lref)
            aeroCSFM = AeroCSFM();
            qBar = 0.5 * rho * Vmps^2;
            aeroCSFM.Cnf = qBar * sref * self.Cnf;
            aeroCSFM.Chm = qBar * sref * lref * self.Chm;
            aeroCSFM.Cbm = qBar * sref * lref * self.Cbm;
        end
    end
end
