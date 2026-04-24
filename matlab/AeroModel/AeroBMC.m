classdef AeroBMC
    properties
        Cl;
        Cm;
        Cn;
    end

    methods
        function self = AeroBMC()
            self.Cl = 0;
            self.Cm = 0;
            self.Cn = 0;
        end

        function aeroMomentsInB = dimensionalize(self, sref, bref, cref, Vmps, rho)
            aeroMomentsInB =zeros(3,1);
            qBar = 0.5 * rho * Vmps^2;
            aeroMomentsInB(1) = qBar * sref * bref * self.Cl;
            aeroMomentsInB(2) = qBar * sref * cref * self.Cm;
            aeroMomentsInB(3)= qBar * sref * bref * self.Cn;
        end
    end
end
