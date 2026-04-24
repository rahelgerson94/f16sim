classdef AeroWMC %wind moment coefficients
    properties
        Cl
        Cm
        Cn
    end

    methods
        function self = AeroWMC()
            self.Cl = 0;
            self.Cm = 0;
            self.Cn = 0;
        end

        function bodyMomentCoeffs = toBody(self, ~, ~)
            bodyMomentCoeffs = AeroBMC();
            bodyMomentCoeffs.Cl = self.Cl;
            bodyMomentCoeffs.Cm = self.Cm;
            bodyMomentCoeffs.Cn = self.Cn;
        end
    end
end
