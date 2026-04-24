classdef AeroWFC %force moment coefficients
    properties
        Cd
        Cy
        Cl
    end

    methods
        function self = AeroWFC()
            self.Cd= 0;
            self.Cy = 0;
            self.Cl = 0;
        end
    end
end
