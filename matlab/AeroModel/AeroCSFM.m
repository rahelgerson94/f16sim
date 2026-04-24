classdef AeroCSFM %constorl surface coefficients
    properties
        
        Cnf %normal force
        Chm %hinge moment
        Cbm %bending moment
    end

    methods
        function self = AeroCSFM()
            self.Cnf = 0;
            self.Chm = 0;
            self.Cbm = 0;
        end

    end
end
