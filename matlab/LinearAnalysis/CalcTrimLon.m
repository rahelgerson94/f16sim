classdef CalcTrimLon
     properties
        aeroModel;
        params;
        Vt;
        theta;
        alpha;
        xGuess;
        nStates ; 
    end

    methods
        function self = CalcTrimLon(self, dt,... 
            vehicleParams,...
            Vt, ...
            alphaDeg, ...
            thetaDeg,...
            xGuess)

            self.params = vehicleParams;
            self.aeroModel =  AeroModel(dataDir, ...
                    '', ... %cfgDir
                    params.Sref, ...
                    params.bref, ...
                    params.cref);
            self.xGuess = xGuess;
            self.theta = thetaDeg*c.RAD2DEG ;
            self.alphaDeg = alphaDeg*c.RAD2DEG ;
            self.Vt = Vt*c.FT2M;
            self.nStates = length(xGuess);
        end

    function [J ]= costStraighLevel(Z)
        
        self.VtFts;  
        x = Z(1:self.nStates);
        u = Z(self.nStates+1: end);
        assert(length(u)==self.nControls)
        xDot = getXdotFromZ(Z);
        [u,w,q,theta,n,d] = unpackStateVectorLon(x);
        [ud, wd, thetad, nd,dd] = unpackStateVectorLon(xDot);
        [a,b] = uvw2ab([u,0,w]);
       
        Q = [xDot;
                (u*u + v*v)^0.5 - self.Vt;
                a - self.alpha;
                theta - self.theta;
                ];
        H = diag(ones(1,size(Q,1)));
        J = Q'*H*Q;
    end

    end
end