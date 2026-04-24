classdef CalcDerivsLon < handle
    properties

        F cell = cell(6,1); %obj.F{i}(args...)
        A (6,6) double;
        B (3,6) double; % [A E T]
        c struct;
        params struct;
        alpha; %desired
        Vt;%desired
        theta;%desired
        aeroModel ; 
    end

    methods
        function self = CalcDerivsLon(self, Vt, alphaDeg, thetaDeg)
            self.A = zeros(6,6);
            self.B = zeros(3,6);
            c = getConstants(); self.c = c;
            self.params = getVehicleParams(self.c);
            self.alpha = alphaDeg*c.DEG2RAD;
            self.theta = alphathetaDegDeg*c.DEG2RAD;
            self.Vt = Vt*c.FT2M;
            self.reset();
            self.populateFunctionVector();
                        self.aeroModel =  AeroModel(dataDir, ...
                '', ... %cfgDir
                params.Sref, ...
                params.bref, ...
                params.cref);
        end

        function reset(self)
            self.A = zeros(6,6);
            self.B = zeros(3,6);
        end

        function populateFunctionVector(self)
            c = self.c;
            params = self.params;
            m = params.mass;
            Iyy = params.Iyy;

            function uDot = f1(x,u,faero,maero,fProp,mProp)
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                uDot = (faero(1) + fProp(1))/m - c.g*sin(theta) - q*wBody;
            end

            function wDot = f2(x,u,faero,maero,fProp,mProp)
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                wDot = (faero(3) + fProp(3))/m - c.g*cos(theta) - q*uBody;
            end

            function qDot = f3(x,u,faero,maero,fProp,mProp)
                qDot = (maero(2) + mProp(2))/Iyy;
            end

            function thetaDot = f4(x,u,faero,maero,fProp,mProp)
                thetaDot = f3(x,u,faero,maero,fProp,mProp);
            end

            function xDotI = f5(x,u,faero,maero,fProp,mProp)
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                theta = x(4) + f4(x,u,faero,maero,fProp,mProp) * c.dt;
                uBody = uBody + f1(x,u,faero,maero,fProp,mProp) * c.dt;
                wBody = wBody + f2(x,u,faero,maero,fProp,mProp) * c.dt;
                xDotI = uBody*cos(theta) + wBody*sin(theta);
            end

            function zDotI = f6(x,u,faero,maero,fProp,mProp)
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                theta = x(4) + f4(x,u,faero,maero,fProp,mProp) * c.dt;
                uBody = uBody + f1(x,u,faero,maero,fProp,mProp) * c.dt;
                wBody = wBody + f2(x,u,faero,maero,fProp,mProp) * c.dt;
                zDotI = -uBody*sin(theta) + wBody*cos(theta);
            end

            self.F = {@f1; @f2; @f3; @f4; @f5; @f6};
        end

        function xDotLon = calcDerivs(self, x, u, faero, maero, fProp, mProp)
            c = self.c;
            m = self.params.mass;
            Iyy = self.params.Iyy;
            [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);

            xDotLon = zeros(6,1);
            xDotLon(1) = (faero(1) + fProp(1))/m - c.g*sin(theta) - q*wBody;
            xDotLon(2) = (faero(3) + fProp(3))/m - c.g*cos(theta) - q*uBody;
            xDotLon(3) = (maero(2) + mProp(2))/Iyy;
            xDotLon(4) = xDotLon(3);

            uBody = x(1) + xDotLon(1) * c.dt;
            wBody = x(2) + xDotLon(2) * c.dt;
            theta = x(4) + xDotLon(4) * c.dt;

            xDotLon(5) = uBody*cos(theta) + wBody*sin(theta);
            xDotLon(6) = -uBody*sin(theta) + wBody*cos(theta);
        end
    end
end
