classdef CalcDerivsLon < handle
    properties

        F cell = cell(6,1); %obj.F{i}(args...)
        A (6,6) double;
        B (3,6) double; % [A E T]
        c struct;
        params struct;
        
        aeroModel ; 
        deltaXlon (6,1) double;
        ue (2,1) double;  %equilibrium control vector
        xe (6,1) double; %equilibrium state vector
        x (6,1) double;  %current state vector
        u;
        w;
        q;
        theta;
        xI;
        zI;
        alpha;
    end

    methods
        function self = CalcDerivsLon(xe, ue)
            self.A = zeros(6,6);
            self.B = zeros(3,6);
            c = getConstants(); self.c = c;
            self.params = getVehicleParams(self.c);
            
            self.reset();
            self.populateFunctionVector();
            self.aeroModel =  AeroModel(fullfile(fileparts(mfilename('fullpath')), '..', 'AeroModel', 'data'), ...
                '', ... 
                self.params.Sref, ...
                self.params.bref, ...
                self.params.cref);

            self.deltaXlon = zeros(6,1);           
            self.ue = ue;
            self.xe = xe;

            self.u = xe(1);
            self.w = xe(2);
            self.q = xe(3);
            self.theta = xe(4);
            self.xI = xe(5);
            self.zI = xe(6);
            self.x = xe;
            
        end
        function setInitialState(self, xe)
            self.xe = xe;
            self.x = xe;
        end
        function reset(self)
            self.A = zeros(6,6);
            self.B = zeros(3,6);
        end
        function  [ FaeroInB, MaeroInB]  = getAeroFM(self, x,u)
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                vInB = [uBody; 0;wBody];
                [a,b ]= uvwToab(vInB);
                alphaDeg = a*self.c.RAD2DEG; 
                V = vecnorm(vInB);
                
                rho = rhoFromAlt(-d);
               [ FaeroInB, MaeroInB] = self.aeroModel.getAeroFM( ...
                   alphaDeg, ...
                    0, ...
                    V,  ...
                    [0,q,0], ...
                    rho,...
                    [0; u(1); 0; u(2) ]);
        end
        function populateFunctionVector(self)
            c = self.c;
            
            m = self.params.mass;
            Iyy = self.params.Iyy;

            
            function uDot = f1(x, u)
                fProp = zeros(3,1);
                mProp = zeros(3,1);
                 [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
               [ FaeroInB, MaeroInB] = self.getAeroFM(x,u);
                uDot = (FaeroInB(1) + fProp(1))/m - c.g*sin(theta) - q*wBody;
            end

            function wDot = f2(x,u)
                fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                [ FaeroInB, MaeroInB] = self.getAeroFM(x,u);
                wDot = (FaeroInB(3) + fProp(3))/m + c.g*cos(theta) + q*uBody;
            end

            function qDot = f3(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                [ FaeroInB, MaeroInB] = self.getAeroFM(x,u);
                qDot = (MaeroInB(2) + mProp(2))/Iyy;
            end

            function thetaDot = f4(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                [ FaeroInB, MaeroInB] = self.getAeroFM(x,u);
                thetaDot = q;
            end

            function xDotI = f5(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                [ FaeroInB, MaeroInB] = self.getAeroFM(x,u);
                xDotI = uBody*cos(theta) + wBody*sin(theta);
            end

            function zDotI = f6(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                [ FaeroInB, MaeroInB] = self.getAeroFM(x,u);
                zDotI = -uBody*sin(theta) + wBody*cos(theta);
            end

            self.F = {@f1; @f2; @f3; @f4; @f5; @f6};
        end

        function xDotLon = calcDerivs(self, x, u_)
            c = self.c;
            m = self.params.mass;
            Iyy = self.params.Iyy;
            [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
            [FaeroInB, MaeroInB] = self.getAeroFM(x, u_);
            fProp = zeros(3,1);
            mProp = zeros(3,1);
            xDotLon = zeros(6,1);
            xDotLon(1) = (FaeroInB(1) + fProp(1))/m - c.g*sin(theta) - q*wBody;
            xDotLon(2) = (FaeroInB(3) + fProp(3))/m + c.g*cos(theta) + q*uBody;
            xDotLon(3) = (MaeroInB(2) + mProp(2))/Iyy;
            xDotLon(4) = q ;

            xDotLon(5) = uBody*cos(theta) + wBody*sin(theta);
            xDotLon(6) = -uBody*sin(theta) + wBody*cos(theta);
        end
        function update(self, u)
            xDot = self.calcDerivs(self.x, u);
            self.x = self.x + xDot*self.c.dt;
        end
        function x = getState(self)
            x = self.x;
        end
        function setDeltaVariables(self, deltaXlon)
            self.deltaXlon = deltaXlon;
        end

        function populateA(self)
            deltaX = zeros(6,1);
            for i = 1: length(self.F)
                for j = 1:length(self.xe)
                    deltaX(j) = self.deltaXlon(j);
                    
                    fi = self.F{i};
                    n = fi(self.xe +deltaX(j) , self.ue ) - fi(self.xe - deltaX(j) , self.ue );
                    self.A(i,j) = n/(2*self.deltaXlon(j));
                end
            end 
        end
    end %methods
end %classdef
