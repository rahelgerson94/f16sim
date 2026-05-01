classdef CalcDerivsLon < handle
    properties
        nStates = 6;
        F cell = cell(6,1); %obj.F{i}(args...)
        A (6,6) double;
        B (2,6) double; % [ E T]
        c struct;
        params struct;
        
        aeroModel ; 
        deltaXlon (6,1) double;
        deltaUe (2,1) double;
        ue (2,1) double;  %equilibrium control vector
        xe (6,1) double; %equilibrium state vector
        x (6,1) double;  %current state vector
        Vt;
        alpha;
        theta; 
        q;
        xI;
        zI;
        
        THROTTLE_IDX=2; ELEVATOR_IDX=1;
    end

    methods
        function self = CalcDerivsLon(xe, ue)
            self.nStates = 6;
            self.A = zeros(self.nStates, self.nStates);
            self.B = zeros(2,self.nStates);
            c = getConstants(); self.c = c;
            self.params = getVehicleParams(self.c);
            
            self.reset();
            self.populateFunctionVector();
            self.aeroModel =  AeroModel(fullfile(fileparts(mfilename('fullpath')), '..', 'AeroModel', 'data'), ...
                '', ... 
                self.params.Sref, ...
                self.params.bref, ...
                self.params.cref);

            
            self.ue = ue; %[del_e, del_t]
            self.xe = xe;

            self.deltaUe = 0.3*ue;
            self.deltaXlon = 0.3*xe; 
            self.checkXlon();
            self.checkDeltaUe()
            self.Vt = xe(1);
            self.alpha = xe(2);
            self.q = xe(3);
            self.theta = xe(4);
            self.xI = xe(5);
            self.zI = xe(6);
            self.x = xe;
            
        end
        function checkXlon(self)
            SMALL_ANGLE = 30; %deg
            SMALL_ANGLE_RATE = 10;%deg
            SMALL_VELOCITY=30; %ft / s
            c = self.c;
            for i =1:4
                if self.deltaXlon(i) == 0
                    switch i
                        case 1
                            self.deltaXlon(i) = SMALL_VELOCITY*c.M2FT; 
                        case 2
                            self.deltaXlon(i) = SMALL_ANGLE*c.DEG2RAD; 
                        case 3
                            self.deltaXlon(i)  = SMALL_ANGLE_RATE*c.DEG2RAD;
                        case 4
                            self.deltaXlon(i)  = SMALL_ANGLE*c.DEG2RAD;
                    end%switch
                end%if
                end%for
        end%funciton

        function checkDeltaUe(self)
            if self.deltaUe(self.THROTTLE_IDX) == 0
                self.deltaUe(self.THROTTLE_IDX) = 0.3;
            end
            if self.deltaUe(self.ELEVATOR_IDX) == 0
                self.deltaUe(self.ELEVATOR_IDX) = 0.3*self.params.limits.cs.eSym;
            end
        end
       
        function setInitialState(self, xe)
            self.xe = xe;
            self.x = xe;
        end
        function reset(self)
            self.A = zeros(self.nStates, self.nStates);
            self.B = zeros(2,self.nStates);
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
                
                thetaDot = q;
            end

            function xDotI = f5(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                
                xDotI = uBody*cos(theta) + wBody*sin(theta);
            end

            function zDotI = f6(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLon(x);
                
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
            xDotLon = zeros(self.nStates,1);
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

        function updateWithF(self,u)
            xDot = zeros(self.nStates,1);
            for i = 1:self.nStates
                xDot(i) = self.F{i}(self.x, u);
            end
            self.x = self.x + xDot*self.c.dt;
        end
        function x = getState(self)
            x = self.x;
        end
        function setDeltaVariables(self, deltaXlon)
            self.deltaXlon = deltaXlon;
        end

        function populateA(self)
            deltaX = zeros(self.nStates,1);
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
