classdef CalcDerivsLon < handle
    properties
        nStates = 6; nStatesLon = 4;
        nControls = 2;
        FinW cell = cell(6,1); %obj.F{i}(args...)
        FinB cell = cell(6,1); %obj.F{i}(args...)
        A (6,6) double;
        B (6,2) double; % [ E T]
        c struct;
        params struct;
        
        aeroModel ; 
        deltaXwind (6,1) double;
        deltaUe (2,1) double;
        ue (2,1) double;  %equilibrium control vector
        xeWind (6,1) double; %equilibrium state vector
        xWind (6,1) double;  %current state vector
        Vt;
        alpha;
        theta; 
        q;
        xI;
        zI;
        VT_IDX = 1; 
        TH_IDX =2;
        ALF_IDX = 3; 
        Q_IDX=4; 
        
        THROTTLE_IDX=2; ELEVATOR_IDX=1;

        %fudged up number per the NASA report on pg 93,
        % for an alt of 10,000 ft and mach 0.2
        MAX_THRUST = 16860; %lbf 
    end

    methods
        function self = CalcDerivsLon(xeWind, ue)
            self.nStates = 6;
            self.A = zeros(self.nStates, self.nStates);
            self.B = zeros(self.nStates, self.nControls);
            c = getConstants(); self.c = c;
            self.params = getVehicleParams(self.c);
            
            self.reset();
            
            self.aeroModel =  AeroModel(fullfile(fileparts(mfilename('fullpath')), '..', 'AeroModel', 'data'), ...
                '', ... 
                self.params.Sref, ...
                self.params.bref, ...
                self.params.cref);

            
            self.ue = ue; %[del_e, del_t]
            
            self.toSI(xeWind);
            self.xWind = self.xeWind;
            self.deltaUe = 0.3*ue;
            self.deltaXwind = 0.3*self.xeWind; 
            self.checkXlon();
            self.checkDeltaUe()
            self.populateFunctionVectorInW();

            self.Vt = self.xeWind(self.VT_IDX);
            self.alpha = self.xeWind(self.ALF_IDX);
            self.q = self.xeWind(self.Q_IDX);
            self.theta = self.xeWind(self.TH_IDX);
            self.xI = self.xeWind(5);
            self.zI = self.xeWind(6);
            self.MAX_THRUST = self.MAX_THRUST*c.LBF2N;

            
        end
        function checkXlon(self)
            SMALL_ANGLE = 30; %deg
            SMALL_ANGLE_RATE = 10;%deg
            SMALL_VELOCITY=30; %ft / s
            c = self.c;
            for i =1:self.nStatesLon
                if self.deltaXwind(i) == 0
                    switch i
                        case 1
                            self.deltaXwind(i) = SMALL_VELOCITY*c.FT2M; 
                        case 2
                            self.deltaXwind(i) = SMALL_ANGLE*c.DEG2RAD; 
                        case 3
                            self.deltaXwind(i)  = SMALL_ANGLE_RATE*c.DEG2RAD;
                        case 4
                            self.deltaXwind(i)  = SMALL_ANGLE*c.DEG2RAD;
                    end%switch
                end%if
                end%for
        end%funciton
        function toSI(self, xInW)
            c = self.c;
            self.xeWind(self.ALF_IDX) = xInW(self.ALF_IDX)*c.DEG2RAD;
            self.xeWind(self.TH_IDX) = xInW(self.TH_IDX)*c.DEG2RAD;
            self.xeWind(self.VT_IDX) = xInW(self.VT_IDX)*c.FT2M;
            self.xeWind(self.Q_IDX) =xInW(self.Q_IDX)*c.DEG2RAD;
        end
        function xWindEnglish = toEnglish(self, x)
            c = self.c;
            xWindEnglish = zeros(self.nStates,1);
            
            xWindEnglish(self.ALF_IDX) = x(self.ALF_IDX)*1/c.DEG2RAD;
            xWindEnglish(self.TH_IDX) = x(self.TH_IDX)*1/c.DEG2RAD;
            xWindEnglish(self.VT_IDX) = x(self.VT_IDX)*1/c.FT2M;
            xWindEnglish(self.Q_IDX) = x(self.Q_IDX)*1/c.DEG2RAD;
          end

        function printDeltaX(self)
            deltaXWindEnglish = self.toEnglish(self.deltaXwind);
            fprintf("[ΔVt  Δθ Δα  Δq] = [%.2f   %.2f     %.2f     %.2f ]\n", deltaXWindEnglish(self.VT_IDX), deltaXWindEnglish(2),deltaXWindEnglish(self.ALF_IDX), deltaXWindEnglish(self.Q_IDX));
        end
        function printXe(self)
                xeWindEnglish = self.toEnglish(self.xeWind);
                fprintf("[Vt θ α  q] = [%.2f   %.2f     %.2f     %.2f ]\n", xeWindEnglish(self.VT_IDX), xeWindEnglish(2), xeWindEnglish(self.ALF_IDX),xeWindEnglish(self.Q_IDX));

        end
        function printUe(self)
                fprintf("[δ_e δ_t] = [%.2f     %.2f ]\n", self.ue(self.ELEVATOR_IDX), self.ue(self.THROTTLE_IDX));

        end
        function printDeltaUe(self)
                fprintf("[Δδ_e    Δδ_t] = [%.2f      %.2f  ]\n", self.deltaUe(self.ELEVATOR_IDX), self.deltaUe(self.THROTTLE_IDX));

        end
        function checkDeltaUe(self)
            if self.deltaUe(self.THROTTLE_IDX) == 0
                self.deltaUe(self.THROTTLE_IDX) = 0.3;
            end
            if self.deltaUe(self.ELEVATOR_IDX) == 0
                self.deltaUe(self.ELEVATOR_IDX) = 0.3*self.params.limits.cs.eSym;
            end
        end
       
        function setInitialState(self, xe)
            self.xeWind = xe;
            self.xWind = xe;
        end
        function reset(self)
            self.A = zeros(self.nStates, self.nStates);
            self.B = zeros(self.nStates, self.nControls);
        end
        function  [ FaeroInB, MaeroInB]  = getAeroFMInB(self, x,eleThr)
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLonInB(x);
                vInB = [uBody; 0;wBody];
                [~,a,b ]= uvw2mab(vInB);
                alphaDeg = a*self.c.RAD2DEG; 
                V = vecnorm(vInB);
                
                rho = rhoFromAlt(-d);
               [ FaeroInB, MaeroInB] = self.aeroModel.getAeroFM( ...
                   alphaDeg, ...
                    0, ...
                    V,  ...
                    [0,q,0], ...
                    rho,...
                    [0; eleThr(1); 0; eleThr(2) ]);
        end

        function  [ FaeroInW, MaeroInW]  = getAeroFMInW(self, xInW, eleThr)
                [Vt, a, q, theta, n,d] = unpackStateVectorLonInW(xInW);
                alphaDeg = a*self.c.RAD2DEG; 
                [u,v,w] = mab2uvw(Vt,a,0);
                rho = rhoFromAlt(-d);
               [ FaeroInB, MaeroInB] = self.aeroModel.getAeroFM( ...
                   alphaDeg, ...
                    0, ...
                    Vt,  ...
                    [0,q,0], ...
                    rho,...
                     [0; eleThr(1); 0; eleThr(2) ]);
            FaeroInW = body2wind(FaeroInB, a, 0); 
            MaeroInW = body2wind(MaeroInB, a, 0);
           end

        function populateFunctionVectorInB(self)
            c = self.c;
            
            m = self.params.mass;
            Iyy = self.params.Iyy;

            
            function uDot = f1(x, u)
                fProp = zeros(3,1);
                mProp = zeros(3,1);
                 [uBody,wBody,q,theta,n,d] = unpackStateVectorLonInB(x);
               [ FaeroInB, MaeroInB] = self.getAeroFMInB(x,u);
                uDot = (FaeroInB(1) + fProp(1))/m - c.g*sin(theta) - q*wBody;
            end

            function wDot = f2(x,u)
                fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLonInB(x);
                [ FaeroInB, MaeroInB] = self.getAeroFMInB(x,u);
                wDot = (FaeroInB(3) + fProp(3))/m + c.g*cos(theta) + q*uBody;
            end

            function qDot = f3(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLonInB(x);
                [ FaeroInB, MaeroInB] = self.getAeroFMInB(x,u);
                qDot = (MaeroInB(2) + mProp(2))/Iyy;
            end

            function thetaDot = f4(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLonInB(x);
                
                thetaDot = q;
            end

            function xDotI = f5(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLonInB(x);
                
                xDotI = uBody*cos(theta) + wBody*sin(theta); 
            end

            function zDotI = f6(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [uBody,wBody,q,theta,n,d] = unpackStateVectorLonInB(x);
                
                zDotI = -uBody*sin(theta) + wBody*cos(theta);
            end

            self.FinB = {@f1; @f2; @f3; @f4; @f5; @f6};
        end

        function populateFunctionVectorInW(self)
            c = self.c;
            
            m = self.params.mass;
            Iyy = self.params.Iyy;

            
            function VtDot = f1(x, u)
                 [Vt,alpha,q,theta,n,d] = unpackStateVectorLonInW(x);
               [ FaeroInW, MaeroInW] = self.getAeroFMInW(x,u);
                
               
                Ft = self.MAX_THRUST*u(self.THROTTLE_IDX);           
                VtDot = Ft*cos(alpha)/m- FaeroInW(c.DRAG_IDX)/m - c.g*sin(theta - alpha);
                
            end

            function alphaDot = f2(x,u)
                [Vt,alpha,q,theta,n,d] = unpackStateVectorLonInW(x);
               [ FaeroInW, MaeroInW] = self.getAeroFMInW(x,u);
               Ft = self.MAX_THRUST*u(self.THROTTLE_IDX);
                alphaDot = 1/(m*Vt)*(-Ft*sin(alpha) ...
                                                   -FaeroInW(c.LIFT_IDX))...
                                                    +c.g*cos(theta-alpha)/Vt...
                                                    + q;
            end

            function qDot = f3(x,u)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [Vt,alpha,q,theta,n,d] = unpackStateVectorLonInW(x);
               [ FaeroInW, MaeroInW] = self.getAeroFMInW(x,u);
                %since yb is the rot axis from body to wind, 
                % the 2nd component of a vector in the body frame remains 
                %  unchanged, so no need to transform M into body
                qDot = (MaeroInW(2) + mProp(2))/Iyy;
            end

            function thetaDot = f4(x,u)

                [Vt,alpha,q,theta,n,d] = unpackStateVectorLonInW(x);
                
                thetaDot = q;
            end

            function xDotI = f5(x,aert)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [Vt,a,q,theta,n,d] = unpackStateVectorLonInW(x);
                [u,v,w] = mab2uvw(Vt, a, 0);
                xDotI = u*cos(theta) + w*sin(theta); 

                
            end

            function zDotI = f6(x,aert)
                 fProp = zeros(3,1);
                mProp = zeros(3,1);
                [Vt,a,q,theta,n,d] = unpackStateVectorLonInW(x);
                [u,v,w] = mab2uvw(Vt, a, 0);
                zDotI = -u*sin(theta) + w*cos(theta);
            end
            %              Vt    th  alf        q
            self.FinW = {@f1; @f4; @f2; @f3; @f5; @f6};
        end

        function xDotLon = calcDerivsInB(self, x, u_)
            c = self.c;
            m = self.params.mass;
            Iyy = self.params.Iyy;
            [uBody,wBody,q,theta,n,d] = unpackStateVectorLonInB(x);
            [FaeroInB, MaeroInB] = self.getAeroFMInB(x, u_);
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
        function update(self, u, frame)
            if frame == 'w'
                xDotInW = self.calcDerivsInB(self.xWind, u);
                self.xWind = self.xWind + xDotInW*self.c.dt;
            end
        end
        
        function updateWithFinW(self,u)
            xDot = zeros(self.nStates,1);
            for i = 1:self.nStates
                xDot(i) = self.FinW{i}(self.xWind, u);
            end
            self.xWind = self.xWind + xDot*self.c.dt;
            %self.updateWindComponents('w');
        end
        function updateWithFinB(self,u)
            xDot = zeros(self.nStates,1);
            for i = 1:self.nStates
                xDot(i) = self.FinW{i}(self.xWind, u);
            end
            self.xWind = self.xWind + xDot*self.c.dt;
                end

         function x = getStateInW(self)
            x = self.xWind;
        end
        function setDeltaVariables(self, deltaXlon)
            self.deltaXwind = deltaXlon;
        end
        function updateWindComponents(self, frame)
            %% 
             % frame: is the frame of the state vector x
             % right now its self.x, but maybe in the future this function
             % will take in self, x, frame
            %%
            if frame == strcmp(frame, 'w')
                self.Vt = self.xWind(self.VT_IDX);
                self.alpha = self.xWind(self.TH_IDX);
                self.q = self.xWind(self.Q_IDX);
                self.theta = self.xWind(self.TH_IDX);
                self.xI = self.xWind(5);
                self.zI = self.xWind(6);
            end
        end
        function populateA(self)
            
            for i = 1: self.nStatesLon
                for j = 1:self.nStatesLon
                    %reinstanitate a vector of zeroes each time, 
                    % and assign the jth element 
                    % we need it to be vector of the same size as 
                    % deltaXwind because it gets added to deltaXwind  
                    deltaXj = zeros(self.nStates,1);
                    deltaXj(j) = self.deltaXwind(j);
                    
                    
                    fi = self.FinW{i};
                    n = fi(self.xeWind +deltaXj, self.ue ) - fi(self.xeWind - deltaXj , self.ue );
                    self.A(i,j) = n/(2*self.deltaXwind(j));
                end
            end 
        end

        function populateB(self)
            for i = 1: self.nStatesLon
                for j = 1:self.nControls
                    deltaUj = zeros(self.nControls, 1);
                    deltaUj(j) = self.deltaUe(j);
                    fi = self.FinW{i};
                    n = fi(self.xeWind, self.ue +  deltaUj ) - fi(self.xeWind ,self.ue -deltaUj );
                    %fprintf("%d, %d\n", i,j);
                    self.B(i,j) = n/(2* self.deltaUe(j));
                end
            end 
        end

        function [u,w,q,theta,n,d] = getStateInB(self)
            %TODO: this is currently not functional, find out why 
            vInB = wind2body([self.xWind(self.VT_IDX);0;0], ...
                self.xWind(self.ALF_IDX), 0 );
            [u,v,w] = unpackVector3(vInB);
            q = self.xWind(self.Q_IDX);
            theta = self.xWind(self.TH_IDX);
            n = self.xWind(5);
            d = self.xWind(6);
        end
        function printAinEnglishUnits(self)
            c= self.c;
            Aenglish = zeros(6,6);
            Aenglish(:, self.VT_IDX) = self.A(:,self.VT_IDX) *c.M2FT;
            Aenglish(:, self.ALF_IDX) = self.A(:,self.ALF_IDX) *c.RAD2DEG;
            Aenglish(:, self.Q_IDX) = self.A(:,self.Q_IDX) *c.RAD2DEG;
            Aenglish(:, self.TH_IDX) = self.A(:,self.TH_IDX) *c.RAD2DEG;
            Aenglish(:, 5) = self.A(:,5) *c.M2FT;
            Aenglish(:, 6) = self.A(:,6) *c.M2FT;
            printMatrix(Aenglish(1:self.nStatesLon, 1:self.nStatesLon));
        end
    end %methods
end %classdef
