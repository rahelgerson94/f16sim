classdef AeroModel < handle
    properties
        tabledir
        cfgPath

        %refence quantities
        Sref (1,1) double = 0
        Bref (1,1) double = 0
        cref (1,1) double = 0
        xCgRef (1,1) double = 0
        %aero state
        mach (1,1) double = 0
        altitude (1,1) double = 0
        alpha (1,1) double = 0
        beta (1,1) double = 0
        qBar (1,1) double = 0
        cg  (3,1) double = zeros(3,1)
        Re (1,1) double = 0
        controlVec (4,1) double = zeros(4,1) %a,e,r,t

        % %damping derivatives
        % clp (1,1) double = 0
        % cmq (1,1) double = 0
        % cnr (1,1) double = 0
        %body angular velocity
        pqr (3,1) double = zeros(3,1)

        %body axes coefficients
        CfTotal (1,1) AeroBFC = AeroBFC() % Cx, Cy, Cz
        CmTotal (1,1) AeroBMC = AeroBMC() % Cl, Cm, Cn
        CmDamping (1,1) AeroBMC = AeroBMC() % Clp, Cmq, Cnr
        
        % Force coefficients
        Cx
        Cy
        Cz
        
        % Moment coefficients
        Cl
        Cm
        Cn
        
        % Leading edge influences
        Cx_lef;
        Cy_lef;
        Cz_lef;
        
        Cl_lef;
        Cm_lef;
        Cn_lef;
        
        % Stability derivatives
        Cxq;
        Cyp;
        Czq;
        Cmq;
        
        Cyr;
        Cnr;
        
        Cnp;
        Clp;
        Clr;
        
        deltaCxq_lef;
        deltaCyr_lef;
        deltaCyp_lef;
        
        deltaCzq_lef;
        deltaClr_lef;
        deltaClp_lef;
        
        deltaCmq_lef;
        deltaCnr_lef;
        deltaCnp_lef;
        
        % Other data
        Cy_r30;
        Cn_r30;
        Cl_r30;
        
        Cy_a20;
        Cy_a20_lef;
        
        Cn_a20;
        Cn_a20_lef;
        
        Cl_a20;
        Cl_a20_lef;
        
        deltaCnbeta;
        deltaClbeta;
        deltaCm;
        
        eta_el

        alpha1range;
        alpha2range;
        beta1range;
        dh1range;
        dh2range;
    end

    methods
        function self = AeroModel(tabledir, cfgPath, Sref, bref, cref)
            self.tabledir = tabledir;
            self.cfgPath = cfgPath;
            self.Sref = Sref;
            self.Bref = bref;
            self.cref = cref;
            self.xCgRef = 0.35*self.cref;
            self.bindCoefficients();
        end

        function bindCoefficients(self)
            function fcn = createAeroFunction(data, varargin)
                nIndepVars = length(varargin);
                gridLengths = zeros(1, nIndepVars);

                for i = 1:nIndepVars
                    gridLengths(i) = length(varargin{i});
                end

                gridVectors = varargin;
                values = data;
                if nIndepVars > 1
                    values = reshape(data, gridLengths);
                end

                fcn = griddedInterpolant(gridVectors, values, 'spline', 'none');
            end

            h5File = fullfile(self.tabledir, 'F16AeroData.h5');

            % Read independent variables
            alpha1 = h5read(h5File, '/alpha1');
            alpha2 = h5read(h5File, '/alpha2');
            beta1  = h5read(h5File, '/beta1');
            dh1    = h5read(h5File, '/dh1');
            dh2    = h5read(h5File, '/dh2');
            
            % Force coefficients
            self.Cx = createAeroFunction(h5read(h5File,'/_Cx'),alpha1,beta1,dh1);
            self.Cy = createAeroFunction(h5read(h5File,'/_Cy'),alpha1,beta1);
            self.Cz = createAeroFunction(h5read(h5File,'/_Cz'),alpha1,beta1,dh1);
            
            % Moment coefficients
            self.Cl = createAeroFunction(h5read(h5File,'/_Cl'),alpha1,beta1,dh2);
            self.Cm = createAeroFunction(h5read(h5File,'/_Cm'),alpha1,beta1,dh1);
            self.Cn = createAeroFunction(h5read(h5File,'/_Cn'),alpha1,beta1,dh2);
            
            % Leading edge influences
            self.Cx_lef = createAeroFunction(h5read(h5File,'/_Cx_lef'),alpha2,beta1);
            self.Cy_lef = createAeroFunction(h5read(h5File,'/_Cy_lef'),alpha2,beta1);
            self.Cz_lef = createAeroFunction(h5read(h5File,'/_Cz_lef'),alpha2,beta1);
            
            self.Cl_lef = createAeroFunction(h5read(h5File,'/_Cl_lef'),alpha2,beta1);
            self.Cm_lef = createAeroFunction(h5read(h5File,'/_Cm_lef'),alpha2,beta1);
            self.Cn_lef = createAeroFunction(h5read(h5File,'/_Cn_lef'),alpha2,beta1);
            
            % Stability derivatives
            self.Cxq = createAeroFunction(h5read(h5File,'/_Cxq'),alpha1);
            self.Cyp = createAeroFunction(h5read(h5File,'/_Cyp'),alpha1);
            self.Czq = createAeroFunction(h5read(h5File,'/_Czq'),alpha1);
            self.Cmq = createAeroFunction(h5read(h5File,'/_Cmq'),alpha1);
            
            self.Cyr = createAeroFunction(h5read(h5File,'/_Cyr'),alpha1);
            self.Cnr = createAeroFunction(h5read(h5File,'/_Cnr'),alpha1);
            
            self.Cnp = createAeroFunction(h5read(h5File,'/_Cnp'),alpha1);
            self.Clp = createAeroFunction(h5read(h5File,'/_Clp'),alpha1);
            self.Clr = createAeroFunction(h5read(h5File,'/_Clr'),alpha1);
            
            self.deltaCxq_lef = createAeroFunction(h5read(h5File,'/_deltaCxq_lef'),alpha2);
            self.deltaCyr_lef = createAeroFunction(h5read(h5File,'/_deltaCyr_lef'),alpha2);
            self.deltaCyp_lef = createAeroFunction(h5read(h5File,'/_deltaCyp_lef'),alpha2);
            
            self.deltaCzq_lef = createAeroFunction(h5read(h5File,'/_deltaCzq_lef'),alpha2);
            self.deltaClr_lef = createAeroFunction(h5read(h5File,'/_deltaClr_lef'),alpha2);
            self.deltaClp_lef = createAeroFunction(h5read(h5File,'/_deltaClp_lef'),alpha2);
            
            self.deltaCmq_lef = createAeroFunction(h5read(h5File,'/_deltaCmq_lef'),alpha2);
            self.deltaCnr_lef = createAeroFunction(h5read(h5File,'/_deltaCnr_lef'),alpha2);
            self.deltaCnp_lef = createAeroFunction(h5read(h5File,'/_deltaCnp_lef'),alpha2);
            
            % Other data
            self.Cy_r30 = createAeroFunction(h5read(h5File,'/_Cy_r30'),alpha1,beta1);
            self.Cn_r30 = createAeroFunction(h5read(h5File,'/_Cn_r30'),alpha1,beta1);
            self.Cl_r30 = createAeroFunction(h5read(h5File,'/_Cl_r30'),alpha1,beta1);
            
            self.Cy_a20 = createAeroFunction(h5read(h5File,'/_Cy_a20'),alpha1,beta1);
            self.Cy_a20_lef = createAeroFunction(h5read(h5File,'/_Cy_a20_lef'),alpha2,beta1);
            
            self.Cn_a20 = createAeroFunction(h5read(h5File,'/_Cn_a20'),alpha1,beta1);
            self.Cn_a20_lef = createAeroFunction(h5read(h5File,'/_Cn_a20_lef'),alpha2,beta1);
            
            self.Cl_a20 = createAeroFunction(h5read(h5File,'/_Cl_a20'),alpha1,beta1);
            self.Cl_a20_lef = createAeroFunction(h5read(h5File,'/_Cl_a20_lef'),alpha2,beta1);
            
            self.deltaCnbeta = createAeroFunction(h5read(h5File,'/_deltaCnbeta'),alpha1);
            self.deltaClbeta = createAeroFunction(h5read(h5File,'/_deltaClbeta'),alpha1);
            self.deltaCm = createAeroFunction(h5read(h5File,'/_deltaCm'),alpha1);
            
            self.eta_el = createAeroFunction(h5read(h5File,'/_eta_el'),dh1);
            self.alpha1range=alpha1;
            self.alpha2range=alpha2;
            self.beta1range=beta1;
            self.dh1range = dh1;
            self.dh2range=dh2;

        end

        function [alphaDeg, betaDeg, a, e, r] = checkInputs(self, alphaDeg, betaDeg, a, e, r)
            alphaTmp = alphaDeg;
            betaTmp = betaDeg;
            eleTmp = e;
            ailTmp = a;
            rudTmp = r;

            if alphaDeg < min(self.alpha1range) || alphaDeg > max(self.alpha1range)
                alphaDeg = clamp(alphaDeg, min(self.alpha1range), max(self.alpha1range));
                %warning("alphaDeg exceeds range (%.2f). clamping ...\n", alphaTmp);
            end

            if betaDeg < min(self.beta1range) || betaDeg > max(self.beta1range)
                betaDeg = clamp(betaDeg, min(self.beta1range), max(self.beta1range));
                %warning("betaDeg exceeds range (%.2f). clamping ...\n", betaTmp);
            end

            if e < min(self.dh1range) || e > max(self.dh1range)
                e = clamp(e, min(self.dh1range), max(self.dh1range));
                %warning("ele exceeds range (%.2f). clamping ...\n", eleTmp);
            end

            aMin = 0;
            aMax = 30;
            if a < aMin || a > aMax
                a = clamp(a, aMin, aMax);
                %warning("ail exceeds range (%.2f). clamping ...\n", ailTmp);
            end

            rMin = 0;
            rMax = 30;
            if r < rMin || r > rMax
                r = clamp(r, rMin, rMax);
                %warning("rud exceeds range (%.2f). clamping ...\n", rudTmp);
            end
        end
        function updateCoeffs(self, alphaDeg, betaDeg, V, wB_rad, rho, aert)
            
            aileron = aert(1);
            ele = aert(2);
            rudder = aert(3);
            throttle = aert(4);
            [alphaDeg, betaDeg, aileron, ele, rudder] = self.checkInputs(alphaDeg, betaDeg, aileron, ele, rudder);
            self.alpha = alphaDeg;
            self.beta = betaDeg;
            self.controlVec = [aileron; ele; rudder; aert(4)];
            self.pqr = wB_rad(:);

            p = wB_rad(1);
            q = wB_rad(2);
            r = wB_rad(3);
            self.qBar = 0.5 * rho * V^2;

            b2V = self.Bref  / (2 * V);
            c2V = self.cref / (2 * V);
            xcgRef = self.xCgRef;
           

            cmDamping = AeroBMC();
            cmDamping.Cl = b2V * (p * self.Clp(alphaDeg) +  self.Clr(alphaDeg)*r);
            cmDamping.Cm = c2V * q * self.Cmq(alphaDeg);
            cmDamping.Cn = b2V *( r *  self.Cnp(alphaDeg) + r * self.Cnr(alphaDeg));

            cfTotal = AeroBFC();
            
            cy0=self.Cy(alphaDeg, betaDeg);
           
            cfTotal.Cx = self.Cx(alphaDeg, betaDeg, ele) + c2V * q* self.Cxq(alphaDeg);
            cfTotal.Cy =cy0 ...
                + b2V * (self.Cyp(alphaDeg)*p +  self.Cyr(alphaDeg)*r) ...
                + self.scaleFiniteDeflection(self.Cy_a20(alphaDeg, betaDeg), cy0, aileron, 20) ...
                + self.scaleFiniteDeflection(self.Cy_r30(alphaDeg, betaDeg), cy0, rudder, 30) ;
            cfTotal.Cz = self.Cz(alphaDeg, betaDeg, ele) +  (c2V * q * self.Czq(alphaDeg));
            cmTotal = AeroBMC();
            cmTotal.Cl = self.Cl(alphaDeg, betaDeg, ele) ...
                + cmDamping.Cl ...
                + self.deltaClbeta(alphaDeg) * betaDeg...            
                + self.scaleFiniteDeflection(self.Cl_a20(alphaDeg, betaDeg), self.Cl(alphaDeg, betaDeg, 0), aileron, 20) ...
                + self.scaleFiniteDeflection(self.Cl_r30(alphaDeg, betaDeg), self.Cl(alphaDeg, betaDeg, 0), rudder, 30) ;
            cmTotal.Cm = self.Cm(alphaDeg, betaDeg, ele) * self.eta_el(ele) ...
                + cfTotal.Cz * (xcgRef) ...
                + self.deltaCm(alphaDeg) ...
                + cmDamping.Cm;
            cmTotal.Cn = self.Cn(alphaDeg, betaDeg, ele) ...
                + cmDamping.Cn ...
                - cfTotal.Cy *(xcgRef) ...
                + self.deltaCnbeta(alphaDeg) * betaDeg...
                + self.scaleFiniteDeflection(self.Cn_a20(alphaDeg, betaDeg), self.Cn(alphaDeg, betaDeg, 0), aileron, 20) ...
                + self.scaleFiniteDeflection(self.Cn_r30(alphaDeg, betaDeg), self.Cn(alphaDeg, betaDeg, 0), rudder, 30);
            self.CfTotal = cfTotal;
            self.CmDamping = cmDamping;
            self.CmTotal = cmTotal ;
        end

        function [F,M] = getAeroFM(self, alphaDeg, betaDeg, V,  wB_rad, rho, aert)
            self.updateCoeffs(alphaDeg, betaDeg, V,wB_rad, rho, aert);

            F = self.CfTotal.dimensionalize(self.Sref, V, rho);
            M = self.CmTotal.dimensionalize(self.Sref, self.Bref, self.cref, V, rho);
        end
    end %methods

    methods (Static, Access = private)
        function deltaCoeff = scaleFiniteDeflection(fullDeflectionCoeff, baselineCoeff, deflectionDeg, referenceDeflectionDeg)
            deltaCoeff = (fullDeflectionCoeff - baselineCoeff) * (deflectionDeg / referenceDeflectionDeg);
        end
    end

end %classdef
