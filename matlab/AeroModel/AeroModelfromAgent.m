classdef AeroModel < handle
    properties
        tabledir
        cfgPath

        %refence quantities
        Sref (1,1) double = 0
        Bref (1,1) double = 0
        Cref (1,1) double = 0

        %aero state
        mach (1,1) double = 0
        altitude (1,1) double = 0
        alpha (1,1) double = 0
        beta (1,1) double = 0
        qBar (1,1) double = 0
        cg  (3,1) double = zeros(3,1)
        Re (1,1) double = 0
        controlVec (4,1) double = zeros(4,1) %a,e,r,t

        %damping derivatives
        clp (1,1) double = 0
        cmq (1,1) double = 0
        cnr (1,1) double = 0
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
    end

    methods
        function self = AeroModel(tabledir, cfgPath, Sref, bref, cref)
            self.tabledir = tabledir;
            self.cfgPath = cfgPath;
            self.Sref = Sref;
            self.Bref = bref;
            self.Cref = cref;
            self.bindCoefficients();
        end

        function bindCoefficients(self)
            function fcn = createAeroFunction(data, varargin)
                nIndepVars = length(varargin);
                gridLengths = zeros(1, nIndepVars);

                for i = 1:nIndepVars
                    gridLengths(i) = length(varargin{i});
                end

                if nIndepVars == 1
                    fcn = griddedInterpolant(varargin{:}, data, 'spline', 'none');
                else
                    fcn = griddedInterpolant(varargin{:}, reshape(data, gridLengths), 'spline', 'none');
                end
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

        end


        
        function updateCoeffs(self, alphaDeg, betaDeg, V, aert, wB_rad, rho)
            aileron = aert(1);
            ele = aert(2);
            rudder = aert(3);

            self.alpha = alphaDeg;
            self.beta = betaDeg;
            self.controlVec = aert(:);
            self.pqr = wB_rad(:);

            p = wB_rad(1);
            q = wB_rad(2);
            r = wB_rad(3);
            self.qBar = 0.5 * rho * V^2;

            qHat = self.Cref * q / (2 * V);
            pHat = self.Bref * p / (2 * V);
            rHat = self.Bref * r / (2 * V);

            cxBase = self.Cx(alphaDeg, betaDeg, ele);
            cyBase = self.Cy(alphaDeg, betaDeg);
            czBase = self.Cz(alphaDeg, betaDeg, ele);

            clBase = self.Cl(alphaDeg, betaDeg, ele);
            cmBase = self.Cm(alphaDeg, betaDeg, ele);
            cnBase = self.Cn(alphaDeg, betaDeg, ele);

            cfTotal = AeroBFC();
            cfTotal.Cx = cxBase + qHat * self.Cxq(alphaDeg);
            cfTotal.Cy = cyBase ...
                + self.scaleFiniteDeflection(self.Cy_a20(alphaDeg, betaDeg), cyBase, aileron, 20) ...
                + self.scaleFiniteDeflection(self.Cy_r30(alphaDeg, betaDeg), cyBase, rudder, 30) ...
                + pHat * self.Cyp(alphaDeg) ...
                + rHat * self.Cyr(alphaDeg);
            cfTotal.Cz = czBase + qHat * self.Czq(alphaDeg);

            cmDamping = AeroBMC();
            cmDamping.Cl = pHat * self.Clp(alphaDeg) + rHat * self.Clr(alphaDeg);
            cmDamping.Cm = qHat * self.Cmq(alphaDeg);
            cmDamping.Cn = pHat * self.Cnp(alphaDeg) + rHat * self.Cnr(alphaDeg);

            cmTotal = AeroBMC();
            cmTotal.Cl = clBase ...
                + self.scaleFiniteDeflection(self.Cl_a20(alphaDeg, betaDeg), self.Cl(alphaDeg, betaDeg, 0), aileron, 20) ...
                + self.scaleFiniteDeflection(self.Cl_r30(alphaDeg, betaDeg), self.Cl(alphaDeg, betaDeg, 0), rudder, 30) ...
                + cmDamping.Cl ...
                + self.deltaClbeta(alphaDeg) * betaDeg;
            cmTotal.Cm = cmBase * self.eta_el(ele) ...
                + self.deltaCm(alphaDeg) ...
                + cmDamping.Cm;
            cmTotal.Cn = cnBase ...
                + self.scaleFiniteDeflection(self.Cn_a20(alphaDeg, betaDeg), self.Cn(alphaDeg, betaDeg, 0), aileron, 20) ...
                + self.scaleFiniteDeflection(self.Cn_r30(alphaDeg, betaDeg), self.Cn(alphaDeg, betaDeg, 0), rudder, 30) ...
                + cmDamping.Cn ...
                + self.deltaCnbeta(alphaDeg) * betaDeg;

            self.CfTotal = cfTotal;
            self.CmDamping = cmDamping;
            self.CmTotal = cmTotal;
        end

        function [F,M] = getAeroFM(self, alphaDeg, betaDeg, aert,  V,  wB_rad, rho)
            self.updateCoeffs(alphaDeg, betaDeg, V, aert, wB_rad, rho);

            F = self.CfTotal.dimensionalize(self.Sref, V, rho);
            M = self.CmTotal.dimensionalize(self.Sref, self.Bref, self.Cref, V, rho);
        end
    end %methods

    methods (Static, Access = private)
        function deltaCoeff = scaleFiniteDeflection(fullDeflectionCoeff, baselineCoeff, deflectionDeg, referenceDeflectionDeg)
            deltaCoeff = (fullDeflectionCoeff - baselineCoeff) * (deflectionDeg / referenceDeflectionDeg);
        end
    end

end %classdef
