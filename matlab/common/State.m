classdef State < handle
    properties
        
        u;
        v;
        w;

        p;
        q;
        r;
        
        qw;
        qx;
        qy;
        qz;

        n;
        e;
        d;

        dt
        params;
        nStates;
        nControls;
        c;
        aeroModel;
    end

    methods
        function self = State( dt, vehicleParams, options)
            arguments
                dt; 
                vehicleParams;
                options.dataDir = fullfile(fileparts(mfilename('fullpath')), '..', 'AeroModel', 'data');
            end
            self.dt = dt;
            self.nStates = 13;
            self.nControls = 4;
            self.params = vehicleParams;
            self.reset();
            self.c = getConstants();
            %# Use the AeroModel data directory found relative to this State class file.
            self.aeroModel =   AeroModel(options.dataDir, ...
                    '', ... %cfgDir
                    vehicleParams.Sref, ...
                    vehicleParams.bref, ...
                    vehicleParams.cref);
        end

        function reset(self)
            self.u = 0;
            self.v = 0;
            self.w = 0;
            self.p = 0;
            self.q = 0;
            self.r = 0;
            self.qw = 0;
            self.qx = 0;
            self.qy = 0;
            self.qz = 0;
            self.n = 0;
            self.e = 0;
            self.d = 0;
        end

        function setInitialState(self, options)
            arguments
                self
                options.vInB0 double = []
                options.pqr0 double = []
                 options.qI2B0 double = []
                options.ned0 double = []
            end

            if ~isempty(options.vInB0)
                assert(isvector(options.vInB0) && numel(options.vInB0) == 3, ...
                    'State:setInitialState:BadVelocityVector', ...
                    'vInB0 must be a 3-element body-velocity vector.');
                vInB0 = options.vInB0(:);
                self.u = vInB0(1);
                self.v = vInB0(2);
                self.w = vInB0(3);
            end

            if ~isempty(options.qI2B0)
                assert(isvector(options.qI2B0) && numel(options.qI2B0) == 4, ...
                    'State:setInitialState:BadQuaternion', ...
                    'qI2B0 must be a 4-element quaternion.');
                qI2B0 = options.qI2B0(:);
                qNorm = vecnorm(qI2B0);
                assert(qNorm > 0, ...
                    'State:setInitialState:ZeroQuaternion', ...
                    'qI2B0 must have nonzero norm.');
                qI2B0 = qI2B0 / qNorm;
                self.qw = qI2B0(1);
                self.qx = qI2B0(2);
                self.qy = qI2B0(3);
                self.qz = qI2B0(4);
            end

            if ~isempty(options.pqr0)
                assert(isvector(options.pqr0) && numel(options.pqr0) == 3, ...
                    'State:setInitialState:BadRateVector', ...
                    'pqr0 must be a 3-element body-rate vector.');
                pqr0 = options.pqr0(:);
                self.p = pqr0(1);
                self.q = pqr0(2);
                self.r = pqr0(3);
            end

            if ~isempty(options.ned0)
                assert(isvector(options.ned0) && numel(options.ned0) == 3, ...
                    'State:setInitialState:BadPositionVector', ...
                    'ned0 must be a 3-element NED position vector.');
                ned0 = options.ned0(:);
                self.n = ned0(1);
                self.e = ned0(2);
                self.d = ned0(3);
            end
        end

       function v = toVector(self )
            
            v = [self.u, self.v, self.w,...
                self.p, self.q, self.r,...
                self.qw, self. qx, self.qy, self.qz,...
                self.n, self.e, self.d
                ].';
       end

       function dState = getDeriv(self, FaeroInB, FpropInB, MaeroInB, MpropInB)
           dState = zeros(self.nStates,1);
           [vInB, wInB, qI2B] = self.unpack();
           if vecnorm(vInB) < 1e-12
                a = 0; b = 0;
           else
             [Vt, a,b] = uvw2mab(vInB);
           end
            W = self.c.g*self.params.mass;
           FgInB =  Quaternion.rotateVectorByQuaternion(qI2B, [0,0,W])';


           dState(1:3)= (FaeroInB(:) + FgInB(:) + FpropInB(:) )/self.params.mass - cross(wInB,vInB);
           HinB = self.params.I*wInB;
           dState(4:6) = self.params.I \ (MaeroInB(:) + MpropInB(:) - cross(wInB, HinB));


          dState(7:10) =Quaternion.computeDerivative(qI2B, wInB );
           qB2I = Quaternion.conjugate(qI2B);
           dState(11:self.nStates) = Quaternion.rotateVectorByQuaternion(qB2I, ...
                [self.u; self.v; self.w]);

       end

      % function dState = getDeriv(self, u)
      %      dState = zeros(self.nStates,1);
      %     [vInB, wInB, qI2B] = self.unpack();
      %        [Vt, a,b ]= uvw2mab(vInB);
      %       V = vecnorm(vInB);
      %       rho = rhoFromAlt(-self.d);
      %      [ FaeroInB, MaeroInB] = self.aeroModel.getAeroFM( ...
      %          a*c.RAD2DEG, ...
      %           b*c.RAD2DEG, ...
      %           V,  ...
      %           wInBrad, ...
      %           rho,...
      %           u);
      % 
      % 
      % 
      %       W = self.c.g*self.params.mass;
      %      FgInB =  Quaternion.rotateVectorByQuaternion(qI2B, [0,0,W])';
      % 
      %      FpropInB = zeros(3,1); %TODO: make engine model
      %      MpropInB = zeros(3,1);%TODO: make engine model
      %      dState(1:3)= (FaeroInB(:) + FgInB(:) + FpropInB(:) )/self.params.mass - cross(wInB,vInB);
      %      HinB = self.params.I*wInB;
      %      dState(4:6) = self.params.I \ (MaeroInB(:) + MpropInB(:) - cross(wInB, HinB));
      % 
      % 
      %       dState(7:10) =Quaternion.computeDerivative(qI2B, wInB );
      %      qB2I = Quaternion.conjugate(qI2B);
      %      dState(11:self.nStates) = Quaternion.rotateVectorByQuaternion(qB2I, ...
      %           [self.u; self.v; self.w]);
      % 
      %  end

       function [vInB, wInB, qI2B, ned] = unpack(self)
            wInB = [self.p, self.q, self.r].';
            vInB = [self.u, self.v, self.w].';
            qI2B = [self.qw, self.qx, self.qy,  self.qz].';
             ned = [self.n, self.e, self.d].';
       end

       function step( self, FaeroInB, FpropInB, MaeroInB, MpropInB)
            dState = self.getDeriv(FaeroInB, FpropInB, MaeroInB, MpropInB);
            self.u  = self.u  + dState(1)  * self.dt;
            self.v  = self.v  + dState(2)  * self.dt;
            self.w  = self.w  + dState(3)  * self.dt;
            self.p  = self.p  + dState(4)  * self.dt;
            self.q  = self.q  + dState(5)  * self.dt;
            self.r  = self.r  + dState(6)  * self.dt;
            
            self.qw = self.qw + dState(7)  * self.dt;
            self.qx = self.qx + dState(8)  * self.dt;
            self.qy = self.qy + dState(9)  * self.dt;
            self.qz = self.qz + dState(10) * self.dt;

            
            self.n = self.n + dState(11)*self.dt;
            self.e = self.e + dState(12)*self.dt;
            self.d = self.d + dState(13)*self.dt;
       end



    end


end
           
