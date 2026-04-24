function aeroCoeffs = getAeroCoeffs(h5File)
% ================================
% Load Aero Coefficients
% ================================
%
% Usage:
%   aeroCoeffs = getAeroCoeffs();
%   aeroCoeffs = getAeroCoeffs('aeroCoeffs.h5');

if nargin < 1
    h5File = 'F16AeroData.h5';
end

% Read independent variables
alpha1 = h5read(h5File,'/alpha1');
alpha2 = h5read(h5File,'/alpha2');
beta1  = h5read(h5File,'/beta1');
dh1    = h5read(h5File,'/dh1');
dh2    = h5read(h5File,'/dh2');

% Force coefficients
aeroCoeffs.Cx = createAeroFunction(h5read(h5File,'/_Cx'),alpha1,beta1,dh1);
aeroCoeffs.Cy = createAeroFunction(h5read(h5File,'/_Cy'),alpha1,beta1);
aeroCoeffs.Cz = createAeroFunction(h5read(h5File,'/_Cz'),alpha1,beta1,dh1);

% Moment coefficients
aeroCoeffs.Cl = createAeroFunction(h5read(h5File,'/_Cl'),alpha1,beta1,dh2);
aeroCoeffs.Cm = createAeroFunction(h5read(h5File,'/_Cm'),alpha1,beta1,dh1);
aeroCoeffs.Cn = createAeroFunction(h5read(h5File,'/_Cn'),alpha1,beta1,dh2);

% Leading edge influences
aeroCoeffs.Cx_lef = createAeroFunction(h5read(h5File,'/_Cx_lef'),alpha2,beta1);
aeroCoeffs.Cy_lef = createAeroFunction(h5read(h5File,'/_Cy_lef'),alpha2,beta1);
aeroCoeffs.Cz_lef = createAeroFunction(h5read(h5File,'/_Cz_lef'),alpha2,beta1);

aeroCoeffs.Cl_lef = createAeroFunction(h5read(h5File,'/_Cl_lef'),alpha2,beta1);
aeroCoeffs.Cm_lef = createAeroFunction(h5read(h5File,'/_Cm_lef'),alpha2,beta1);
aeroCoeffs.Cn_lef = createAeroFunction(h5read(h5File,'/_Cn_lef'),alpha2,beta1);

% Stability derivatives
aeroCoeffs.Cxq = createAeroFunction(h5read(h5File,'/_Cxq'),alpha1);
aeroCoeffs.Cyp = createAeroFunction(h5read(h5File,'/_Cyp'),alpha1);
aeroCoeffs.Czq = createAeroFunction(h5read(h5File,'/_Czq'),alpha1);
aeroCoeffs.Cmq = createAeroFunction(h5read(h5File,'/_Cmq'),alpha1);

aeroCoeffs.Cyr = createAeroFunction(h5read(h5File,'/_Cyr'),alpha1);
aeroCoeffs.Cnr = createAeroFunction(h5read(h5File,'/_Cnr'),alpha1);

aeroCoeffs.Cnp = createAeroFunction(h5read(h5File,'/_Cnp'),alpha1);
aeroCoeffs.Clp = createAeroFunction(h5read(h5File,'/_Clp'),alpha1);
aeroCoeffs.Clr = createAeroFunction(h5read(h5File,'/_Clr'),alpha1);

aeroCoeffs.deltaCxq_lef = createAeroFunction(h5read(h5File,'/_deltaCxq_lef'),alpha2);
aeroCoeffs.deltaCyr_lef = createAeroFunction(h5read(h5File,'/_deltaCyr_lef'),alpha2);
aeroCoeffs.deltaCyp_lef = createAeroFunction(h5read(h5File,'/_deltaCyp_lef'),alpha2);

aeroCoeffs.deltaCzq_lef = createAeroFunction(h5read(h5File,'/_deltaCzq_lef'),alpha2);
aeroCoeffs.deltaClr_lef = createAeroFunction(h5read(h5File,'/_deltaClr_lef'),alpha2);
aeroCoeffs.deltaClp_lef = createAeroFunction(h5read(h5File,'/_deltaClp_lef'),alpha2);

aeroCoeffs.deltaCmq_lef = createAeroFunction(h5read(h5File,'/_deltaCmq_lef'),alpha2);
aeroCoeffs.deltaCnr_lef = createAeroFunction(h5read(h5File,'/_deltaCnr_lef'),alpha2);
aeroCoeffs.deltaCnp_lef = createAeroFunction(h5read(h5File,'/_deltaCnp_lef'),alpha2);

% Other data
aeroCoeffs.Cy_r30 = createAeroFunction(h5read(h5File,'/_Cy_r30'),alpha1,beta1);
aeroCoeffs.Cn_r30 = createAeroFunction(h5read(h5File,'/_Cn_r30'),alpha1,beta1);
aeroCoeffs.Cl_r30 = createAeroFunction(h5read(h5File,'/_Cl_r30'),alpha1,beta1);

aeroCoeffs.Cy_a20 = createAeroFunction(h5read(h5File,'/_Cy_a20'),alpha1,beta1);
aeroCoeffs.Cy_a20_lef = createAeroFunction(h5read(h5File,'/_Cy_a20_lef'),alpha2,beta1);

aeroCoeffs.Cn_a20 = createAeroFunction(h5read(h5File,'/_Cn_a20'),alpha1,beta1);
aeroCoeffs.Cn_a20_lef = createAeroFunction(h5read(h5File,'/_Cn_a20_lef'),alpha2,beta1);

aeroCoeffs.Cl_a20 = createAeroFunction(h5read(h5File,'/_Cl_a20'),alpha1,beta1);
aeroCoeffs.Cl_a20_lef = createAeroFunction(h5read(h5File,'/_Cl_a20_lef'),alpha2,beta1);

aeroCoeffs.deltaCnbeta = createAeroFunction(h5read(h5File,'/_deltaCnbeta'),alpha1);
aeroCoeffs.deltaClbeta = createAeroFunction(h5read(h5File,'/_deltaClbeta'),alpha1);
aeroCoeffs.deltaCm = createAeroFunction(h5read(h5File,'/_deltaCm'),alpha1);

aeroCoeffs.eta_el = createAeroFunction(h5read(h5File,'/_eta_el'),dh1);

end

function fcn = createAeroFunction(data,varargin)
nIndepVars = length(varargin);
L = zeros(1,nIndepVars);

for i = 1:nIndepVars
    L(i) = length(varargin{i});
end

if nIndepVars == 1
    fcn = griddedInterpolant(varargin, data, 'spline', 'none');
else
    fcn = griddedInterpolant(varargin, reshape(data, L), 'spline', 'none');
end

end