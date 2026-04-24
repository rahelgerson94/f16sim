
matlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dataDir = fullfile(matlabRoot, 'AeroModel', 'data');
addpath(matlabRoot);
addpath(fullfile(matlabRoot, 'AeroModel'));
addpath(fullfile(matlabRoot, 'utilities'));
addpath(fullfile(matlabRoot, 'KinematicMath'));
c = getConstants();
params = getVehicleParams(c);
aeromodel = AeroModel(dataDir, '', params.Sref, params.bref, params.cref);
