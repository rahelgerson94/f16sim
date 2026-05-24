prp = fileparts(fileparts(mfilename('fullpath')));
matlabRoot = [prp '/matlab'];
dataDir = fullfile(prp, 'matlab/AeroModel', 'data');
addpath(prp);
addpath(fullfile(prp, 'matlab/AeroModel'));
addpath(fullfile(prp, 'matlab/tools'));
addpath(fullfile(prp, 'matlab/KinematicMath'));
addpath(fullfile(prp, 'matlab/LinearAnalysis'));
addpath(fullfile(prp, 'matlab/common'));