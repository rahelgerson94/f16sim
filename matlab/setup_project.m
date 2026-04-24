prp = fileparts(fileparts(mfilename('fullpath')));
dataDir = fullfile(prp, 'matlab/AeroModel', 'data');
addpath(prp);
addpath(fullfile(prp, 'matlab/AeroModel'));
addpath(fullfile(prp, 'matlab/tools'));
addpath(fullfile(prp, 'matlab/KinematicMath'));
addpath(fullfile(prp, 'matlab/common'));