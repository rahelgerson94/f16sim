function tests = testAeroModelAI
tests = functiontests(localfunctions);
end

function setupOnce(~)
matlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(matlabRoot);
addpath(fullfile(matlabRoot, 'AeroModel'));
addpath(fullfile(matlabRoot, 'utilities'));
addpath(fullfile(matlabRoot, 'KinematicMath'));
end

function testCoeffContainersDimensionalize(testCase)
forceCoeffs = AeroBFC();
forceCoeffs.Cx = 1.0;
forceCoeffs.Cy = -2.0;
forceCoeffs.Cz = 0.5;

forces = forceCoeffs.dimensionalize(10.0, 20.0, 1.225);
qBar = 0.5 * 1.225 * 20.0^2;

verifyEqual(testCase, forces(1), qBar * 10.0 * 1.0, "AbsTol", 1e-12);
verifyEqual(testCase, forces(2), qBar * 10.0 * -2.0, "AbsTol", 1e-12);
verifyEqual(testCase, forces(3), qBar * 10.0 * 0.5, "AbsTol", 1e-12);

momentCoeffs = AeroBMC();
momentCoeffs.Cl = 0.1;
momentCoeffs.Cm = -0.2;
momentCoeffs.Cn = 0.3;

moments = momentCoeffs.dimensionalize(10.0, 30.0, 5.0, 20.0, 1.225);

verifyEqual(testCase, moments(1), qBar * 10.0 * 30.0 * 0.1, "AbsTol", 1e-12);
verifyEqual(testCase, moments(2), qBar * 10.0 * 5.0 * -0.2, "AbsTol", 1e-12);
verifyEqual(testCase, moments(3), qBar * 10.0 * 30.0 * 0.3, "AbsTol", 1e-12);
end

function testAeroModelConstructorAndFM(testCase)
matlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dataDir = fullfile(matlabRoot, 'AeroModel', 'data');
paramsPath = fullfile(matlabRoot, 'common', 'getVehicleParams.m');

model = AeroModel(dataDir, paramsPath);
[forces, moments] = model.getAeroFM(5.0, 0.0, 150.0, [0.0; 0.0; 0.0], 1.225, [0.0; 0.0; 0.0; 0.5]);

verifyEqual(testCase, size(forces), [3, 1]);
verifyEqual(testCase, size(moments), [3, 1]);
verifyTrue(testCase, all(isfinite([forces; moments])));
end
