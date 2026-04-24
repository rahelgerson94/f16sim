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

verifyEqual(testCase, forces.Fx, qBar * 10.0 * 1.0, "AbsTol", 1e-12);
verifyEqual(testCase, forces.Fy, qBar * 10.0 * -2.0, "AbsTol", 1e-12);
verifyEqual(testCase, forces.Fz, qBar * 10.0 * 0.5, "AbsTol", 1e-12);

momentCoeffs = AeroBMC();
momentCoeffs.Cl = 0.1;
momentCoeffs.Cm = -0.2;
momentCoeffs.Cn = 0.3;

moments = momentCoeffs.dimensionalize(10.0, 30.0, 5.0, 20.0, 1.225);

verifyEqual(testCase, moments.Mx, qBar * 10.0 * 30.0 * 0.1, "AbsTol", 1e-12);
verifyEqual(testCase, moments.My, qBar * 10.0 * 5.0 * -0.2, "AbsTol", 1e-12);
verifyEqual(testCase, moments.Mz, qBar * 10.0 * 30.0 * 0.3, "AbsTol", 1e-12);
end

function testAeroModelConstructorAndFM(testCase)
matlabRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
dataDir = fullfile(matlabRoot, 'AeroModel', 'data');

model = AeroModel(dataDir, '', 300.0, 30.0, 11.32);
[forces, moments] = model.getAeroFM(5.0, 0.0, [0.0; 0.0; 0.0; 0.5], 150.0, [0.0; 0.0; 0.0], 1.225);

verifyClass(testCase, forces, 'AeroBF');
verifyClass(testCase, moments, 'AeroBM');
verifyTrue(testCase, all(isfinite([forces.Fx, forces.Fy, forces.Fz, moments.Mx, moments.My, moments.Mz])));
end
