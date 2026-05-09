
% Run project setup from the matlab root so this test works from any folder.
linearAnalysisDir = fileparts(mfilename('fullpath'));
matlabRoot = fullfile(linearAnalysisDir, '..');
currentDir = pwd;
cd(matlabRoot); setupProject; 
cd(fullfile(matlabRoot, 'LinearAnalysis'));
c = getConstants();
f16NominalTrim; % get xe, ue into the workspace
cd(currentDir);
paramsPath = fullfile(matlabRoot, 'common', 'getVehicleParams.m');
% Pass the params function path so CalcDerivsLon builds params internally.
test = CalcDerivsLon(xeWind, ue, paramsPath);
test.printDeltaUe();
test.printDeltaX();
dt = c.dt;

test.populateA();
test.populateB();

printMatrix(test.A(1:4, 1:4));
%test.printAinEnglishUnits();
printMatrix(test.B(1:4,:)) %B is 6,2


figure;
poles = eig(test.A);
zeros = []; gain = 1;
sys = zpk(zeros, poles, gain);
pzmap(sys);grid on
