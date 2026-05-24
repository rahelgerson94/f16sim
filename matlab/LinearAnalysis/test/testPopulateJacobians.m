
% Run project setup from the matlab root so this test works from any folder.
testDir = fileparts(mfilename('fullpath'));
matlabRoot = fullfile(testDir, '..', '..');
currentDir = pwd;
cd(matlabRoot); setupProject; cd(fullfile(matlabRoot, 'LinearAnalysis'));
c = getConstants();
%f16NominalTrim; % get xe, ue into the workspace
xeInW = load([matlabRoot '/generated/xeInW.mat']).xeInW;
ue = load([matlabRoot '/generated/ue.mat']).ue;
fprintf("ue : [δ_e, δ_t] = [%.2f   %.2f]\n", ue)
fprintf("[Vt (ft),   θ (deg), α (deg), q (deg)]  = [%.2f    %.2f     %.2f     %.2f ]\n", xeInW(1), xeInW(2), xeInW(3), xeInW(4) );

cd(currentDir);
paramsPath = fullfile(matlabRoot, 'common', 'getVehicleParams.m');
% Pass the params function path so CalcDerivsLon builds params internally.
test = CalcDerivsLon(xeInW, ue, paramsPath);
test.printDeltaUe();
test.printDeltaX();
dt = c.dt;

test.populateA();
test.populateB();


%i want to compare with STevens, which will require a reording of variabels

T = [1 0 0 0;
        0 0 1 0;
        0 1 0 0;
        0 0 0 1];
fprintf("ΔVt     Δα     Δθ     Δq \n");
%A = test.getAinEnglishUnits();
A = test.A;
B = test.B;

% Save generated longitudinal Jacobians using full file paths.
generatedDir = fullfile(matlabRoot, 'generated');
if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

save(fullfile(generatedDir, 'Alon.mat'), 'A');
save(fullfile(generatedDir, 'Blon.mat'), 'B');
StevensA = T*A(1:4, 1:4)*T;
printMatrix(StevensA, 6);
printMatrix(test.B(1:4,:),4) %B is 6,2
