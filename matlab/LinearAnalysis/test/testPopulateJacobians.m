
% Run project setup from the matlab root so this test works from any folder.
testDir = fileparts(mfilename('fullpath'));
matlabRoot = fullfile(testDir, '..', '..');
currentDir = pwd;
cd(matlabRoot); setupProject; cd(fullfile(matlabRoot, 'LinearAnalysis'));
c = getConstants();
f16NominalTrim; % get xe, ue into the workspace
cd(currentDir);
test = CalcDerivsLon(xeWind, ue);
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
Aenglish = test.getAinEnglishUnits();
StevensA = T*Aenglish(1:4, 1:4)*T;
printMatrix(StevensA, 4);
printMatrix(test.B(1:4,:)) %B is 6,2

