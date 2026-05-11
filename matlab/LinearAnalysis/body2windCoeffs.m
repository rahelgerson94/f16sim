function CinW = body2windCoeffs(CinB, alphaRad, betaRad)

CinW = zeros(3,1);
Cx = CinB(1); Cn = -CinB(3);
Cy = CinB(2);

Cd = -cos(alphaRad)*cos(betaRad)*Cx - sin(betaRad)*Cy + sin(alphaRad)*cos(betaRad)*Cn;
Cl = sin(alphaRad)*Cx + cos(alphaRad)*Cn;

CinW(1) = Cd;
CinW(2) = Cy;
CinW(3)  = Cl;
end