function mpt_mplp_test17

% tests that we correctly compute cost if F*x affine term is given:
%

Double_Integrator;
probStruct.norm = 1; probStruct.N = 1;
M = mpt_constructMatrices(sysStruct, probStruct);
M.F = [1 0];
[a, b, c, d, e, f] = mpt_mplp(M);
M.F = [1 0]';
[a, b, c, d, e, f] = mpt_mplp(M);


sdpvar x u
F = set(-1 < x < 1) + set(-1 < u < 1) + set( x < u);
mpsol = solvemp(F,x+u,[],x);
mbg_assertequal(mpsol{1}.Bi{1}, 2);
