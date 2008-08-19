function mpt_mpsol2ctrl_test1

% mpt_mpsol2ctrl() converts solutions generated by solvemp() into MPT's
% conroller objects

Double_Integrator
probStruct.N = 1; probStruct.norm = 2;
probStruct.Tconstraint = 0;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
sol = solvemp(C, O, [], V.x{1}, V.u{1});
ctrl = mpt_mpsol2ctrl(sol, length(V.u{1}));
mbg_assertequal(length(ctrl.Pn), length(sol{1}.Pn));

Double_Integrator
probStruct.N = 1; probStruct.norm = 1;
probStruct.Tconstraint = 0;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
sol = solvemp(C, O, [], V.x{1}, V.u{1});
ctrl = mpt_mpsol2ctrl(sol, length(V.u{1}), length(V.y{1}));
mbg_assertequal(length(ctrl.Pn), length(sol{1}.Pn));

opt_sincos
probStruct.N = 1; probStruct.norm = 2;
probStruct.Tconstraint = 0;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
sol = solvemp(C, O, [], V.x{1}, V.u{1});
ctrl = mpt_mpsol2ctrl(sol, length(V.u{1}), length(V.y{1}));
nr = 0;
for i = 1:length(sol),
    nr = nr + length(sol{i}.Pn);
end
mbg_assertequal(nr, length(ctrl.Pn));

opt_sincos
probStruct.N = 1; probStruct.norm = 1;
probStruct.Tconstraint = 0;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
sol = solvemp(C, O, [], V.x{1}, V.u{1});
ctrl = mpt_mpsol2ctrl(sol, length(V.u{1}));
nr = 0;
for i = 1:length(sol),
    nr = nr + length(sol{i}.Pn);
end
mbg_assertequal(nr, length(ctrl.Pn));