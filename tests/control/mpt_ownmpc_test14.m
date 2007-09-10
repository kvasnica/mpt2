function mpt_ownmpc_test14

% tests that we return correct number of manipulated variables if move blocking
% is used:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=089dad36110e

Double_Integrator
probStruct.Tconstraint = 0;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
mbg_assertequal(length(V.u), probStruct.N);
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
mbg_assertequal(length(V.u), probStruct.N);

Double_Integrator
probStruct.Tconstraint = 0;
probStruct.Nc = 2;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
mbg_assertequal(length(V.u), probStruct.Nc);
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
mbg_assertequal(length(V.u), probStruct.Nc);

two_tanks
probStruct.Tconstraint = 0;
probStruct.N = 5;
probStruct.Nc = 1;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
mbg_assertequal(length(V.u), probStruct.Nc);
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
mbg_assertequal(length(V.u), probStruct.Nc);
