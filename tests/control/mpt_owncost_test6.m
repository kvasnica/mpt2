function mpt_ownmpc_test6

% on this example solvemp() was returning error code 0 (=no problems), even
% though the returned solution is empty

Double_Integrator
probStruct.N=3; probStruct.norm=1;

sysStruct.xmax=sysStruct.ymax;
sysStruct.xmin=sysStruct.ymin;

[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
C=C+set(norm(V.x{2}, 1) < norm(V.x{1}, 1))+set(norm(V.x{3}, 1) < norm(V.x{2}, 1));
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V);
mbg_assertequal(length(ctrl), 32);


% test the same with an on-line controller
[C, O, V] = mpt_ownmpc(sysStruct, probStruct, 'online');
C=C+set(norm(V.x{2}, 1) < norm(V.x{1}, 1))+set(norm(V.x{3}, 1) < norm(V.x{2}, 1));
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
