function extendx0_test1

% nonlinear constraints (ctrl.details.yalmipData is used)
Double_Integrator
probStruct.tracking=1;
probStruct.Tconstraint=0;
probStruct.N = 2;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
C=C+set(V.x{1}'*V.x{1}<=100);
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[a,b]=extendx0(ctrl,[1;1],0,[1;1]);
mbg_assertequal(length(a), 5);
mbg_asserttrue(b);


probStruct.tracking=0;
sysStruct.dumax = 1;
sysStruct.dumin = -1;
[C,O,V]=mpt_ownmpc(sysStruct, probStruct, 'online');
C=C+set(V.x{1}'*V.x{1}<=100);
ctrl=mpt_ownmpc(sysStruct,probStruct,C,O,V,'online');
[a,b]=extendx0(ctrl,[1;1],0,[1;1]);
mbg_assertequal(length(a), 3);
mbg_asserttrue(b);
