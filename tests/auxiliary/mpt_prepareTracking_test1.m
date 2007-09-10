function mpt_prepareTracking_test1

% tests that we properly deal with cases when probStruct.xref is defined along
% with probStruct.Tset which is a polytope array:
%

% Tset is a single polytope
Double_Integrator
probStruct.N = 1;
probStruct.xref = [0.5; 0];
probStruct.Tset = unitbox(2, 1);
[st,pt] = mpt_prepareTracking(sysStruct, probStruct);
mbg_assertequal(length(pt.Tset), 1);
mbg_asserttrue(pt.Tset == probStruct.Tset+(-probStruct.xref));

% Tset is a polytope array
Double_Integrator
probStruct.N = 1;
probStruct.xref = [0.5; 0];
probStruct.Tset = [unitbox(2, 1) unitbox(2,1)];
[st,pt] = mpt_prepareTracking(sysStruct, probStruct);
mbg_assertequal(length(pt.Tset), 2);
mbg_asserttrue(pt.Tset == probStruct.Tset+(-probStruct.xref));

% Tset is a polytope array, PWA system
opt_sincos
probStruct.N = 1;
probStruct.xref = [0.5; 0];
probStruct.Tset = [unitbox(2, 1) unitbox(2,1)];
[st,pt] = mpt_prepareTracking(sysStruct, probStruct);
mbg_assertequal(length(pt.Tset), 2);
mbg_asserttrue(pt.Tset == probStruct.Tset+(-probStruct.xref));
