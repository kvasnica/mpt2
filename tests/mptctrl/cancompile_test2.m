function cancompile_test1

opt_sincos
sysStruct.ymax = [1;1];
sysStruct.ymin = [-1;-1];
probStruct.Tconstraint = 0;
probStruct.N = 2;

% can export 1-norm controllers for PWA
S = sysStruct;
P = probStruct;
P.norm = 1;
ctrl = mpt_control(S, P);
mbg_asserttrue(cancompile(ctrl));
[C, O, V] = mpt_ownmpc(S, P);
ctrl = mpt_ownmpc(S, P, C, O, V);
mbg_asserttrue(cancompile(ctrl));

% can't export 1-norm controllers for PWA
S = sysStruct;
P = probStruct;
P.norm = 2;
ctrl = mpt_control(S, P);
mbg_assertfalse(cancompile(ctrl));
[C, O, V] = mpt_ownmpc(S, P);
ctrl = mpt_ownmpc(S, P, C, O, V);
mbg_assertfalse(cancompile(ctrl));
