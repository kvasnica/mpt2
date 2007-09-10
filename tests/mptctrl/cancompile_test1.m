function cancompile_test1

Double_Integrator
sysStruct.ymax = [1;1];
sysStruct.ymin = [-1;-1];
probStruct.Tconstraint = 0;
probStruct.N = 2;

% can export 2-norm controllers for linear systems
S = sysStruct;
P = probStruct;
ctrl = mpt_control(S, P);
mbg_asserttrue(cancompile(ctrl));
[C, O, V] = mpt_ownmpc(S, P);
ctrl = mpt_ownmpc(S, P, C, O, V);
mbg_asserttrue(cancompile(ctrl));

% can export 2-norm controllers for linear systems
S = sysStruct;
P = probStruct;
ctrl = mpt_control(S, P);
mbg_asserttrue(cancompile(ctrl));
[C, O, V] = mpt_ownmpc(S, P);
ctrl = mpt_ownmpc(S, P, C, O, V);
mbg_asserttrue(cancompile(ctrl));

% can't export if we have overlaps
S = sysStruct;
P = probStruct;
S.Uset = [-1 0 1];
ctrl = mpt_control(S, P);
mbg_assertfalse(cancompile(ctrl));
[C, O, V] = mpt_ownmpc(S, P);
ctrl = mpt_ownmpc(S, P, C, O, V);
mbg_assertfalse(cancompile(ctrl));

% can export overlapping controllers if they come from minimum-time strategy
S = sysStruct;
P = probStruct;
P.Tconstraint = 1;
P.subopt_lev = 1;
ctrl = mpt_control(S, P);
mbg_asserttrue(cancompile(ctrl));
