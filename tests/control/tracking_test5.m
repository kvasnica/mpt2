function mpt_tracking_test5

% tests on-line MPC for systems where we can augment the state vector to cope
% with tracking/deltaU constraints (PWA, no boolean inputs, ny==nx)

pwa_sincos
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;
probStruct.N = 2;
probStruct.norm = 2;
probStruct.subopt_lev = 0;
x0 = [-1; 0];

%------------------------------------------------------------------------
% tracking=1 (deltaU formulation, no boolean inputs, ny == nx, 2-norm)
P = probStruct;
S = sysStruct;
P.norm = 2;
% S.yrefmax = S.ymax;
% S.yrefmin = S.ymin;
% S.xrefmax = S.xmax;
% S.xrefmin = S.xmin;
P.P_N = zeros(2);
P.tracking = 1;
new = mpt_control(S, P, 'online')
exp = mpt_control(S, P)
Options.reference = [1; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
mbg_asserttolequal(cn, ce, 1e-7);
mbg_asserttolequal(cn, 12.319181379542902, 1e-7);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);


%------------------------------------------------------------------------
% tracking=1 (deltaU formulation, no boolean inputs, ny == nx, 1-norm)
P = probStruct;
S = sysStruct;
P.norm = 1;
P.Q = 10*eye(2);
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
P.P_N = zeros(2);
P.tracking = 1;
new = mpt_control(S, P, 'online')
Options.reference = [1; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
mbg_asserttolequal(cn, 1.073549363043369e+02, 1e-7);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);


%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation, no boolean inputs, ny == nx, 2-norm)
P = probStruct;
S = sysStruct;
P.norm = 2;
P.Q = 10*eye(2);
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
P.P_N = zeros(2);
P.tracking = 2;
new = mpt_control(S, P, 'online')
old = mpt_control(S, P, 'online', struct('force_mpt', 1))
exp = mpt_control(S, P)
Options.reference = [1; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
mbg_asserttolequal(cn, co, 3e-4);
mbg_asserttolequal(cn, ce, 1e-7);
mbg_asserttolequal(cn, 1.432673963492821e+02, 1e-5);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);


%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation, no boolean inputs, ny == nx, 1-norm)
P = probStruct;
S = sysStruct;
P.norm = 1;
P.Q = 10*eye(2);
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
P.P_N = zeros(2);
P.tracking = 2;
new = mpt_control(S, P, 'online')
old = mpt_control(S, P, 'online', struct('force_mpt', 1))
exp = mpt_control(S, P)
Options.reference = [1; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
mbg_asserttolequal(cn, co, 1e-7);
mbg_asserttolequal(cn, ce, 1e-7);
mbg_asserttolequal(cn, 1.178194984992522e+02, 1e-7);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);
