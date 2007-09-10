function mpt_tracking_test3

% tests on-line MPC for systems where we can augment the state vector to cope
% with tracking/deltaU constraints (Inf-norm case)

Double_Integrator
probStruct.norm = Inf;
x0 = [1; 1];

%------------------------------------------------------------------------
% tracking=1 (deltaU formulation)
P = probStruct;
S = sysStruct;
P.P_N = zeros(2);
P.R = 0;
P.Rdu = 0;
P.tracking = 1;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = [2; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, 2.92196311537876);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);

%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation)
P = probStruct;
S = sysStruct;
P.P_N = zeros(2);
P.tracking = 2;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = [2; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, 4.91329235042084);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);

%------------------------------------------------------------------------
% tracking=1, ny != nx
P = probStruct;
S = sysStruct;
S.C = [1 0]; S.D = 0; S.ymax = S.ymax(1); S.ymin = S.ymin(1);
P.P_N = zeros(1);
P.Qy = 10;
P.tracking = 1;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = 2;
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, 11.99609375000002);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);

%------------------------------------------------------------------------
% tracking=2, ny != nx
P = probStruct;
S = sysStruct;
S.C = [1 0]; S.D = 0; S.ymax = S.ymax(1); S.ymin = S.ymin(1);
P.P_N = zeros(1);
P.Qy = 10;
P.tracking = 2;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = 2;
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, 11.99609375000001);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);

%------------------------------------------------------------------------
% tracking=0, deltaU penalty
P = probStruct;
S = sysStruct;
P.Rdu = 10;
P.P_N = zeros(2);
P.tracking = 0;
P.Tconstraint = 0;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = [2; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
mbg_asserttolequal(cn, co, 1e-12);
%mbg_asserttolequal(cn, 14.04492187500000);
mbg_asserttolequal(cn, 14.06224681);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);

%------------------------------------------------------------------------
% tracking=0, deltaU constraints, deltaU penalty 
P = probStruct;
S = sysStruct;
S.dumax = 0.5; S.dumin = -0.5;
P.Rdu = 10;
P.P_N = zeros(2);
P.tracking = 0;
P.Tconstraint = 0;
new = mpt_control(S, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = [2; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
mbg_asserttolequal(cn, 16.11208767361111);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);

%------------------------------------------------------------------------
% tracking=0, deltaU constraints, ny!=nx
P = probStruct;
S = sysStruct;
S.dumax = 0.5; S.dumin = -0.5;
S.C = [1 0]; S.D = 0; S.ymax = S.ymax(1); S.ymin = S.ymin(1);
P.Qy = 1;
P.Rdu = 10;
P.P_N = 0;
P.tracking = 0;
P.Tconstraint = 0;
new = mpt_control(S, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = [2; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
mbg_asserttolequal(cn, 15.93755206164099);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);
