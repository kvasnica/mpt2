function mpt_tracking_test1

% tests on-line MPC for systems where we can augment the state vector to cope
% with tracking/deltaU constraints

Double_Integrator
x0 = [1; 1];
probStruct.N = 2;

%------------------------------------------------------------------------
% tracking=1 (deltaU formulation)
P = probStruct;
S = sysStruct;
P.P_N = zeros(2);
P.tracking = 1;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
exp = mpt_control(S, P);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = [2; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(cn, cw);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, ce);
mbg_asserttolequal(cn, 3.92734219142345);
% check that we reached the reference
mbg_asserttolequal(xn(end, :)', Options.reference, 5e-2);
% check displaying of controllers
new, old, exp

%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation)
P = probStruct;
S = sysStruct;
P.P_N = zeros(2);
P.tracking = 2;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
exp = mpt_control(S, P);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = [2; 0];
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(cn, cw);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, ce);
mbg_asserttolequal(cn, 4.54769892444988);
% check that we reached the reference
mbg_asserttolequal(xn(end, :)', Options.reference, 5e-2);
% check displaying of controllers
new, old, exp

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
exp = mpt_control(S, P);
Options.reference = 2;
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(cn, cw);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, ce);
mbg_asserttolequal(cn, 11.20821763758693);
% check that we reached the reference
mbg_asserttolequal(yn(end, :)', Options.reference, 5e-3);
% check displaying of controllers
new, old, exp

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
exp = mpt_control(S, P);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
Options.reference = 2;
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(cn, cw);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, ce);
mbg_asserttolequal(cn, 11.38947258699962);
% check that we reached the reference
mbg_asserttolequal(yn(end, :)', Options.reference, 5e-3);
% check displaying of controllers
new, old, exp
