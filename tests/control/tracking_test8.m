function tracking_tets8

% tests tracking with mpt_ownmpc()

Double_Integrator
probStruct.N = 2;
x0 = [-1; 0];
Options.reference = [1; 0];

%----------------------------------------------------------------------------
% tracking=1
S = sysStruct;
P = probStruct;
P.tracking = 1;
[C, O, V] = mpt_ownmpc(S, P, 'online');
onl = mpt_ownmpc(S, P, C, O, V)
[C, O, V] = mpt_ownmpc(S, P);
exp = mpt_ownmpc(S, P, C, O, V)
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
mbg_asserttolequal(co, ce);
mbg_asserttolequal(co, 8.59074316147959);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%----------------------------------------------------------------------------
% tracking=2
S = sysStruct;
P = probStruct;
P.tracking = 2;
[C, O, V] = mpt_ownmpc(S, P, 'online');
onl = mpt_ownmpc(S, P, C, O, V)
[C, O, V] = mpt_ownmpc(S, P);
exp = mpt_ownmpc(S, P, C, O, V)
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
mbg_asserttolequal(co, ce);
mbg_asserttolequal(co, 7.48134003106734);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%----------------------------------------------------------------------------
% tracking=0, deltaU constraints
S = sysStruct;
S.dumax = 0.3; S.dumin = -0.15;
P = probStruct;
P.tracking = 0;
[C, O, V] = mpt_ownmpc(S, P, 'online');
onl = mpt_ownmpc(S, P, C, O, V)
[C, O, V] = mpt_ownmpc(S, P);
exp = mpt_ownmpc(S, P, C, O, V)
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
mbg_asserttolequal(co, ce);
mbg_asserttolequal(co, 2.01423656631167);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%----------------------------------------------------------------------------
% tracking=1, deltaU constraints
Options.reference = [1; 0];
S = sysStruct;
S.dumax = 0.5; S.dumin = -0.5;
P = probStruct;
P.tracking = 1;
P.Q = 10*eye(2);
[C, O, V] = mpt_ownmpc(S, P, 'online');
onl = mpt_ownmpc(S, P, C, O, V)
[C, O, V] = mpt_ownmpc(S, P);
exp = mpt_ownmpc(S, P, C, O, V)
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
mbg_asserttolequal(co, ce, 1e-7);
mbg_asserttolequal(co, 1.591763153928906e+002, 1e-7);
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);
