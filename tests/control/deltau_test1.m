function deltau_test1

% tests deltaU formulation for LTI systems

Double_Integrator
probStruct.N = 2;
x0 = [1; 1];
Options = [];

%------------------------------------------------------------------------
% tracking=0, deltaU penalty
P = probStruct;
S = sysStruct;
P.Rdu = 10;
P.P_N = zeros(2);
P.tracking = 0;
P.Tconstraint = 0;
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 35.83904584148972);
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%------------------------------------------------------------------------
% tracking=0, deltaU constraints, deltaU penalty 
P = probStruct;
S = sysStruct;
S.dumax = 0.3; S.dumin = -0.15;
P.Rdu = 10;
P.P_N = zeros(2);
P.tracking = 0;
P.Tconstraint = 0;
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 42.49219843293512);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%------------------------------------------------------------------------
% tracking=0, deltaU constraints, ny!=nx
P = probStruct;
S = sysStruct;
S.dumax = 0.3; S.dumin = -0.15;
S.C = [1 0]; S.D = 0; S.ymax = S.ymax(1); S.ymin = S.ymin(1);
P.Qy = 1;
P.Rdu = 10;
P.P_N = 0;
P.tracking = 0;
P.Tconstraint = 0;
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 37.79623443027141);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);
