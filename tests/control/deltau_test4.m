function deltau_test4

% tests whether we properly update references in mpt_yalmipDU:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=afefdb1ec90f

x0 = [-1; 0];
Double_Integrator
probStruct.N = 2;
sysStruct.dumax = 1;
sysStruct.dumin = -1;
Options = [];

%-------------------------------------------------------------------
% 2-norm, xref, ny==nx
S = sysStruct; P = probStruct;
P.norm = 2;
P.xref = [1; 0];
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 7.27351457768077);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%-------------------------------------------------------------------
% 1-norm, xref, ny==nx
S = sysStruct; P = probStruct;
P.norm = 1;
P.P_N = zeros(2);
P.Q = eye(2)*10;
P.xref = [1; 0];
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 54.91796874999999);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%-------------------------------------------------------------------
% 2-norm, yref, ny==nx
S = sysStruct; P = probStruct;
P.norm = 2;
P.yref = [1; 0];
P.Qy = 10*eye(2);
P.P_N = zeros(2);
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 60.48230482793647);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%-------------------------------------------------------------------
% 1-norm, yref, ny==nx
S = sysStruct; P = probStruct;
P.norm = 1;
P.yref = [1; 0];
P.Qy = 10*eye(2);
P.P_N = zeros(2);
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 54.91796875000011);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%-------------------------------------------------------------------
% 2-norm, yref, ny!=nx
S = sysStruct; P = probStruct;
S.C = [1 0]; S.D = 0; S.ymax = S.ymax(1); S.ymin = S.ymin(1);
P.norm = 2;
P.yref = 1;
P.Qy = 10;
P.P_N = zeros(2);
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 52.19203061709420);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);

%-------------------------------------------------------------------
% 1-norm, yref, ny!=nx
S = sysStruct; P = probStruct;
S.C = [1 0]; S.D = 0; S.ymax = S.ymax(1); S.ymin = S.ymin(1);
P.norm = 1;
P.yref = 1;
P.Qy = 10;
P.P_N = 0;
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 34.99609375000004);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(uo-uw)), 0);
mbg_asserttolequal(max(max(abs(xo-xw))), 0);
