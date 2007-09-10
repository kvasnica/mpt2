function deltau_test2

% tests deltaU formulation for PWA systems

opt_sincos
probStruct.norm = 2;
probStruct.N = 2;
x0 = [5; 1];
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
mbg_asserttolequal(ce, co, 1e-6);
mbg_asserttolequal(ce, 52.03183748681701);
% check displaying of controllers
exp, onl


%------------------------------------------------------------------------
% tracking=0, deltaU constraints, deltaU penalty 
P = probStruct;
S = sysStruct;
S.dumax = 0.3; S.dumin = -0.15;
P.Rdu = 0.1;
P.P_N = zeros(2);
P.tracking = 0;
P.Tconstraint = 0;
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co);
mbg_asserttolequal(ce, 55.44110942769241);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl

%------------------------------------------------------------------------
% tracking=0, deltaU constraints, ny!=nx
P = probStruct;
S = sysStruct;
S.dumax = 0.3; S.dumin = -0.15;
[S.C{:}] = deal([1 0]);
[S.D{:}] = deal(0); 
S.ymax = S.ymax(1); S.ymin = S.ymin(1);
P.Qy = 10;
P.Rdu = 0.1;
P.P_N = 0;
P.tracking = 0;
P.Tconstraint = 0;
exp = mpt_control(S, P);
onl = mpt_control(S, P, 'online');
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(ce, co, 1e-5);
mbg_asserttolequal(ce, 3.545192260855196e+002);
% check deltaU constraints
mbg_asserttrue(max(diff(ue)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(ue)) >= S.dumin-1e-10); 
% check displaying of controllers
exp, onl
