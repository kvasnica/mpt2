function tracking_test4

% tests on-line MPC for systems where we can augment the state vector to cope
% with tracking/deltaU constraints (PWA, boolean inputs)

two_tanks
probStruct.norm = 2; probStruct.N = 2;
probStruct = rmfield(probStruct, 'yref');
x0 = [0.5; 0.1];

%------------------------------------------------------------------------
% tracking=1 (deltaU formulation, ny != nx)
lasterr('');
P = probStruct;
S = sysStruct;
P.P_N = zeros(2);
P.tracking = 1;
try
    % this system has boolean inputs, cannot use tracking=1
    new = mpt_control(S, P, 'online');
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
lasterr('');
try
    % this system has boolean inputs, cannot use tracking=1
    new = mpt_control(S, P);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);


%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation, boolean inputs, ny != nx, 2-norm)
P = probStruct;
S = sysStruct;
P.norm = 2;
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
P.P_N = zeros(2);
P.tracking = 2;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
exp = mpt_control(S, P);
Options.reference = 0.3;
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
% mbg_asserttolequal(cn, co. 1e-7);
% mbg_asserttolequal(cn, ce. 1e-7);
mbg_asserttolequal(ce, 28.374504987250685);

% check displaying of controllers
new, old, exp
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);


%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation, boolean inputs, ny != nx, 1-norm)
P = probStruct;
S = sysStruct;
P.norm = 1;
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
P.P_N = zeros(2);
P.tracking = 2;
new = mpt_control(S, P, 'online');
old = mpt_control(S, P, 'online', struct('force_mpt', 1));
exp = mpt_control(S, P);
Options.reference = 0.3;
[xn, un, yn, cn] = sim(new, x0, 10, Options);
[xo, uo, yo, co] = sim(old, x0, 10, Options);
[xe, ue, ye, ce] = sim(exp, x0, 10, Options);
mbg_asserttolequal(cn, co);
mbg_asserttolequal(cn, ce);
mbg_asserttolequal(cn, 2.151840488219607e+002);
% check displaying of controllers
new, old, exp
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xw, uw, yw, cw] = sim(own, x0, 10, Options);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);
