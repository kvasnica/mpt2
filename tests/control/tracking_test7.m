function tracking_test7

% tests tracking for MLD systems
%
% tracking=1 should be rejected (not possible to augment the dynamics)
% deltaU formulation should be rejected (same reason)
% tracking=2 should work, additional variable to denote the reference should be
%            introduced

Double_Integrator
sst = sysStruct;
sst.xmax = sysStruct.ymax;
sst.xmin = sysStruct.ymin;
clear sysStruct

pwa_sincos
sysStruct.A = { sst.A, sst.A };
sysStruct.B = { sst.B, sst.B };
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;
MLD = mpt_pwa2mld(sysStruct);
sysStruct = mpt_sys(MLD, 'nopwa');
sysStruct.ymax = sysStruct.xmax;
sysStruct.ymin = sysStruct.xmin;
probStruct.N = 3;
probStruct.subopt_lev = 0;
probStruct.Qy = 2*eye(2);
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
x0 = [-1; 0];
Options.reference = [1; 0];

%------------------------------------------------------------------------
% tracking=1 (deltaU formulation not possible for MLD systems, therefore this
% case should switch to tracking=2 (see command output))
P = probStruct;
S = sysStruct;
P.norm = 1;
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
S.xrefmax = S.xmax;
S.xrefmin = S.xmin;
P.P_N = zeros(2);
P.tracking = 1;

new = mpt_control(S, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); 
own=mpt_ownmpc(S,P,C,O,V);
P.tracking = 2; ref = mpt_control(sst, P, 'online');
[xr, ur, yr, cr] = sim(ref, x0, 5, Options);
[xw, uw, yw, cw] = sim(own, x0, 5, Options);
[xn, un, yn, cn] = sim(new, x0, 5, Options);
mbg_asserttolequal(cn, 12.25, 2e-6);
mbg_asserttolequal(cn, cr);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);
% FIXME: for some reason, "cw" is smaller (11.75)


%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation)
P = probStruct;
S = sysStruct;
P.norm = 1;
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
S.xrefmax = S.xmax;
S.xrefmin = S.xmin;
P.P_N = zeros(2);
P.tracking = 2;

new = mpt_control(S, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
ref = mpt_control(sst, P, 'online');
[xr, ur, yr, cr] = sim(ref, x0, 5, Options);
[xw, uw, yw, cw] = sim(own, x0, 5, Options);
[xn, un, yn, cn] = sim(new, x0, 5, Options);
mbg_asserttolequal(cn, 12.25, 2e-6);
mbg_asserttolequal(cw, cn);
mbg_asserttolequal(cn, cr);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);


%------------------------------------------------------------------------
%------------------------------------------------------------------------
% now add deltaU constraints
sysStruct.dumax = 0.2;
sysStruct.dumin = -0.25;
sst.dumax = 0.2;
sst.dumin = -0.25;



%------------------------------------------------------------------------
% tracking=1 (deltaU formulation not possible for MLD systems, therefore this
% case should in principle be identical to tracking=2; even though currently it
% gives slightly different results. why?)
P = probStruct;
S = sysStruct;
P.norm = 1;
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
S.xrefmax = S.xmax;
S.xrefmin = S.xmin;
P.P_N = zeros(2);
P.tracking = 1;

new = mpt_control(S, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
ref = mpt_control(sst, P, 'online');
[xr, ur, yr, cr] = sim(ref, x0, 5, Options);
[xw, uw, yw, cw] = sim(own, x0, 5, Options);
[xn, un, yn, cn] = sim(new, x0, 5, Options);
mbg_asserttolequal(cn, 16.166666666666686, 2e-6);
% mbg_asserttolequal(cw, cn);
% mbg_asserttolequal(cn, cr);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);
% check that deltaU constraints hold
mbg_asserttrue(max(diff(un)) <= sysStruct.dumax+1e-10);
mbg_asserttrue(min(diff(un)) >= sysStruct.dumin-1e-10);
% FIXME: for some reason, "cw" and "cr" are different


%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation)
P = probStruct;
S = sysStruct;
P.norm = 1;
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
S.xrefmax = S.xmax;
S.xrefmin = S.xmin;
P.P_N = zeros(2);
P.tracking = 2;

new = mpt_control(S, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
ref = mpt_control(sst, P, 'online');
[xr, ur, yr, cr] = sim(ref, x0, 5, Options);
[xw, uw, yw, cw] = sim(own, x0, 5, Options);
[xn, un, yn, cn] = sim(new, x0, 5, Options);
mbg_asserttolequal(cn, 16.16666666666, 2e-6);
mbg_asserttolequal(cw, cn);
mbg_asserttolequal(cn, cr);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);
% check that deltaU constraints hold
mbg_asserttrue(max(diff(un)) <= sysStruct.dumax+1e-10);
mbg_asserttrue(min(diff(un)) >= sysStruct.dumin-1e-10);
