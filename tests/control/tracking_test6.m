function tracking_test6

% tests on-line MPC for systems where we can augment the state vector to cope
% with tracking/deltaU constraints (PWA, no boolean inputs, ny!=nx)

Double_Integrator
sst = sysStruct;
sst.xmax = sysStruct.ymax;
sst.xmin = sysStruct.ymin;
sst.ymax = sysStruct.ymax(1);
sst.ymin = sysStruct.ymin(1);
sst.C = [1 0]; sst.D = 0;
clear sysStruct

pwa_sincos
sysStruct.A = { sst.A, sst.A };
sysStruct.B = { sst.B, sst.B };
[sysStruct.C{:}] = deal([1 0]);
[sysStruct.D{:}] = deal(0);
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;
sysStruct.ymax = sysStruct.ymax(1);
sysStruct.ymin = sysStruct.ymin(1);
probStruct.N = 3;
probStruct.subopt_lev = 0;
probStruct.Qy = 2;
x0 = [-1; 0];
Options.reference = 1;

%------------------------------------------------------------------------
% tracking=1 (deltaU formulation, no boolean inputs, ny != nx, 2-norm)
P = probStruct;
S = sysStruct;
P.norm = 1;
S.xrefmax = S.xmax + 1;
S.xrefmin = S.xmin - 1;
P.P_N = zeros(2);
P.tracking = 1;
new = mpt_control(S, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
ref = mpt_control(sst, P, 'online');
[xr, ur, yr, cr] = sim(ref, x0, 5, Options);
[xw, uw, yw, cw] = sim(own, x0, 5, Options);
[xn, un, yn, cn] = sim(new, x0, 5, Options);
mbg_asserttolequal(cn, 8.64, 2e-6);
mbg_asserttolequal(cw, cn);
mbg_asserttolequal(cn, cr);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);


%------------------------------------------------------------------------
% tracking=1 (deltaU formulation, no boolean inputs, ny != nx, 1-norm)
P = probStruct;
S = sysStruct;
P.norm = 1;
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
P.P_N = zeros(2);
P.tracking = 1;
new = mpt_control(S, P, 'online');
ref = mpt_control(sst, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xn, un, yn, cn] = sim(new, x0, 5, Options);
[xw, uw, yw, cw] = sim(own, x0, 5, Options);
[xr, ur, yr, cr] = sim(ref, x0, 5, Options);
mbg_asserttolequal(cn, 8.64);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);
mbg_asserttolequal(cw, cn);
mbg_asserttolequal(cn, cr);


% FIXME:
% %------------------------------------------------------------------------
% % tracking=2 (no deltaU formulation, no boolean inputs, ny != nx, 2-norm)
% P = probStruct;
% S = sysStruct;
% P.norm = 2;
% S.yrefmax = S.ymax;
% S.yrefmin = S.ymin;
% sst.yrefmax = S.ymax;
% sst.yrefmin = S.ymin;
% P.P_N = zeros(2);
% P.tracking = 2;
% new = mpt_control(S, P, 'online');
% ref = mpt_control(sst, P, 'online');
% [C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
% [xn, un, yn, cn] = sim(new, x0, 5, Options);
% [xw, uw, yw, cw] = sim(own, x0, 5, Options);
% [xr, ur, yr, cr] = sim(ref, x0, 5, Options);
% mbg_asserttolequal(cn, 11.923170061464933);
% mbg_asserttolequal(max(abs(un-uw)), 0);
% mbg_asserttolequal(max(max(abs(xn-xw))), 0);
% mbg_asserttolequal(cw, cn);
% mbg_asserttolequal(cn, cr);


%------------------------------------------------------------------------
% tracking=2 (no deltaU formulation, no boolean inputs, ny != nx, 1-norm)
P = probStruct;
S = sysStruct;
P.norm = 1;
S.yrefmax = S.ymax;
S.yrefmin = S.ymin;
P.P_N = zeros(2);
P.tracking = 2;
new = mpt_control(S, P, 'online');
ref = mpt_control(sst, P, 'online');
[C,O,V]=mpt_ownmpc(S,P,'online'); own=mpt_ownmpc(S,P,C,O,V);
[xn, un, yn, cn] = sim(new, x0, 5, Options);
[xw, uw, yw, cw] = sim(own, x0, 5, Options);
[xr, ur, yr, cr] = sim(ref, x0, 5, Options);
mbg_asserttolequal(cn, 8.75);
mbg_asserttolequal(max(abs(un-uw)), 0);
mbg_asserttolequal(max(max(abs(xn-xw))), 0);
mbg_asserttolequal(cw, cn);
mbg_asserttolequal(cn, cr);
