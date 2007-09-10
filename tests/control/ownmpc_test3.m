function ownmpc_test1

% tests that we properly set "variables.reference_in_x0" and
% "variables.uprev_in_x0":
%

% MLD system
opt_sincos
M = mpt_pwa2mld(sysStruct);
sysStruct = mpt_sys(M, 'nopwa');
probStruct.N = 2;
probStruct.Tconstraint = 0;

%---------------------------------------------------------
% no tracking, no deltaU formulation
S = sysStruct;
P = probStruct;
P.tracking = 0;
[C, O, Ve] = mpt_ownmpc(S, P);
[C, O, Vo] = mpt_ownmpc(S, P, 'online');
mbg_assertfalse(isfield(Ve, 'reference_in_x0'));
mbg_assertfalse(isfield(Ve, 'uprev_in_x0'));
mbg_assertfalse(isfield(Vo, 'reference_in_x0'));
mbg_assertfalse(isfield(Vo, 'uprev_in_x0'));

%---------------------------------------------------------
% no tracking, deltaU formulation
S = sysStruct;
S.dumax = 1; S.dumin = -1;
P = probStruct;
P.tracking = 0;
% deltaU constraints not allowed for MLD models/explicit control
try
    [C, O, Ve] = mpt_ownmpc(S, P);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
[C, O, Vo] = mpt_ownmpc(S, P, 'online');
mbg_assertfalse(isfield(Vo, 'reference_in_x0'));
mbg_assertfalse(isfield(Vo, 'uprev_in_x0'));

%---------------------------------------------------------
% tracking=1
S = sysStruct;
P = probStruct;
P.tracking = 1;
% tracking=1 not allowed for MLD systems/explicit control
try
    [C, O, Ve] = mpt_ownmpc(S, P);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
[C, O, Vo] = mpt_ownmpc(S, P, 'online');
mbg_assertfalse(isfield(Vo, 'reference_in_x0'));
mbg_assertfalse(isfield(Vo, 'uprev_in_x0'));

%---------------------------------------------------------
% tracking=2
S = sysStruct;
P = probStruct;
P.tracking = 2;
% tracking=2 not allowed for MLD systems/explicit control
try
    [C, O, Ve] = mpt_ownmpc(S, P);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
[C, O, Vo] = mpt_ownmpc(S, P, 'online');
mbg_assertfalse(isfield(Vo, 'reference_in_x0'));
mbg_assertfalse(isfield(Vo, 'uprev_in_x0'));
