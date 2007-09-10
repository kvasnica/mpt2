function ownmpc_test1

% tests that we properly set "variables.reference_in_x0" and
% "variables.uprev_in_x0":
%

% PWA system
opt_sincos
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
[C, O, Ve] = mpt_ownmpc(S, P);
[C, O, Vo] = mpt_ownmpc(S, P, 'online');
mbg_assertfalse(isfield(Ve, 'reference_in_x0'));
mbg_asserttrue(isfield(Ve, 'uprev_in_x0'));
mbg_assertfalse(isfield(Vo, 'reference_in_x0'));
mbg_asserttrue(isfield(Vo, 'uprev_in_x0'));

%---------------------------------------------------------
% tracking=1
S = sysStruct;
P = probStruct;
P.tracking = 1;
[C, O, Ve] = mpt_ownmpc(S, P);
[C, O, Vo] = mpt_ownmpc(S, P, 'online');
mbg_asserttrue(isfield(Ve, 'reference_in_x0'));
mbg_asserttrue(isfield(Ve, 'uprev_in_x0'));
mbg_asserttrue(isfield(Vo, 'reference_in_x0'));
mbg_asserttrue(isfield(Vo, 'uprev_in_x0'));

%---------------------------------------------------------
% tracking=2
S = sysStruct;
P = probStruct;
P.tracking = 2;
[C, O, Ve] = mpt_ownmpc(S, P);
[C, O, Vo] = mpt_ownmpc(S, P, 'online');
mbg_asserttrue(isfield(Ve, 'reference_in_x0'));
mbg_assertfalse(isfield(Ve, 'uprev_in_x0'));
mbg_asserttrue(isfield(Vo, 'reference_in_x0'));
mbg_assertfalse(isfield(Vo, 'uprev_in_x0'));
