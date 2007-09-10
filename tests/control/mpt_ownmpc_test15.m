function mpt_ownmpc_test15

% test whether we correctly return slack variables if soft constraints are used:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=6a53243bbaf7

Double_Integrator
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;

% no slacks should be returned unless soft constraints are enabled
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
mbg_assertfalse(isfield(V, 'sx'));
mbg_assertfalse(isfield(V, 'sy'));
mbg_assertfalse(isfield(V, 'su'));

% V.sy should be defined for soft output constraints
P = probStruct;
P.Sy = 1;
[C, O, V] = mpt_ownmpc(sysStruct, P);
mbg_assertfalse(isfield(V, 'sx'));
mbg_asserttrue(isfield(V, 'sy'));
mbg_assertfalse(isfield(V, 'su'));
mbg_assertequal(length(V.sy), length(V.y));

% V.sx should be defined for soft state constraints
P = probStruct;
P.Sx = 1;
[C, O, V] = mpt_ownmpc(sysStruct, P);
mbg_asserttrue(isfield(V, 'sx'));
mbg_assertfalse(isfield(V, 'sy'));
mbg_assertfalse(isfield(V, 'su'));
mbg_assertequal(length(V.sx), length(V.x));

% V.su should be defined for soft input constraints
P = probStruct;
P.Su = 1;
[C, O, V] = mpt_ownmpc(sysStruct, P);
mbg_assertfalse(isfield(V, 'sx'));
mbg_assertfalse(isfield(V, 'sy'));
mbg_asserttrue(isfield(V, 'su'));
mbg_assertequal(length(V.su), length(V.u));
mbg_assertequal(length(V.su), P.N);

% V.su must have correct dimension if move blocking is used
P = probStruct;
P.Su = 1;
probStruct.Nc = 2;
[C, O, V] = mpt_ownmpc(sysStruct, P);
mbg_assertfalse(isfield(V, 'sx'));
mbg_assertfalse(isfield(V, 'sy'));
mbg_asserttrue(isfield(V, 'su'));
mbg_assertequal(length(V.su), length(V.u));

% V.sx and V.sy must be defined
P = probStruct;
P.Sx = 1; P.symax = [1; 1];
[C, O, V] = mpt_ownmpc(sysStruct, P);
mbg_asserttrue(isfield(V, 'sx'));
mbg_asserttrue(isfield(V, 'sy'));
mbg_assertfalse(isfield(V, 'su'));
mbg_assertequal(length(V.sx), length(V.x));
mbg_assertequal(length(V.sy), length(V.y));

% All constraints soft
P = probStruct;
P.sxmax = [1; 1]; P.symax=[0; 0]; P.sumax = 1;
P.Nc = 2;
[C, O, V] = mpt_ownmpc(sysStruct, P);
mbg_asserttrue(isfield(V, 'sx'));
mbg_asserttrue(isfield(V, 'sy'));
mbg_asserttrue(isfield(V, 'su'));
mbg_assertequal(length(V.sx), length(V.x));
mbg_assertequal(length(V.sy), length(V.y));
mbg_assertequal(length(V.su), length(V.u));
mbg_assertequal(length(V.su), P.Nc);
