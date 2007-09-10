function mpt_ownmpc_test1

% YALMIP after May-12-2006 extracts bounds on variables directly from
% constraints, therefore the bounds() operator is deprecated. that's why this
% test is skipped
disp('Test no longer necessary...');
return

global mptOptions
ib = mptOptions.infbox;

% YALMIP since 22nd of March 2006 requires bounds on variables to be set if
% implies() or iff() operators are used, otherwise it will give a warning.
% therefore mpt_yalmipcftoc() has been changed to introduce
% +/- mptOptions.infbox bounds if they cannot be deduced from the constraints.
% however, this is only done if we really use implies() or iff() (PWA or
% piecewise nonlinear systems)
%
% therefore here we check that we set the bounds correctly

disp('No warnings should be given in this test.');

% PWA system, bounds must be not infinity
opt_sincos
sysStruct = rmfield(sysStruct, 'xmax');
sysStruct = rmfield(sysStruct, 'xmin');
probStruct.N = 2;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
B = yalmip('getbounds', getvariables(V.x{1}));
mbg_asserttrue(isequal(B, [-ib ib; -ib ib]));

% PWA system, Inf terms in constraints must be replaced
opt_sincos
sysStruct.xmax = [Inf; 20];
sysStruct.xmin = [-10; -Inf];
probStruct.N = 2;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
B = yalmip('getbounds', getvariables(V.x{1}));
mbg_asserttrue(isequal(B, [-10 ib; -ib 20]));

% PWA system w/ soft constraints, bounds must not be infinity
opt_sincos
probStruct.N = 2;
probStruct.Sx = 100;  % this will set +Inf bounds on slacks
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
B = yalmip('getbounds', getvariables(V.x{1}));
mbg_asserttrue(isequal(B, [-[ib; ib]+sysStruct.xmin [ib; ib]+sysStruct.xmax]));

disp('Except of "WARNING: no state constraints given, cannot soften them."');
% PWA system with no state constraints and soft state constraints, bounds must
% not be infinity 
opt_sincos
probStruct.N = 2;
probStruct.Sx = 100;  % this will set +Inf bounds on slacks
sysStruct = rmfield(sysStruct, 'xmax');
sysStruct = rmfield(sysStruct, 'xmin');
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
B = yalmip('getbounds', getvariables(V.x{1}));
mbg_asserttrue(isequal(B, [-[ib; ib] [ib; ib]]));


disp('No more warnings beyond this point.');
% PWA system w/ Inf terms in U
opt_sincos
probStruct.N = 2;
sysStruct.umax = Inf;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
B = yalmip('getbounds', getvariables(V.u{1}));
mbg_asserttrue(isequal(B, [sysStruct.umin ib]));

% PWA system w/ Inf terms in U and with soft input constraints
opt_sincos
probStruct.N = 2;
sysStruct.umax = Inf;
sysStruct.Su = 100;
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
B = yalmip('getbounds', getvariables(V.u{1}));
mbg_asserttrue(isequal(B, [sysStruct.umin ib]));

% LTI systems, bounds can be infinity
Double_Integrator
sysStruct.xmax = [Inf; 10];
sysStruct.xmin = [-4; -Inf];
probStruct.N = 2;
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
B = yalmip('getbounds', getvariables(V.x{1}));
mbg_asserttrue(isequal(B, [sysStruct.xmin sysStruct.xmax]));
