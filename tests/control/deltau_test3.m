function deltau_test3

% tests deltaU formulation for systems with boolean inputs

Double_Integrator
probStruct.norm = 2;
probStruct.N = 2;
x0 = [-4; 0];
Options = [];
sysStruct.Uset = [-0.6 -0.3 0 0.3 .6];

%------------------------------------------------------------------------
% deltaU constraints
P = probStruct;
S = sysStruct;
S.dumax = 0.3;
S.dumin = -0.3;
P.tracking = 0;
% explicit scheme shouldn't work because we cannot augment the system dynamics
% such that the state vector contains u(k-1)
lasterr('');
try
    exp = mpt_control(S, P);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
onl = mpt_control(S, P, 'online');
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(co, 49.54750000000001);
% check deltaU constraints
mbg_asserttrue(max(diff(uo)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(uo)) >= S.dumin-1e-10);
% % check displaying of controllers
onl

%------------------------------------------------------------------------
% tracking=1, deltaU constraints
P = probStruct;
S = sysStruct;
S.dumax = 0.3;
S.dumin = -0.3;
P.tracking = 1;
% explicit scheme shouldn't work because we cannot augment the system dynamics
% such that the state vector contains u(k-1)
lasterr('');
try
    exp = mpt_control(S, P);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
% on-line controllers also don't work with tracking=1 since we cannot augment
% the dynamics
lasterr('');
try
    onl = mpt_control(S, P, 'online');
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);


%------------------------------------------------------------------------
% tracking=2, deltaU constraints
P = probStruct;
S = sysStruct;
S.xmax = S.ymax;
S.xmin = S.ymin;
S.dumax = 0.3;
S.dumin = -0.3;
P.tracking = 2;
% explicit scheme shouldn't work because we cannot augment the system dynamics
% such that the state vector contains u(k-1)
lasterr('');
try
    exp = mpt_control(S, P);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
onl = mpt_control(S, P, 'online');
Options.reference = [1; 0];
[xo, uo, yo, co] = sim(onl, x0, 10, Options);
mbg_asserttolequal(co, 98.99500000000002);
% check deltaU constraints
mbg_asserttrue(max(diff(uo)) <= S.dumax+1e-10);
mbg_asserttrue(min(diff(uo)) >= S.dumin-1e-10);
% % check displaying of controllers
onl
