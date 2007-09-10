function mpt_ownmpc_test11

Double_Integrator
probStruct.N = 2;
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);
C = C + set(V.x{end}'*V.x{end} <= 1);  % nonlinear constraint

% no explicit solution for nonlinear constraints
fprintf('Here we should get an error:\n');
lasterr('');
try
    ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);

% on-line version should work
fprintf('\nOn-line version should give a warning:\n');
ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V, 'online');
% and the constraints should be stored in ctrl.details.yalmipData
mbg_asserttrue(isfield(ctrl.details, 'yalmipData'));

% the problem is solved using solvesdp() in mpt_getInput
u_mpt = ctrl([1; 1], struct('openloop', 1, 'nlsolver', 'local', 'convertconvex', 1, 'verbose', 0));
% compare it to pure call to solvesdp()
solvesdp(C + set(V.x{1}==[1;1]), O, sdpsettings('verbose', 0));
u_yalmip = double([V.u{:}])';
mbg_asserttolequal(u_mpt, [-1; -0.5282], 1e-4);
mbg_assertequal(u_mpt, u_yalmip);
