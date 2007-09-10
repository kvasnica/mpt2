function mpt_ownmpc_test12

% tests whether we properly reject sigmonial constraints and objectives:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=8eafaa43e22f

Double_Integrator
probStruct.N = 2;
probStruct.Tconstraint = 0;
probStruct.P_N = zeros(2);
[C, O, V] = mpt_ownmpc(sysStruct, probStruct);

%fprintf('Sigmonial constraints should be rejected:\n');
F = C + set(1/V.x{1}(1) <= 1);  % sigmonial constraint
lasterr('');
try
    ctrl = mpt_ownmpc(sysStruct, probStruct, F, O, V);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Sigmonial constraints (e.g. 1/x <= a) not supported.');


%fprintf('\nSigmonial objectives should be rejected:\n');
O = O + 1/V.x{1}(1);  % sigmonial objective
lasterr('');
try
    ctrl = mpt_ownmpc(sysStruct, probStruct, C, O, V);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Sigmonial objectives not supported.');
