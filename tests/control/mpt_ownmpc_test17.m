function mpt_ownmpc_test17

% tests that we properly reject suboptimal strategies in mpt_ownmpc():
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=d118ec6a238b

%fprintf('Expecting an error due to N=Inf:\n');
Double_Integrator
probStruct.N = Inf;
probStruct.subopt_lev = 0;
lasterr('');
try
    [C, O, V] = mpt_ownmpc(sysStruct, probStruct);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Prediction horizon must not be infinity.');


%fprintf('\nExpecting an error due to subopt_lev>0:\n');
Double_Integrator
probStruct.N = 1;
probStruct.subopt_lev = 2;
lasterr('');
try
    [C, O, V] = mpt_ownmpc(sysStruct, probStruct);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Suboptimal strategies not supported by this function.');
