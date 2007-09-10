function mpt_verifySysProb_test2

% verifies that Options.useyalmip works:
% http://control.ee.ethz.ch/~mpt/hg/mpt-yalmip?cs=2f73b9f6d5fc

opt_sincos
probStruct.N = 5;
probStruct.Nc = 2;

%fprintf('This one should give an error:\n');
lasterr('');
try
    [sst, pst] = mpt_verifySysProb(sysStruct, probStruct);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Control horizon is only supported for quadratic cost function.');


%fprintf('\nThis one should work:\n');
lasterr('');
[sst, pst] = mpt_verifySysProb(sysStruct, probStruct, struct('useyalmip', 1));


% probStruct.inputblocking must only be set if Options.useyalmip=0
mbg_assertfalse(isfield(pst, 'inputblocking'));
