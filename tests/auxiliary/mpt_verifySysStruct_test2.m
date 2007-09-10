function mpt_verifySysStruct_test2

% tests Options.ybounds_optional:
% http://control.ee.ethz.ch/~mpt/hg/mpt-yalmip?cs=93bbd7b2f2c5

Double_Integrator
sysStruct = rmfield(sysStruct, 'ymax');
sysStruct = rmfield(sysStruct, 'ymin');

fprintf('This one should complain about missing output/state constraints:\n');
lasterr('');
try
    sst = mpt_verifySysStruct(sysStruct);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);

fprintf('\nThis one should work:\n');
lasterr('');
sysStruct.xmax = [1;1]; sysStruct.xmin = [-1;-1];
try
    sst = mpt_verifySysStruct(sysStruct, struct('ybounds_optional', 1));
    worked = 1;
    disp('ok');
catch
    worked = 0;
end
disp(lasterr);
mbg_asserttrue(worked);


% lets also test mpt_verifySysProb for the same issue
fprintf('\nThis one should work:\n');
lasterr('');
try
    [sst, pst] = mpt_verifySysProb(sysStruct, probStruct, struct('ybounds_optional', 1));
    worked = 1;
    disp('ok');
catch
    worked = 0;
end
disp(lasterr);
mbg_asserttrue(worked);
