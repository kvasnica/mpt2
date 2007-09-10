function mpt_getInput_test3

% tests whether we give nice error messages when dimension of x0 is wrong:
% http://control.ee.ethz.ch/~mpt/hg/mpt-25x?cs=1d9c35868c0a

Double_Integrator
probStruct.tracking = 1;
ctrl = mpt_control(sysStruct, probStruct, 'online');

fprintf('mpt_getInput should nicely complain about wrong dimension of x0:\n');
lasterr('');
try
    u = mpt_getInput(ctrl, [1;1;1]);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Wrong size of "Options.reference". Expecting 2 elements, got 5.');
