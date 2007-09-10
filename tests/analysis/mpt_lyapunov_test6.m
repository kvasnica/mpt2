function mpt_lyapunov_test6

% tests whether we can call mpt_lyapunov() directly with a sysStruct input:
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=52e610b2cd3e


Double_Integrator

%fprintf('Input is an invalid sysStruct, should give an error:\n');
lasterr('');
try
    c = mpt_lyapunov(sysStruct, 'quad');
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'MPTCTRL: only autonomous systems can be converted.');


%fprintf('\nInput is an invalid structure, should give an error:\n');
lasterr('');
try
    c = mpt_lyapunov(probStruct, 'quad');
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'MPT_LYAPUNOV: First input argument must be a valid MPTCTRL object!');


%fprintf('\nInput is a valid sysStruct:\n');
% make an autonomous LTI system:
k = dlqr(sysStruct.A, sysStruct.B, eye(2), 1);
sysStruct.A = sysStruct.A - sysStruct.B*k;
sysStruct.B = [0;0];
c = mpt_lyapunov(sysStruct, 'quad', struct('verbose', 0));
% quadratic lyapunov function should have been found
mbg_asserttrue(c.details.lyapunov.feasible);
mbg_assertequal(c.details.lyapunov.type, 'quadratic');

