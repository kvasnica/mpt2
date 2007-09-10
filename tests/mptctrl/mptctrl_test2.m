function mptctrl_test2

% tests conversion of sysStruct objects into dummy controller partitions:
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=e77b1d046d04

%fprintf('This one should give an error because the system is not autonomous:\n');
Double_Integrator
lasterr('');
try
    ctrl = mptctrl(sysStruct);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'MPTCTRL: only autonomous systems can be converted.');


%fprintf('\nThis one should work, linear systems should result in 1 region:\n');
Double_Integrator
sysStruct.B = zeros(2, 1);
ctrl = mptctrl(sysStruct);
mbg_assertequal(length(ctrl.Pn), 1);


%fprintf('\nThis one should work, this PWA system should result in 2 regions:\n');
opt_sincos
[sysStruct.B{:}] = deal(zeros(2, 1));
ctrl = mptctrl(sysStruct);
mbg_assertequal(length(ctrl.Pn), 2);
