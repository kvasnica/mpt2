function mptctrl_test3

% we should reject conversion of nonlinear systems:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=987869e88c1d

% expecting an error:
sysStruct = mpt_sys(@duffing_oscilator);
lasterr('');
try
    ctrl = mptctrl(sysStruct);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'MPTCTRL: nonlinear systems cannot be converted.');
