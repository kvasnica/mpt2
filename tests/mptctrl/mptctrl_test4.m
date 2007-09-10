function mptctrl_test4

% tests that we properly recognize non-autonomous systems even though the B
% matrix has all zero elements, but guardU contains non-zero elements:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=5abce6f7cb89

fprintf('System is not autonomous if it has non-zero guards on U:\n');
opt_sincos
[sysStruct.B{:}] = deal(zeros(2, 1));
sysStruct.guardU{1} = 1;
sysStruct.guardU{2} = 0;
lasterr('');
try
    ctrl = mptctrl(sysStruct);
    worked = 1;
catch
    worked = 0;
end

disp(lasterr);
mbg_assertfalse(worked);
