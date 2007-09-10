function horzcat_test1

% tests whether we properly disable concatenation and indexed assignments for
% MPTCTRL objects:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=14bc5e6a1fc8

ctrl = mptctrl;

%disp('Testing horzcat()');
try
    C = [ctrl ctrl];
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Horizontal concatentation is not supported, please use cell arrays instead.');

%disp('Testing vertcat()');
try
    C = [ctrl; ctrl];
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Vertical concatentation is not supported, please use cell arrays instead.');

%disp('Testing subsasgn()');
try
    ctrl(1) = ctrl;
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Indexed assignment for MPTCTRL objects is not supported.');
