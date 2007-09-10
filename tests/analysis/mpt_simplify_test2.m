function mpt_simplify_test2

% controllers with overlaps which cannot be removed should be rejected:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=9110d1fe79c9

load ctrl_1d_searchtree
ctrlStruct = struct(ctrl_st);

% 2-norm/overlaps => reject
lasterr('');
ctrl = ctrlStruct;
ctrl.overlaps = 1;
ctrl.probStruct.norm = 2;
ctrl = mptctrl(ctrl);
try
    mpt_simplify(ctrl);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);

% 1-norm/overlaps => should remove overlaps
ctrl = ctrlStruct;
ctrl.overlaps = 1;
ctrl.probStruct.norm = 1;
[ctrl.Ai{:}] = deal(0);
ctrl = mptctrl(ctrl);
mpt_simplify(ctrl);

% 2-norm/no overlaps => should work
ctrl = ctrlStruct;
ctrl.overlaps = 0;
ctrl.probStruct.norm = 2;
ctrl = mptctrl(ctrl);
mpt_simplify(ctrl);

% 2-norm/no overlaps => should work
ctrl = ctrlStruct;
ctrl.overlaps = 0;
ctrl.probStruct.norm = 2;
ctrl = mptctrl(ctrl);
mpt_simplify(ctrl);
