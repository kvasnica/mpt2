function mpt_removeOverlaps_test5

% there was a bug in mpt_removeOverlaps() causing details.keptParts to be always
% set to empty. fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt-25x?cs=658cf8407683

load removeovlps_bug1
a = mpt_removeOverlaps(S);

mbg_assertequal(a.details.keptParts, [1 2]);

% 6?6?6?3=24
% 13 3 27 2 = 24