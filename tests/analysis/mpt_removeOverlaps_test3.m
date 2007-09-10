function mpt_removeOverlaps_test3

% tests mpt_removeOverlaps() on minimum-time solutions (remember that we use
% different approach there)

load lti1d_mintime

a = mpt_removeOverlaps(ctrl);
mbg_assertequal(length(a), 7);

% check that polytopes do not overlap
[r,ia,ib,iab]=dointersect(a.Pn, a.Pn);
mbg_assertequal(iab, [(1:7)' (1:7)']);
