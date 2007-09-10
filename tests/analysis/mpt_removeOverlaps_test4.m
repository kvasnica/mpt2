function mpt_removeOverlaps_test4

% tests that we correctly return details.keptParts even when only one partition
% is used in mpt_removeOverlaps:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=77ebadfd57e1

load lti1d_ctrl_N1_norm1

cs = struct(ctrl);

A = mpt_removeOverlaps(ctrl);
B = mpt_removeOverlaps(cs);

% A must be an mptctrl object
mbg_asserttrue(isa(A, 'mptctrl'));
% B must be a structure
mbg_asserttrue(isstruct(B));

% details.keptParts must be correctly set
mbg_assertequal(A.details.keptParts, 1);
mbg_assertequal(B.details.keptParts, 1);
