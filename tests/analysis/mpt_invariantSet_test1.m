function mpt_invariantSet_test1

% tests how we cope with lower-dimensional noise in V-representation
load ctrl_lowdimnoise

invctrl = mpt_invariantSet(ctrl);
mbg_assertequal(length(invctrl), 7);
