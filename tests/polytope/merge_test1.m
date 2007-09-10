function merge_test1

% merge now calls reduceunion() to decrease the number of regions which
% need to be merged:
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/4c44c05a7ee2

P = [unitbox(2) unitbox(2)];
[Pm, details] = merge(P, struct('use_reduceunion', 1));
mbg_asserttrue(Pm == P(1));
mbg_assertequal(length(Pm), 1);
mbg_assertequal(details.before, length(P));
mbg_assertequal(details.after, 1);
mbg_assertequal(details.alg, 'reduceunion');
