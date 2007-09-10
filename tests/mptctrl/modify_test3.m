function modify_test3

% test if we get correct second output:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=2849163734bc

load ctrl_flatregions
ind_remove = [1 2 5];
ind_pick = [7 4 2];

[a, i] = modify(ctrl, 'remove', ind_remove);
mbg_assertequal(i, setdiff(1:length(ctrl), ind_remove));

[a, i] = modify(ctrl, 'pick', ind_pick);
mbg_assertequal(i, [2 4 7]);

[a, i] = modify(ctrl, 'removeflat');
mbg_assertequal(i, [1 2 3 4 5]);
