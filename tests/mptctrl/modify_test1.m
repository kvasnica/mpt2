function modify_test1

load ctrl1

% test removing of regions
ind_remove = [1 2 5];
ind_keep = setdiff(1:length(ctrl), ind_remove);
a = modify(ctrl, 'remove', ind_remove);
mbg_assertequal(a.Pn, ctrl.Pn(ind_keep));
% after removing regions, modify() should set Pfinal=Pn
mbg_assertequal(length(a.Pn), length(a.Pfinal));

% test picking of regions
ind_pick = [7 4 2];
a = modify(ctrl, 'pick', ind_pick);
mbg_assertequal(length(a.Pn), 3);
mbg_assertequal(a.Pn, ctrl.Pn(ind_pick));
