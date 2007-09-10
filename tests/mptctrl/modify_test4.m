function modify_test4

% if mpt_onvariantSet() is used, the controller object can have wrong dimension
% of ctrl.details.regionHorizon. check that we cope with such situation
% correctly:

load ctrl1
ind_remove = [1 2 5];

% check activeConstraints having wrong dimension
cs = struct(ctrl);
cs.details.activeConstraints = {[], []};
ctrl2 = mptctrl(cs);
cm = modify(ctrl2, 'remove', ind_remove);

% check IterStore having wrong dimension
cs = struct(ctrl);
cs.details.IterStore = [1 2];
ctrl2 = mptctrl(cs);
cm = modify(ctrl2, 'remove', ind_remove);

% check regionHorizon having wrong dimension
cs = struct(ctrl);
cs.details.regionHorizon = [1 2];
ctrl2 = mptctrl(cs);
cm = modify(ctrl2, 'remove', ind_remove);
