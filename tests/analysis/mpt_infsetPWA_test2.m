function mpt_infsetPWA_test2

% tests mpt_infsetPWA() for cases where the control law u = Fi*x+Gi is
% defined for the whole prediction horizon (i.e. the Fi and Gi matrices
% have "N*nu" rows instead of just "nu".

lti1d
probStruct.norm = 2; probStruct.N = 3;
probStruct.Tconstraint = 0;
ctrl = mpt_control(sysStruct, probStruct);

% make sure the control law is defined for the whole prediction horizon
nu = size(sysStruct.B, 2);
mbg_assertequal(size(ctrl.Fi{1}, 1), nu*probStruct.N);

% an unfixed mpt_infsetPWA() would break with the "Inner matrix dimensions
% must agree" error
[Pn, dyns, invctrl] = mpt_infsetPWA(ctrl);

% make sure the output is really a controller object
mbg_assertequal(class(invctrl), 'mptctrl');

% was the controller object updated correctly?
mbg_assertequal(length(Pn), length(invctrl.Pn));
mbg_asserttrue(Pn == invctrl.Pn);
