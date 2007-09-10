function sub_computeTrajectory_test4

% test computation of closed-loop cost for various setups
% only an LTI system w/ 2-norm cost is tested here

load ctrl_data

% tracking=0, dumode=0
ctrl = ctrl_tr0_du0;
[x, u, y, d, c] = sub_computeTrajectory(ctrl, [-1;0], 10, []);
mbg_asserttolequal(c, 1.8085, 1e-4);

% tracking=0, dumode=0, yref
ctrl = ctrl_tr0_du0_yref;
[x, u, y, d, c] = sub_computeTrajectory(ctrl, [-1;0], 10, []);
mbg_asserttolequal(c, 6.4548, 1e-4);

% tracking=0, dumode=0, xref
ctrl = ctrl_tr0_du0_xref;
[x, u, y, d, c] = sub_computeTrajectory(ctrl, [-1;0], 10, []);
mbg_asserttolequal(c, 7.2396, 1e-4);

% tracking=0, dumode=1
ctrl = ctrl_tr0_du1;
[x, u, y, d, c] = sub_computeTrajectory(ctrl, [-1;0], 10, []);
mbg_asserttolequal(c, 1.8184, 1e-4);

% tracking=0, dumode=1, yref
ctrl = ctrl_tr0_du1_yref;
[x, u, y, d, c] = sub_computeTrajectory(ctrl, [-1;0], 10, []);
mbg_asserttolequal(c, 6.1115, 1e-4);

% tracking=0, dumode=1, xref
ctrl = ctrl_tr0_du1_xref;
[x, u, y, d, c] = sub_computeTrajectory(ctrl, [-1;0], 10, []);
mbg_asserttolequal(c, 7.2735, 1e-4);

% probStruct.tracking = 1
ctrl = ctrl_tr1;
[x, u, y, d, c] = sub_computeTrajectory(ctrl, [-1;0], 10, struct('reference', 1));
mbg_asserttolequal(c, 6.1115, 1e-4);

% probStruct.tracking = 2
ctrl = ctrl_tr2;
[x, u, y, d, c] = sub_computeTrajectory(ctrl, [-1;0], 10, struct('reference', 1));
mbg_asserttolequal(c, 6.4548, 1e-4);
