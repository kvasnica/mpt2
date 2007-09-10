function mpt_computeTrajectory_test1

% tests computeTrajectory() for various control problems
load ctrl3d_data

x0 = [0.1; 0; 0];
[X,U,Y] = sim(ctrl_ny3nx3nu2, x0);
mbg_asserttolequal(X(end, :), [0 0 0], 1e-2);
mbg_asserttrue(size(X, 1) < 10);

[X,U,Y] = sim(ctrl_ny3nx3nu2_tracking1, x0, [], struct('reference', [0; 0; 0]));
mbg_asserttolequal(X(end, :), [0 0 0], 1e-2);
mbg_asserttrue(size(X, 1) < 10);

[X,U,Y] = sim(ctrl_ny3nx3nu2_tracking2, x0, [], struct('reference', [0; 0; 0]));
mbg_asserttolequal(X(end, :), [0 0 0], 1e-2);
mbg_asserttrue(size(X, 1) < 10);

[X,U,Y] = sim(ctrl_ny3nx3nu2_xref, x0, 10);
mbg_assertequal(size(X, 1), 11);

[X,U,Y] = sim(ctrl_ny3nx3nu2_yref, x0, 10);
mbg_assertequal(size(X, 1), 11);

[X,U,Y] = sim(ctrl_ny1nx3nu2_du, x0);
mbg_asserttolequal(X(end, :), [0 0 0], 1e-2);
mbg_asserttrue(size(X, 1) < 10);

[X,U,Y] = sim(ctrl_ny1nx3nu2_tracking1, x0, [], struct('reference', 0.5));
mbg_asserttolequal(Y(end, :), [0.5], 1e-2);
mbg_assertequal(size(Y, 1), 21);

[X,U,Y] = sim(ctrl_ny1nx3nu2_tracking2, x0, 10, struct('reference', 0.5));
mbg_assertequal(size(Y, 1), 10);

[X,U,Y] = sim(ctrl_ny1nx3nu2_yref , x0, 10);
mbg_assertequal(size(Y, 1), 10);
