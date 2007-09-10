function tracking_test12

% When tracking is used, the state vector is extended by the u(k-1) element
% and input constraints are introduced into the augmented state/output
% constraints. Due to numeriucs, it could however happen that such
% constraints are violated (say the optimization returns 1.00001, violating
% the input bounds of +/-1). In such case the simulation functions would
% break with a "no feasible control law found" message. To prevent such
% problems, we expand the input constraints by a factor of 1+1e-6.

T = 10;
oscilator;
probStruct.tracking = 1; probStruct.Qy = 10;
sysStruct.xmax = [10; 10]; sysStruct.xmin = [-10; -10];
ctrl = mpt_control(sysStruct, probStruct, 'online');
[x, u, y] = sim(ctrl, [2;2], T, struct('reference', 1));
if length(y) ~= T
    warning('test failed!');
end
% mbg_assertequal(length(y), T);
% mbg_asserttolequal(y(end), 1, 1e-1);
