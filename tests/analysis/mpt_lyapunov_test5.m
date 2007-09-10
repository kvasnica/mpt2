function mpt_lyapunov_test5

% tests that we properly deal with controllers which have as many regions as
% there are dynamics (usually happens when investigating stability of autonomous
% systems converted using "sysStruct"):
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=fa5c9aa9c4a6

% tests PWA systems where only one region contains the origin


Double_Integrator
K = dlqr(sysStruct.A, sysStruct.B, eye(2), 1);
A = sysStruct.A - sysStruct.B*K;
sysStruct.A = {A, A};
sysStruct.B = {zeros(2, 1), zeros(2, 1)};
sysStruct.C = {eye(2), eye(2)};
sysStruct.D = {zeros(2, 1), zeros(2, 1)};
sysStruct.guardX = {[1 0], [-1 0]};
sysStruct.guardC = {1, -1};

% PWP lyapunov function of order 4 does exists (even with order 2)
ctrl = mpt_lyapunov(sysStruct, 'pwp');
mbg_asserttrue(ctrl.details.lyapunov.feasible);
mbg_asserttrue(isequal(lower(ctrl.details.lyapunov.type), 'pwp'));

% SOS lyapunov function of order 4 does exists (even with order 2)
% following patch is needed:
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=592f88c28215
ctrl = mpt_lyapunov(sysStruct, 'sos', 2);
mbg_asserttrue(ctrl.details.lyapunov.feasible);
mbg_asserttrue(isequal(lower(ctrl.details.lyapunov.type), 'sos'));

% Common quadratic lyapunov function exists
ctrl = mpt_lyapunov(sysStruct, 'quad');
mbg_asserttrue(ctrl.details.lyapunov.feasible);
mbg_asserttrue(isequal(lower(ctrl.details.lyapunov.type), 'quadratic'));

% PWQ function should be found
ctrl = mpt_lyapunov(sysStruct, 'pwq');
mbg_asserttrue(ctrl.details.lyapunov.feasible);
mbg_asserttrue(isequal(lower(ctrl.details.lyapunov.type), 'pwq'));
