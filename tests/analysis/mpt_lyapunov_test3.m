function mpt_lyapunov_test3

% tests that we properly deal with controllers which have as many regions as
% there are dynamics (usually happens when investigating stability of autonomous
% systems converted using "sysStruct"):
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=fa5c9aa9c4a6

% tests LTI systems


% make an autonomous system, stabilize it using an LQR feedback
Double_Integrator
k = dlqr(sysStruct.A, sysStruct.B, eye(2), 1);
sysStruct.A = sysStruct.A - sysStruct.B*k;
sysStruct.B = [0;0];

% PWP lyapunov function of order 4 does exists (even with order 2)
ctrl = mpt_lyapunov(sysStruct, 'pwp');
mbg_asserttrue(ctrl.details.lyapunov.feasible);
mbg_asserttrue(isequal(lower(ctrl.details.lyapunov.type), 'pwp'));

% SOS lyapunov function of order 4 does exists (even with order 2)
ctrl = mpt_lyapunov(sysStruct, 'sos', 2);
mbg_asserttrue(ctrl.details.lyapunov.feasible);
mbg_asserttrue(isequal(lower(ctrl.details.lyapunov.type), 'sos'));

% Common quadratic lyapunov function exists, needs following patch:
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=f9ad2223113d
ctrl = mpt_lyapunov(sysStruct, 'quad');
mbg_asserttrue(ctrl.details.lyapunov.feasible);
mbg_asserttrue(isequal(lower(ctrl.details.lyapunov.type), 'quadratic'));

% PWQ function should be found, needs following patch:
% http://control.ee.ethz.ch/~mpt/hg/mpt-stm?cs=2c7c8f2cd889
ctrl = mpt_lyapunov(sysStruct, 'pwq');
mbg_asserttrue(ctrl.details.lyapunov.feasible);
mbg_asserttrue(isequal(lower(ctrl.details.lyapunov.type), 'pwq'));
