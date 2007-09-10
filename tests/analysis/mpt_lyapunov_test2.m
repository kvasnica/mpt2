function mpt_lyapunov_test2

% tests mpt_lyapunov with controllers for which no lyapunov function exists:
% http://control.ee.ethz.ch/~mpt/hg/mpt-25x?cs=347e877f2f0c

lti1d
sysStruct.B = 0;
probStruct.norm = 1;

ctrl = mpt_control(sysStruct, probStruct);

L=mpt_lyapunov(ctrl, 'pwp');
L=mpt_lyapunov(ctrl, 'pwa');
L=mpt_lyapunov(ctrl, 'pwq');
L=mpt_lyapunov(ctrl, 'quad');
L=mpt_lyapunov(ctrl, 'sos');
