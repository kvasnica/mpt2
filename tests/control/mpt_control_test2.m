function mpt_control_test2

% control horizon should now be supported for linear costs and for PWA systems:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=cedc93418d40
%
% moreover, here we also test that warning from implies() are prevented by
% bounding all variables:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=c5e02764e626

Double_Integrator;
probStruct.norm = 1;
probStruct.Tconstraint = 0;
probStruct.Nc = 2;
ctrl = mpt_control(sysStruct, probStruct, 'online');


opt_sincos
probStruct.N = 5;
probStruct.Nc = 2;
ctrl = mpt_control(sysStruct, probStruct, 'online');
