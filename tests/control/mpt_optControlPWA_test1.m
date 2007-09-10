function mpt_optControlPWA_test1

% test CFTOC for 1D systems. there was a bug which throwed an error for 1D
% systems:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=1fe40897a9377db3618a3e0f0a9bafadcff9f8af

% set up CFTOC problem
pwa1d
probStruct.norm = 1;
probStruct.subopt_lev = 0;
probStruct.N = 2;

ctrl=mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl.Pn), 4);
