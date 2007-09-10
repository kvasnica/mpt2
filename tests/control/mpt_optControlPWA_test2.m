function mpt_optControlPWA_test2

% test CFTOC for 1D systems with the "old" approach, i.e. without PWA cost-to-go

% set up CFTOC problem
pwa1d
probStruct.norm = 1;
probStruct.subopt_lev = 0;
probStruct.N = 2;

% due to an old mplp solver we need to switch to mpt_optControlPWAold()
Options.mplpver = 1;
ctrl=mpt_control(sysStruct, probStruct, Options);
mbg_assertequal(length(ctrl.Pn), 6);
