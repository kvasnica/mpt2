function mpt_optInfControlPWA_test1

% test infinite-time solution for PWA systems

% set up CITOC problem
pwa1d
probStruct.norm = 1;
probStruct.subopt_lev = 0;
ctrl=mpt_control(sysStruct, probStruct);
mbg_assertequal(length(ctrl.Pn), 8);
