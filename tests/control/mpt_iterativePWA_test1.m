function mpt_iterativePWA_test1

% tests different settings of Options.lyapunov_type:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=02f93efda25dc716af5a54663343f2dfa39179a9

pwa1d
% set up one-step problem
probStruct.subopt_lev = 2;
probStruct.norm = 1;
probStruct.N = 1;

% no lyapunov function should be computed
ctrl = mpt_control(sysStruct, probStruct, struct('lyapunov_type', 'none'));
mbg_assertequal(ctrl.details.PWATime, 0);
mbg_assertequal(ctrl.details.PWQTime, 0);

% compute PWQ Lyapunov function
ctrl = mpt_control(sysStruct, probStruct, struct('lyapunov_type', 'pwq'));
mbg_asserttrue(ctrl.details.PWQlyapQ);

% compute PWA Lyapunov function
ctrl = mpt_control(sysStruct, probStruct, struct('lyapunov_type', 'pwa'));
mbg_asserttrue(ctrl.details.PWAlyapL);

% compute any Lyapunov function (first try PWQ, then PWA). in this example PWQ
% can be found
ctrl = mpt_control(sysStruct, probStruct, struct('lyapunov_type', 'any', 'lpsolver', 3));
mbg_asserttrue(ctrl.details.PWQlyapQ);

% also test if result is correct (6 regions with NAG, 4 regions with CDD)
mbg_asserttrue((length(ctrl)==6)|(length(ctrl)==4));
