function mpt_optControlPWA_test4

% tests whether we return correct ctrl.Pfinal:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3a5b598ba7f5

pwa1d
probStruct.subopt_lev = 0;
probStruct.N = 2;
probStruct.norm = 1;

% before we have been returning ctrl.Pfinal as an empty polytope
ctrl = mpt_control(sysStruct, probStruct);

% now we return it as a polytope array
mbg_assertequal(length(ctrl.Pfinal), 4);

% ctrl.Pfinal has to be a union of ctrl.Pn
mbg_asserttrue(union(ctrl.Pn)==union(ctrl.Pfinal));
