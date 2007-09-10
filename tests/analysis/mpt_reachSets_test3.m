function mpt_reachSets_test3

% tests following patch:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=2220beff0194

pwa_car
U0=polytope([0; 1]);
X0=polytope([-3.5 0; -3.49, 0; -3.49, 0.01; -3.5 0.01]);
n = 3;
opt.verbose = 2;
Psets = mpt_reachSets(sysStruct, X0, U0, n, opt);

mbg_assertequal(length(Psets), 3);
