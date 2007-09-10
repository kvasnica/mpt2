function mpt_mplp_test4

% tests that we prevent cycling in mpt_mplp_ver7

% this example used to cycle (i.e. never finish). seems to be fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3583353cd0a2

load mplp_cycling1
M = MatricesReduced;
Pfeas = projection(polytope([-M.E M.G], M.W), 1:2);
Pn = mpt_mplp(M);
mbg_assertequal(length(Pn), 4);
% check Pn with respect to feasible set obtained by projection
mbg_asserttrue(Pn==Pfeas);



% this was also behaving rather strange, seems to be fixed by
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3583353cd0a2
load mplp_cycling2
Pfeas = projection(polytope([-R.E R.G], R.W), 1:2);
Pbnd = polytope(R.bndA, R.bndb);
Pn = mpt_mplp(R);
mbg_assertequal(length(Pn), 4);
% check Pn with respect to feasible set obtained by projection
mbg_asserttrue(Pn==(Pfeas & Pbnd));



% same cycling problems, also appears to be fixed by
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3583353cd0a2
%
% however it only works with NAG

opt = mpt_options;
if ~ismember(0, opt.solvers.lp)
    % no NAG, skip this test
    warning('This test requires NAG');
    return
end

load mplp_cycling3
R = MatricesReduced;
Pfeas = projection(polytope([-R.E R.G], R.W), 1:2);
Pn = mpt_mplp(R);
mbg_assertequal(length(Pn), 13);
% check Pn with respect to feasible set obtained by projection
mbg_asserttrue(Pn==Pfeas);
