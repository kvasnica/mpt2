function mpt_mpqp_test2

% tests that we properly detect infeasible problems:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=42c743cf45db

fprintf('This problem should be infeasible from the begining:\n\n');
load mpqp_infeasible
[Pn, Fi, Gi] = mpt_mpqp(Matrices);
mbg_asserttrue(~isfulldim(Pn));
mbg_asserttrue(isempty(Fi));
mbg_asserttrue(isempty(Gi));
