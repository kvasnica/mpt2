function mpt_mplp_test3

% tests whether we properly include Pbnd when constructing regions:
%

load unbound_prob

% without the above patch mpt_mplp would return an unbounded solution
Pn = mpt_mplp(Matrices);
mbg_asserttrue(isbounded(Pn));
