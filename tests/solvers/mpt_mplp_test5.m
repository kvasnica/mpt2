function mpt_mplp_test5

% for this example we have been getting huge holes in the solution. fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=5e9f3686b61e

load mplp_hole
[Pn, Fi, Gi, AC, Phard] = mpt_mplp(R);

% look at plot(Pn), we used to have a huge hole in the solution!

mbg_asserttrue(Pn==Phard);
mbg_asserttrue(length(Pn)>17);
