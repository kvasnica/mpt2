function mpt_mpqp_test7

% this is a case where bndA and bndb are empty and we fail to compute Phard as
% polytope(hardA, hardb), because also hardA and hardb are empty. temporary
% workaround is to return Phard as an empty polytope in such case:
% 

load mpqp_noPhard

% as of 04-Dec-2005 mpt_mpqp prints a warning that Phard is an empty polytope
Pn = mpt_mpqp(Matrices);
