function mpt_randLTISys_test1

% tests mpt_randLTISys

% create 10 random UNSTABLE LTI systems with 2 states and 1 input:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=7907129f2b33

s = mpt_randLTISys(10,2,1,struct('justUnstable',1));
% test whether 10 systems have been generated in the cell array
mbg_asserttrue(iscell(s));
mbg_assertequal(length(s), 10);
% test whether all systems are really unstable
for ii = 1:length(s),
    e = abs(real(eig(s{ii}.A)));
    mbg_asserttrue(any(e>1));
end


% create 10 random STABLE LTI systems with 2 states and 1 input
s = mpt_randLTISys(10,2,1,struct('enforceStability',1));
% test whether 10 systems have been generated in the cell array
mbg_asserttrue(iscell(s));
mbg_assertequal(length(s), 10);
% test whether all systems are really unstable
for ii = 1:length(s),
    e = abs(real(eig(s{ii}.A)));
    mbg_asserttrue(all(e<1));
end
