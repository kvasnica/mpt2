function regiondiff_test1

% tests whether we properly deal with Options.infbox larger than
% mptOptions.infbox:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=0b08c5cc76fe

opt.infbox=1e7; % try to use infbox in regiondiff
                 % bigger than mptOptions.infbox
qw=polytope([1;-1],[inf;inf],1,1); % R^1
as=polytope([1;-1],[1;1]);
zx=regiondiff(qw,as,opt); % this generated an error prior to Nov-10-2005

mbg_assertequal(length(zx), 2);
