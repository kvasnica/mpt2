function modify_test2

% test modify(ctrl, 'removeflat') option:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=bafd0fcda3c316813b4886c25f45a45f7dc53856


load ctrl_flatregions
a = modify(ctrl, 'removeflat');

% that controller has 5 non-flat regions
mbg_assertequal(length(a), 5);

% check if kept regions are really not flat
[x,r] = chebyball(a.Pn);
mbg_asserttrue(all(r > 1e-5));


% also test that we can specify options
a = modify(ctrl, 'removeflat', struct('r_small', 5e-7));

% ctrl_flatregions controller has 7 regions whose chebychev redius is bigger
% than 5e-7
mbg_assertequal(length(a), 7);
