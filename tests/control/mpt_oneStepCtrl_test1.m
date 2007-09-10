function mpt_oneStepCtrl_test1

% tests Options.useprojection = 0/1:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=3bb98cb9fa613be8129da834db35976e350f603d

lti1d

c1 = mpt_oneStepCtrl(sysStruct, probStruct);
% the resulting controller should have 5 regions
mbg_assertequal(length(c1.Pn), 5);

% now test Options.useprojection = 0, in which case we solve multi-parametric
% programs to obtain feasible sets
c2 = mpt_oneStepCtrl(sysStruct, probStruct, struct('useprojection', 0));
% the resulting controller should have 5 regions
mbg_assertequal(length(c2.Pn), 5);

% the two controllers must be identical
mbg_asserttrue(c1.Pn==c2.Pn);
mbg_asserttrue(c1.Pfinal==c2.Pfinal);
