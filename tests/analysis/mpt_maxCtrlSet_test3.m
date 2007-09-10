function mpt_maxCtrlSet_test3

% tests that we cope with custom Options.{probStruct|Q|R|Tset} correctly:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=f7d32f56741b23f58250f477befcb728678b051b

lti1d
expected_output = unitbox(1, sysStruct.ymax);

% use custom probStruct
probStruct.R = 10*probStruct.R;
Options.probStruct = probStruct;

% Cinf set is a unitbox in 1D with size of 4, computed in 2 iteration steps
[P, iter] = mpt_maxCtrlSet(sysStruct, Options);
mbg_asserttrue(P==expected_output);
mbg_assertequal(iter, 2);

% Kinf set is a unitbox in 1D with size of 4, computed in 2 iteration steps for
% a custom choice of probStruct.R
Options.Kinf = 1;
[P, iter] = mpt_maxCtrlSet(sysStruct, Options);
mbg_asserttrue(P==expected_output);
mbg_assertequal(iter, 2);

%======================================================================
% now test Options.Tset

clear Options
Options.Tset = unitbox(1, 0.2);
Options.Kinf = 1;   % Options.Tset makes only sense for Kinf computation
[P, iter] = mpt_maxCtrlSet(sysStruct, Options);
mbg_asserttrue(P==expected_output);
mbg_assertequal(iter, 5);      % with custom Options.Tset we need 5 iterations


%======================================================================
% now test Options.R

clear Options
Options.R = probStruct.R;
Options.Kinf = 1;
[P, iter] = mpt_maxCtrlSet(sysStruct, Options);
mbg_asserttrue(P==expected_output);
mbg_assertequal(iter, 2);      % with custom Options.R we need 2 iterations
