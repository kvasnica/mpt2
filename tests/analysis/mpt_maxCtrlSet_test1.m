function mpt_maxCtrlSet_test1

% tests mpt_maxCtrlSet

lti1d
expected_output = unitbox(1, sysStruct.ymax);

% Cinf set is a unitbox in 1D with size of 4, computed in 2 iteration steps
[P, iter] = mpt_maxCtrlSet(sysStruct);
mbg_asserttrue(P==expected_output);
mbg_assertequal(iter, 2);

% Kinf set is a unitbox in 1D with size of 4, computed in 4 iteration steps
[P, iter] = mpt_maxCtrlSet(sysStruct, struct('Kinf', 1));
mbg_asserttrue(P==expected_output);
mbg_assertequal(iter, 4);

% test Options.maxCtr, iterations should be aborted at Options.maxCtr+1
disp('Options.maxCtr = 2');
[P, iter] = mpt_maxCtrlSet(sysStruct, struct('Kinf', 1, 'maxCtr', 2));
mbg_assertequal(iter, 3);
mbg_asserttrue(P==expected_output);

% test calling mpt_maxCtrlSet with just one output argument
P = mpt_maxCtrlSet(sysStruct);
mbg_asserttrue(P==expected_output);
