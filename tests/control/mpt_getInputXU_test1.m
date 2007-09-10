function mpt_getInputXU_test1

% test the XU functionality

load ctrlXU_1d

% first compute optimal control law
uopt = mpt_getInput(ctrlXU, 0.2);
mbg_asserttolequal(uopt, -0.2);

% test mpt_getInputXU with the 'random' strategy
Options.strategy = 'rand';
urand1 = mpt_getInputXU(ctrlXU, 0.2, Options);
urand2 = mpt_getInputXU(ctrlXU, 0.2, Options);
% the two inputs should not be equal, since they are random
mbg_asserttrue(urand1~=urand2);


% now test Options.useXU for mpt_getInput()
Options.useXU = 1;
Options.strategy = 'rand';
urand1 = mpt_getInput(ctrlXU, 0.2, Options);
urand2 = mpt_getInput(ctrlXU, 0.2, Options);
% the two inputs should not be equal, since they are random
mbg_asserttrue(urand1~=urand2);
