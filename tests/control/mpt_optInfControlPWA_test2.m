function mpt_optInfControlPWA_test1

% test Options.onlycore and Options.core

% set up CITOC problem
pwa1d
probStruct.norm = 1;
probStruct.subopt_lev = 0;

% compute only the core
coreOpt.onlycore = 1;
T = evalc('core = mpt_control(sysStruct, probStruct, coreOpt);');

% output of the command above must not contain the string 'Exploring the
% state-space', since we only asked for the core
space_was_explored = ~isempty(findstr(T, 'Exploring the state-space'));
mbg_assertfalse(space_was_explored);
mbg_assertequal(length(core.Pn), 2);


% test if we can initialize the CITOC solution with a given core
Options.core = core;
T = evalc('ctrl = mpt_control(sysStruct, probStruct, Options);');

% output of the command above must not contain the string 'Generating the
% "core"', since we provide the core explicitly
core_was_generated = ~isempty(findstr(T, 'Generating the "core"'));
mbg_assertfalse(core_was_generated);
mbg_assertequal(length(ctrl), 8);
