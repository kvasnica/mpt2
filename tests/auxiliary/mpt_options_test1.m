function mpt_options_test1

% test if we can retrieve individual fields of mptOptions:

% this must work
initlpsolver = mpt_options('lpsolver');

% now set linprog as lp solver
mpt_options('lpsolver', 'linprog');

% check if linprog is selected
lpsolver = mpt_options('lpsolver');
mbg_assertequal(lpsolver, 1);

% now set back original lp solver
mpt_options('lpsolver', initlpsolver);

% check if the LP solver was set correctly
lpsolver = mpt_options('lpsolver');
mbg_assertequal(lpsolver, initlpsolver);

% test that we throw an error if a non-existent field is specified
try
    o = mpt_options('aaa');
    worked = 1;
catch
    worked = 0;
end
% mpt_options('aaa') must give an error
mbg_assertfalse(worked);


% test that we throw an error if a non-existent field is specified
try
    o = mpt_options('aaa', 1);
    worked = 1;
catch
    worked = 0;
end
% mpt_options('aaa') must give an error
mbg_assertfalse(worked);



% just to be sure - set CDD as LP solver
mpt_options('lpsolver', 'cdd');


