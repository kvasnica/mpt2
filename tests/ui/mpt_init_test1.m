function mpt_init_test1

% should work:
mpt_init('rehash');
mpt_init('lpsolver','linprog');
mpt_init('qpsolver','quadprog');
mpt_init('extreme_solver','cdd','lpsolver','cdd')
mpt_init('extreme_solver','matlab')

% should not work:
try
    mpt_init('lpsolver', 'mexpress');
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);

% set CDD as default LP solver
mpt_init('lpsolver','cdd')
