function projection_test10

% the mplp-based method should not work if no QP solver is available (because
% then we can't use a reliable mplp solver)

p = unitbox(2,1);
lasterr('');
try
    q = projection(p, 1, struct('projection', 7, 'qpsolver', -1));
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'PROJECTION: couldn''t compute projection, selected method failed!');
