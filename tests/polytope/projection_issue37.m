function projection_issue37

% http://autsun04.ee.ethz.ch:8080/mpt/issue37
% we should not output "all methods failed" if only one option is specified by
% Options.projection

lasterr('');
p = unitbox(2,1);
try
    % use a non-existing projection solver
    q = projection(p, 1, struct('projection', -1));
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'PROJECTION: couldn''t compute projection, selected method failed!');
