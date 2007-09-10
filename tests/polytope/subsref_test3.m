function subsref_test3

% tests indexing of polytopes by logicals

u1 = unitbox(2)*1;
u2 = unitbox(2)*2;
u3 = unitbox(2)*3;

%-------------------------------------------------------------
p = [u1 u2 u3];
q = p(logical([0 1 1]));
mbg_assertequal(length(q), 2);
mbg_asserttrue(q(1) == u2);
mbg_asserttrue(q(2) == u3);


%-------------------------------------------------------------
p = u1;
q = p(logical(1));
mbg_assertequal(length(q), 1);
mbg_asserttrue(p == q);


%-------------------------------------------------------------
p = u1;
q = p(logical(0));
mbg_assertequal(length(q), 0);
mbg_assertfalse(isfulldim(q));


%-------------------------------------------------------------
p = u1;
q = p(logical([0 0 0]));
mbg_assertequal(length(q), 0);
mbg_assertfalse(isfulldim(q));


%-------------------------------------------------------------
p = u1;
try
    q = p(logical([0 0 1]));
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
