function subsref_test2

% tests indexing like p([1 1 2 2 1])

%-------------------------------------------------------------
%fprintf('should work\n');
p = unitbox(2);
q = p([1 1 1]);
mbg_assertequal(length(q), 3);
for i = 1:length(q),
    mbg_asserttrue(q(i) == p);
end


%-------------------------------------------------------------
%fprintf('\n\nshould give an error\n');
p = unitbox(2);
lasterr('');
try
    q = p([1 2]);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, '??? Index exceeds array dimension');


%-------------------------------------------------------------
%fprintf('\n\nshould work\n');
p1 = unitbox(2);
p2 = unitbox(2, 2);
p = [p1 p2];
ind = [1 2 2 1];
q = p(ind);
mbg_assertequal(length(q), 4);
for i = 1:length(q),
    mbg_asserttrue(q(i) == p(ind(i)));
end


%-------------------------------------------------------------
%fprintf('\n\nshould give an error\n');
p1 = unitbox(2);
p2 = unitbox(2, 2);
p = [p1 p2];
ind = [1 2 2 3];
lasterr('');
try
    q = p(ind)
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, '??? Index exceeds matrix dimension');


%-------------------------------------------------------------
%fprintf('\n\nindexing by a zero should give an error\n');
p1 = unitbox(2);
p2 = unitbox(2, 2);
p = [p1 p2];
ind = 0;
lasterr('');
try
    q = p(ind)
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Subscript indices must either be real positive integers or logicals.');


%-------------------------------------------------------------
%fprintf('\n\nindexing by a vector containing a zero should give an error\n');
p1 = unitbox(2);
p2 = unitbox(2, 2);
p = [p1 p2];
ind = [1 0 2];
lasterr('');
try
    q = p(ind)
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'Subscript indices must either be real positive integers or logicals.');
