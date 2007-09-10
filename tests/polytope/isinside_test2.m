function isinside_test2

p1 = unitbox(2);
p2 = unitbox(2)+[2;2];
p3 = unitbox(2)+[-10; 0];
p = [p1 p2 p3];

%-------------------------------------------------------------
a = isinside(p1, [0.5; 0]);
mbg_asserttrue(a);


%-------------------------------------------------------------
[a, b] = isinside(p1, [0.5; 0]);
mbg_asserttrue(a);
mbg_assertequal(b, 1);


%-------------------------------------------------------------
[a, b, c] = isinside(p1, [0.5; 0]);
mbg_asserttrue(a);
mbg_assertequal(b, 1);
mbg_asserttrue(isempty(c));


%-------------------------------------------------------------
[a, b, c] = isinside([p1 p2 p3], [0.5; 0]);
mbg_asserttrue(a);
mbg_assertequal(b, 1);
mbg_asserttrue(isempty(c));


%-------------------------------------------------------------
[a, b, c] = isinside([p1 p2 p3], [1; 1]);
mbg_asserttrue(a);
mbg_assertequal(b, [1; 2]);
mbg_asserttrue(isempty(c));
% inwhich should be a column vector
mbg_assertequal(size(b), [2 1]);


%-------------------------------------------------------------
[a, b, c] = isinside([p1 p2 p3], [10; 1]);
mbg_assertfalse(a);
mbg_asserttrue(isempty(b));
mbg_assertequal(c, 2);


%-------------------------------------------------------------
[a, b, c] = isinside([p1 p2 p3], -[10; 2]);
mbg_assertfalse(a);
mbg_asserttrue(isempty(b));
mbg_assertequal(c, 3);


%-------------------------------------------------------------
% wrong dimension of x0
lasterr('');
try
    a = isinside(p1, 0);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);


%-------------------------------------------------------------
% wrong dimension of x0
lasterr('');
try
    a = isinside(p1, []);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);


%-------------------------------------------------------------
% wrong dimension of x0
lasterr('');
try
    a = isinside(p1, [1; 1; 1]);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);


%-------------------------------------------------------------
% wrong dimension of x0
lasterr('');
try
    a = isinside([p1 p2 p3], 0);
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);


%-------------------------------------------------------------
% isinside(emptypoly, x0) should return 0, not an error
a = isinside(polytope, [0; 0]);
mbg_assertfalse(a);


%-------------------------------------------------------------
% isinside(emptypoly, x0) should return 0, not an error
[a, b, c] = isinside(polytope, 0);
mbg_assertfalse(a);
mbg_asserttrue(isempty(b));
mbg_assertequal(c, 1);


