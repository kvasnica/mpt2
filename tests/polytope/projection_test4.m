function projection_test4

% tests that we correctly catch wrong dimensions
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=092f7155af73

P = unitbox(2,1);

lasterr('');
%disp('Projection on [1 2 3] should give an error.');
dim = [1 2 3];
try
    Q = projection(P, dim);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'PROJECTION: Dimensions must agree.');


lasterr('');
%disp('Projecting on [1 0] should give an error.');
dim = [1 0];
try
    Q = projection(P, dim);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'PROJECTION: Projection dimensions exceed polytope dimension.');


lasterr('');
%disp('Projecting on [] should give an error.');
dim = [];
try
    Q = projection(P, dim);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'PROJECTION: Projection dimensions cannot be empty.');


lasterr('');
%disp('Projecting on [1 3] should give an error.');
dim = [1 3];
try
    Q = projection(P, dim);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'PROJECTION: Projection dimensions exceed polytope dimension.');
