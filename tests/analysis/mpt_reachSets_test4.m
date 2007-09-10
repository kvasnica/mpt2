function mpt_reachSets_test4

% tests whether we properly catch cases where U0 is either a not fully
% dimensional polytope, or if it is a polytope array

pwa_car
U0=polytope([0; 1]);
X0=polytope([-3.5 0; -3.49, 0; -3.49, 0.01; -3.5 0.01]);
n = 3;
opt.verbose = 2;

%fprintf('Expecting failure when U0 is a polytope array\n');
lasterr('');
try
    UU = [U0 U0];
    Psets = mpt_reachSets(sysStruct, X0, UU, n, opt);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'This function does not support polytope arrays in U0.');

%fprintf('\nExpecting failure when U0 is not fully dimensional\n');
lasterr('');
try
    UU = polytope;
    Psets = mpt_reachSets(sysStruct, X0, UU, n, opt);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
T = lasterr;
mbg_assertcontains(lasterr, 'U0 must be a fully dimensional polytope.');
