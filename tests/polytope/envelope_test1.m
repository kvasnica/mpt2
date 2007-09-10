function envelope_test1

% prior to 2.0.3 envelope() returned R^n as a polytope in 1D. since 2.0.3 we
% return R^n in the same dimension as the input polytopes


% create two polytopes whose envelope is R^n
p1 = polytope([1 1; 1 -1],[1;1]);
p2 = -p1;

E = envelope([p1 p2]);
if dimension(E) ~= dimension(p1),
    error('R^n has wrong dimension!');
end
