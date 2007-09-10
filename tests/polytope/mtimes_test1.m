function mtimes_test1

% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=c1fca82b4f63

P = unitbox(2,1);
% test scaling with a scalar
Q1 = 0.5*P;
Q2 = P*0.5;
good = unitbox(2,0.5);
mbg_asserttrue(Q1 == Q2);
mbg_asserttrue(Q1 == good);

% test multiplication by a matrix -> leads to range()
Q1 = diag([2 1])*P;
Q2 = P*diag([2 1]);
good = polytope([eye(2); -eye(2)], [2;1;2;1]);
mbg_asserttrue(Q1 == Q2);
mbg_asserttrue(Q1 == good);

% test multiplication of two polytopes -> leads to a polytope in higher
% dimension
P2 = unitbox(1, 0.1);
Q = P*P2;
good = polytope([eye(3); -eye(3)], [1; 1; 0.1; 1; 1; 0.1]);
mbg_asserttrue(Q == good);
