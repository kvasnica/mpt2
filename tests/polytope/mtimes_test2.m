function mtimes_test2

% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=c1fca82b4f63

P = unitbox(2,1);
% test special cases of scalar scaling

% 1. scaling with one = no scaling
Q = 1*P;
good = unitbox(2,1);
mbg_asserttrue(Q == good);

% 2. scaling with zero = empty polytope
Q = P*0;
good = polytope;
mbg_asserttrue(Q == good);

% 3. scaling a polytope which does not containt the origin (was not available
% prior to Nov-10-2005
P = polytope([eye(2); -eye(2)], [3;3;-1;-1]);
good = polytope([eye(2); -eye(2)], [1.5; 1.5; -0.5; -0.5]);
Q = 0.5*P;
mbg_asserttrue(Q == good);

% 4. scaling of polytope arrays
PP = [P -P];
Q = 0.5*PP;
g1 = polytope([eye(2); -eye(2)], [1.5; 1.5; -0.5; -0.5]);
g2 = polytope([-eye(2); eye(2)], [1.5; 1.5; -0.5; -0.5]);
mbg_assertequal(length(Q), 2);
mbg_asserttrue(Q == [g1 g2]);
