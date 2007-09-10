function reduce_test3

% check that we preserve order of hyperplanes:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=aac953fb070c

A=[eye(2); -eye(2); eye(2)]; 
B=ones(6,1); 
P = polytope(A,B);

% P should preserve the order. without the patch:
% [-1  0]      [1]
% [ 0 -1]      [1]
% [ 0  1] x <= [1]
% [ 1  0]      [1]
%
% with the patch:
% [-1  0]      [1]
% [ 0 -1]      [1]
% [ 1  0] x <= [1]
% [ 0  1]      [1]

hk = double(P);
mbg_assertequal(hk, [-1 0 1; 0 -1 1; 1 0 1; 0 1 1]);
