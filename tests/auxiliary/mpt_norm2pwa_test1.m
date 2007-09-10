function mpt_norm2pwa_test1

% tests mpt_norm2pwa():
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=9ed7ec70fe49

P = mpt_norm2pwa(eye(2), 1);
mbg_assertequal(length(P.Pn), 4);
mbg_asserttolequal(sort([P.Bi{:}]), sort([1 1 -1 1 1 -1 -1 -1]));
mbg_asserttolequal([P.Ci{:}], [0 0 0 0]);

Q = eye(3);
P = mpt_norm2pwa(Q, Inf);
mbg_assertequal(length(P.Pn), 6);
mbg_asserttolequal(sort([P.Bi{:}]), sort([0 0 1 0 1 0 1 0 0 0 0 -1 0 -1 0 -1 0 0]));
mbg_asserttolequal([P.Ci{:}], [0 0 0 0 0 0]);


P = mpt_norm2pwa([eye(2); -4 6], 1, struct('Pn', unitbox(2, 10)));
mbg_asserttrue(P.Pn == unitbox(2, 10));
mbg_asserttolequal(sort([P.Bi{:}]), sort([-3 7 -5 7 5 -5 -5 5 5 -7 3 -7]));
mbg_asserttolequal([P.Ci{:}], [0 0 0 0 0 0]);
