function mpt_norm2pwa_test2

% tests mpt_norm2pwa() with the enumeration approach:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=fa83ab2a7dac

Options.method = 2;  % use the enumeration method

P = mpt_norm2pwa(eye(2), 1, Options);
mbg_assertequal(length(P.Pn), 4);
mbg_asserttolequal(sort([P.Bi{:}]), sort([-1 -1 -1 1 1 -1 1 1]));
mbg_asserttolequal([P.Ci{:}], [0 0 0 0]);

Q = eye(3);
P = mpt_norm2pwa(Q, Inf, Options);
mbg_assertequal(length(P.Pn), 6);
mbg_asserttolequal(sort([P.Bi{:}]), sort([1 0 0 0 1 0 0 0 1 -1 0 0 0 -1 0 0 0 -1]));
mbg_asserttolequal([P.Ci{:}], [0 0 0 0 0 0]);

Options.Pn = unitbox(2, 10); % limit the exploration space
P = mpt_norm2pwa([eye(2); -4 6], 1, Options);
mbg_asserttrue(P.Pn == unitbox(2, 10));
mbg_asserttolequal(sort([P.Bi{:}]), sort([3 -7 -5 5 -5 7 5 -7 5 -5 -3 7]));
mbg_asserttolequal([P.Ci{:}], [0 0 0 0 0 0]);
