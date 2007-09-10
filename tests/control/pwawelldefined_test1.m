function pwawelldefined_test1

% PWAwelldefined() failed with an error if the H-representation was not a
% cell array

pwa = {};
h = [eye(2); -eye(2)];
k = ones(4, 1);
pwa{1}.Hi{1} = h;
pwa{1}.Ki{1} = k;
out = PWAwelldefined(pwa, h, k, 3);
mbg_asserttrue(out);
