function double_test1

p = unitbox(2, 1);
q = unitbox(2, 2);

P = [p q];

% test single polytope
[h, k]=double(p);
mbg_assertequal(h, [eye(2); -eye(2)]);
mbg_assertequal(k, repmat(1, 4, 1));

hk = double(p);
mbg_assertequal(hk, [[eye(2); -eye(2)] repmat(1, 4, 1)]);


% test polyarrays
[h, k] = double(P);

mbg_asserttrue(iscell(h) & iscell(k));
mbg_assertequal(length(h), 2);
mbg_assertequal(k{1}, repmat(1, 4, 1));
mbg_assertequal(k{2}, repmat(2, 4, 1));
