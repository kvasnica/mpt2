function dimension_test1

% dimension of an empty polytope is 1
p = polytope;
d = dimension(p);
mbg_assertequal(d, 1);

p = unitbox(4,1);
d = dimension(p);
mbg_assertequal(d, 4);
