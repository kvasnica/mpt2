function triangulate_test2

% triangulation of certain polytopes gives a qhull warnings:

load triang1

t = triangulate(p);
mbg_assertequal(length(t), 2);
mbg_asserttrue(t==p);

v = volume(p);
mbg_asserttolequal(v, 0.51111, 1e-5);
