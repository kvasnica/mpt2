function dointersect_test1


% 2 polytopes sharding a facet, they do not intersect (in the sense that the
% intersection is not fully dimensional)
p1 = polytope([0 0; 1 0; 1 1; 0 1]);
p2 = polytope([0 0; -1 0; -1 1; 0 1]);
i = dointersect(p1, p2);
mbg_assertequal(i, 0);

% p1 and p3 do intersect
p3 = unitbox(2, 0.5);
i = dointersect(p1, p3);
mbg_assertequal(i, 1);

% [p1 p2] and p3 do intersect
i1 = dointersect([p1 p2], p3);
i2 = dointersect(p3, [p1 p2]);
mbg_assertequal(i1, 1);
mbg_assertequal(i2, 1);

% [p1 p2 p3] and p4 do not intersect
p4 = polytope([-0.5 1; 0.5 1; 0.5 2; -0.5 2]);
i = dointersect([p1 p2 p3], p4);
mbg_assertequal(i, 0);

% [p1 p2 p3] and [p4 p5] do intersect
p5 = unitbox(2, 0.2);
i = dointersect([p1 p2 p3], [p4 p5]);
mbg_assertequal(i, 1);
