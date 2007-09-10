function bounding_box_test1

% bounding box of en empty polytope is handled specially, check that:
P = polytope;
B = bounding_box(P);
mbg_asserttrue(~isfulldim(B));

[B,l,u,lv,uv] = bounding_box(P);
mbg_asserttrue(~isfulldim(B));

[B,l,u,lv,uv] = bounding_box(P, struct('noPolyOutput', 1));


p1=polytope([0 0; 6 0; 8 -2; 0 -2]);
p2=polytope([0 0; 4 0; 6 2; 0 2]);
p3=polytope([4 0; 6 0; 6 2; 8 2]);
p4=polytope([6 0; 8 2; 8 -2]);
P = [p1 p2 p3 p4];

b = bounding_box(p1);
mbg_assertequal(b, polytope([0 -2; 0 0; 8 0; 8 -2]));

b = bounding_box(P);
mbg_assertequal(b, polytope([0 -2; 0 2; 8 2; 8 -2]));

[b, l, u] = bounding_box(p1);
mbg_assertequal(l, [0; -2]);
mbg_assertequal(u, [8; 0]);

[b, l, u] = bounding_box(P);
mbg_assertequal(l, [0; -2]);
mbg_assertequal(u, [8; 2]);
