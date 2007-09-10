function bounding_box_test2

% tests whether we compute vertices of bounding boxes correctly:

P = unitbox(2,1);
[B,l,u,lv,uv] = bounding_box(P);  % bounding box will be returned as a polytope
mbg_asserttrue(B==P);      % bounding box of a unitbox is the unit box itself
mbg_assertequal(l, [-1; -1]);
mbg_assertequal(u, [1; 1]);

% Options.noPolyOutput=1, Options.bboxvertices=0 will return P with bounding box
% stored in P.bbox
[B,l,u,lv,uv] = bounding_box(P, struct('noPolyOutput', 1));
mbg_asserttrue(isa(B, 'polytope'));
mbg_assertequal(l, [-1; -1]);
mbg_assertequal(u, [1; 1]);



% now test the same with polytope arrays
P = [unitbox(2,1) unitbox(2,1)+[1;2]];

% Options.noPolyOutput=1, Options.bboxvertices=0 will return P with bounding box
% stored in P.bbox
[B,l,u,lv,uv] = bounding_box(P, struct('noPolyOutput', 1));
mbg_asserttrue(isa(B, 'polytope'));
mbg_asserttrue(B==P);
mbg_assertequal(l, [-1; -1]);
mbg_assertequal(u, [2; 3]);
