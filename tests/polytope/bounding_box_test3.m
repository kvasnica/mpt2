function bounding_box_test3

% tests whether we return correct polytope with Options.noPolyOutput=1:

P = unitbox(2,1);
% delete any boudning box information stored in P
P = set(P, 'bbox', []);

% use Options.noPolyOutput=1, that will result in B containing P, but with
% updated bounding box information, i.e. B=P, B.bbox = [l u]
B = bounding_box(P, struct('noPolyOutput', 1));
mbg_asserttrue(isa(B, 'polytope'));
bbox = get(B, 'bbox');
mbg_assertequal(bbox, [-1 1; -1 1]);


% now test the same with polytope arrays
P1 = unitbox(2,1)+[-1;0];
P2 = unitbox(2,1)+[1;0];
% delete any boudning box information stored in P
P1 = set(P1, 'bbox', []);
P2 = set(P2, 'bbox', []);
Q = [P1 P2];

% use Options.noPolyOutput=1, that will result in B containing P, but with
% updated bounding box information, i.e. B=P, B.bbox = [l u]
B = bounding_box(Q, struct('noPolyOutput', 1));
mbg_asserttrue(isa(B, 'polytope'));

% output must be a polyarray (because it is just a copy of the input)
mbg_assertequal(length(B), 2);

bbox1 = get(B(1), 'bbox');
bbox2 = get(B(2), 'bbox');

% check whether the bounding box information has been updated correctly
mbg_assertequal(bbox1, [-2 0; -1 1]);
mbg_assertequal(bbox2, [0 2; -1 1]);
