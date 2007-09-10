function bounding_box_test4

% tests Options.Voutput:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=3986c280ac8b

P = unitbox(2,1);

% B will contain lower and upper vertices if Voutput=1
B = bounding_box(P, struct('Voutput', 1));
mbg_assertequal(B, [-1 1; -1 1]);

% B will contain the vertices of the box if Voutput=1 and bboxvertices=1
[B,l,u,lv,uv] = bounding_box(P, struct('Voutput', 1, 'bboxvertices', 1));
mbg_assertequal(B, [-1 -1 1 1; -1 1 -1 1]);


% now test the same for polyarrays
P = [unitbox(2,1) unitbox(2,1)+[1;2]];

% B will contain lower and upper vertices if Voutput=1
B = bounding_box(P, struct('Voutput', 1));
mbg_assertequal(B, [-1 2; -1 3]);

% B will contain the vertices of the box if Voutput=1 and bboxvertices=1
[B,l,u,lv,uv] = bounding_box(P, struct('Voutput', 1, 'bboxvertices', 1));
mbg_assertequal(B, [-1 -1 2 2; -1 3 -1 3]);
