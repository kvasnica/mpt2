function projection_test8

% tests projection of lower-dimensional polytopes

% this is a polytope x1+x2 = 0
H = [1 1; -1 -1];
K = [0; 0];

% bound every element
bH = [eye(2); -eye(2)];
bK = [1; 1; 1; 1];
H = [H; bH];
K = [K; bK];

% first create a dummy polytope in compatible dimension
P = unitbox(2, 1);

% now overwrite it's H-representation with our data
P = set(P, 'H', H);
P = set(P, 'K', K);

% just to be sure, set chebychev center to zero
P = set(P, 'xCheb', zeros(size(H, 2), 1));

% now project the polytope down
Q = projection(P, 1);

% projection must be equal to a unit box in 1D
mbg_asserttrue(Q==unitbox(1,1));
