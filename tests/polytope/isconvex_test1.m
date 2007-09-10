function isconvex_test1

% changeset 381:  	11a22ee285ec
% parent 380:	14435437284e
% manifest: 	54e56b6709b8
% author: 	Michal Kvasnica <kvasnica@control.ee.ethz.ch>
% date: 	Sat Aug 27 18:59:02 2005 (2 weeks ago)
% files: 	@polytope/isconvex.m
% description: 	[PATCH] handle cases where bounding_box() returns NaN answer
% 
% Signed-off-by: Michal Kvasnica
% 
% if E=envelope(P) returns R^n, bounding_box(E) returns NaNs. we should
% escape quickly, because this means that P is non-convex.


% isconvex() run on R^n should not give an error

p1 = polytope([1 1; 1 -1],[1;1]);
p2 = -p1;

% bounding box of the following envelope (R^n) contains NaNs
E = envelope([p1 p2]);

% this must not give an error
isconvex([p1 p2]);
