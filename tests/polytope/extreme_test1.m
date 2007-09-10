function extreme_test1

% a bug in MPT 2.0.3 made it impossible to enumerate extreme points for cyclic
% polytopes:
% http://control.ee.ethz.ch/~mpt/hg/mpt-20x?cmd=changeset;node=7139809be5b0445d7df29fa03118268ea948c51d

% a cyclic polytope has less extreme points than the number of facets.

% this creates a random polytope in "n"-dimensions with "m" vertices:
% n=3;           % dimension
% m=10;          % # of vertices
% v=rand(1, m);  
% V = (repmat(v,n,1).^(repmat(1:n,m,1)'))';
% p=polytope(V);


%==================================================================
% TEST 1
%==================================================================

% a cyclic polytope in 8D with 10 vertices
V=[  4    -3    -4    -3     5     4     5     1     5    -8; ...
     -7    -1     0    -2    -2    -4     6    -7    -5    -3; ...
      2    -5    -7    -7     2     0     8     1     5    -7; ...
     -7    -2    -6     3     2    -2     0    -2    -1    -3; ...
     -3    -9    -3    -9    -8    -1     7    -1     7     0; ...
      8    -6    -4     9    -8    -5    -8    -8     6    -5; ...;
      8     0    -8    -8     2     3     8    -1     3     5; ...
     -6     7     8     5     9    -5    -1    -7    -9     7];
V = V';
p = polytope(V);

% here extreme() should just return those 10 points which have been used for
% construction and have been stored inside of "p"
e=extreme(p);

% points must be exactly equal, because they have been stored when constructing
% the polytope and thus an exact copy should be returned by extreme()
mbg_assertequal(e, V);

% check extreme point enumeration on H-representation
[h,k] = double(p);
% the H-representation must consist of 16 hyperplanes
q = polytope(h,k);
mbg_assertequal(nconstr(q), 16);

%==================================================================
% TEST 2
%==================================================================
% a cyclic polytope in 3D with 10 vertices
V = [    0.1000    0.0100    0.0010
    0.2000    0.0400    0.0080
    0.3000    0.0900    0.0270
    0.4000    0.1600    0.0640
    0.5000    0.2500    0.1250
    0.6000    0.3600    0.2160
    0.7000    0.4900    0.3430
    0.8000    0.6400    0.5120
    0.9000    0.8100    0.7290
    1.0000    1.0000    1.0000];
p = polytope(V);

% here extreme() should just return those 10 points which have been used for
% construction and have been stored inside of "p"
e=extreme(p);

% points must be exactly equal, because they have been stored when constructing
% the polytope and thus an exact copy should be returned by extreme()
mbg_assertequal(e, V);

% check extreme point enumeration on H-representation
[h,k] = double(p);

% the H-representation must consist of 16 hyperplanes
q = polytope(h,k);
mbg_assertequal(nconstr(q), 16);

% extreme points must be equal
w = extreme(q);

% we must get 10 extreme points
mbg_assertequal(size(w, 1), 10);

% check if extreme points are correct
mbg_asserttolequal(sortrows(e), sortrows(w), 1e-10);
