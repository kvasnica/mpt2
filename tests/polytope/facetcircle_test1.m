function facetcircle_test1


% reported by Mato Baotic on 17.9.2005:
%   center and radius of a facet are wrong
%
% fixed in
% http://control.ee.ethz.ch/~mpt/hg/mpt-20x?cmd=changeset;node=b74a86be25caad52f814c33d159f3172fdee041d

P=polytope([1 0; -1e-4 1+1e-4; 0 -1], [10; 0; 0]);
[x,r] = facetcircle(P, 2);

mbg_asserttolequal(x, [5; 0.0005], 1e-7);
mbg_asserttolequal(r, 5, 1e-7);
