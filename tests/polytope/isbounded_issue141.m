function issue141_isbounded
% PROFILEME

% isbounded() with the old equality-constraints formulation was giving wrong
% results sometimes.
%
% fixed in
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=5d8fec61a19a02346a83968134a871cf73147f06

load issue141
% all three polytopes must be bounded
b1 = isbounded(A);
b2 = isbounded(B);
b3 = isbounded(A & B);

mbg_assertequal(b1, 1);
mbg_assertequal(b2, 1);
mbg_assertequal(b3, 1);
