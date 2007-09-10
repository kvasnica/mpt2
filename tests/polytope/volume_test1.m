function volume_test1

% tests that we do not return volume as a complex number:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=762d7bcdffed

load complex_volume
V = volume(P);
mbg_asserttrue(isreal(V));
mbg_asserttolequal(V, 0.7418098, 1e-6);
