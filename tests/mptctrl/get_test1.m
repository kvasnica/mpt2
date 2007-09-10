function get_test1

% check if mptctrl/get() works fine
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=9ee694f0d63ad19a0d63c553aad2f5feb1f6b0a5

load ctrl1
o = get(ctrl, 'overlaps');
mbg_assertequal(o, 0);

o = get(ctrl, 'Pn');
mbg_assertequal(length(o), 9);
