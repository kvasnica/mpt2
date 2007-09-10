function set_test1

% check if mptctrl/set() works fine
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=9ee694f0d63ad19a0d63c553aad2f5feb1f6b0a5

load ctrl1
a = set(ctrl, 'overlaps', 1);
mbg_assertequal(a.overlaps, 1);

a = set(ctrl, 'Pn', [], 'Ai', eye(2));
mbg_assertequal(isempty(a.Pn), 1);
mbg_assertequal(a.Ai, eye(2));
