function sedumi_test1

% tests correct behavior of sedumi with respect to verbosity changes as per
% Bug 43: http://control.ee.ethz.ch/~mpt/bugzilla/show_bug.cgi?id=43

yalmip('clear');
sdpvar x
F = set(-1 < x < 1);

%fprintf('No verbosity\n');
opt = sdpsettings('solver', 'sedumi', 'verbose', 0);
T = evalc('solvesdp(F, x''*x, opt);');
mbg_asserttrue(isempty(T));
mbg_assertequal(double(x), 0);

%fprintf('Default verbosity\n');
opt = sdpsettings('solver', 'sedumi', 'verbose', 1);
T = evalc('solvesdp(F, x''*x, opt);');
mbg_assertfalse(isempty(T));
mbg_asserttrue(strfind(T, 'Max-norms'));
mbg_assertequal(double(x), 0);
