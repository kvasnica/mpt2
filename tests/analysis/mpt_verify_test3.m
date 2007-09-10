function mpt_verify_test3

% - tests mpt_verify() when input is a sysStruct and Options.usereachsets=0
% - LTI systems
% - related to Bug 37: 
%     http://control.ee.ethz.ch/~mpt/bugzilla/show_bug.cgi?id=37

Options.usereachsets = 0;
Double_Integrator;


% ===== Target is a single polytope =====
X0 = unitbox(2, 0.5)+[3; 0];
U0 = unitbox(1, 1);
N = 10;
Xf_feasible = unitbox(2, 0.5);
Xf_infeasible = unitbox(2, 0.5)+[-50; 0];

% LTI sysStruct, U0 given, feasible verification
Xf = Xf_feasible;
[yesno, Nf, opt] = mpt_verify(sysStruct, X0, Xf, N, U0, Options);
mbg_asserttrue(yesno);
mbg_assertequal(length(opt), N);

% LTI sysStruct, U0 given, infeasible verification
Xf = Xf_infeasible;
[yesno, Nf, opt] = mpt_verify(sysStruct, X0, Xf, N, U0, Options);
mbg_assertfalse(yesno);
mbg_asserttrue(isempty(Nf));  % Nf is returned as [] if the problem is infeasible
mbg_asserttrue(isempty(opt));

% LTI sysStruct, no U0 given, feasible verification
Xf = Xf_feasible;
[yesno, Nf, opt] = mpt_verify(sysStruct, X0, Xf, N, Options);
mbg_asserttrue(yesno);
mbg_assertequal(length(opt), N);

% LTI sysStruct, no U0 given, infeasible verification
Xf = Xf_infeasible;
[yesno, Nf, opt] = mpt_verify(sysStruct, X0, Xf, N, Options);
mbg_assertfalse(yesno);
mbg_asserttrue(isempty(Nf));  % Nf is returned as [] if the problem is infeasible
mbg_asserttrue(isempty(opt));




% ===== Target is a polytope array =====
X0 = unitbox(2, 0.5)+[3; 0];
U0 = unitbox(1, 1);
N = 10;
Xf_feasible = [unitbox(2, 1)+[40; 0] unitbox(2, 0.5)];
Xf_infeasible = [unitbox(2, 0.5)+[-50; 0]; unitbox(2, 1)+[30; 0]];

% LTI sysStruct, U0 given, feasible verification
Xf = Xf_feasible;
[yesno, Nf, opt] = mpt_verify(sysStruct, X0, Xf, N, U0, Options);
mbg_asserttrue(yesno);
mbg_assertequal(length(opt), N);

% LTI sysStruct, U0 given, infeasible verification
Xf = Xf_infeasible;
[yesno, Nf, opt] = mpt_verify(sysStruct, X0, Xf, N, U0, Options);
mbg_assertfalse(yesno);
mbg_asserttrue(isempty(Nf));  % Nf is returned as [] if the problem is infeasible
mbg_asserttrue(isempty(opt));

% LTI sysStruct, no U0 given, feasible verification
Xf = Xf_feasible;
[yesno, Nf, opt] = mpt_verify(sysStruct, X0, Xf, N, Options);
mbg_asserttrue(yesno);
mbg_assertequal(length(opt), N);

% LTI sysStruct, no U0 given, infeasible verification
Xf = Xf_infeasible;
[yesno, Nf, opt] = mpt_verify(sysStruct, X0, Xf, N, Options);
mbg_assertfalse(yesno);
mbg_asserttrue(isempty(Nf));  % Nf is returned as [] if the problem is infeasible
mbg_asserttrue(isempty(opt));
