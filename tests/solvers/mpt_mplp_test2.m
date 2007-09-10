function mpt_mplp_test2

% we have been getting "Division by zero" warnings on some specific examples,
% fixed by
% http://control.ee.ethz.ch/~mpt/hg/mpt?cs=229a486b9db4

load mplp_divbyzero

% switch on warnings just in case we switched them off
warning on

% the "Division by zero" problem only happens with CDD
mpt_options('lpsolver', 'cdd');
T = evalc('Pn=mpt_mplp(Matrices, Options);');

% no output should be print out to the screen (i.e. no warnings)
mbg_asserttrue(isempty(T));

% solution should consist of 2 regions
mbg_assertequal(length(Pn), 2);




% here is the same problem problem, but error with
% this setup happens only with GLPK as LP solver

load mplp_glpkprob
% note that Options.lpsolver is 4

% as of 20.10.2005 this one gives an error:
Pn = mpt_mplp(Matrices, Options);
mbg_assertequal(length(Pn), 1);
