function mpt_solveLPs_test1

load mpt_solveLPs_test1
[xopt,fval,lambda,exitflag,how]=mpt_solveLPs(f, A, B, [], [], [], 3);
