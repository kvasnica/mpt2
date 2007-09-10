function mpt_infset_test1

% mpt_infset() should give an error if the partition is passed as a
% polytope array (mpt_infsetPWA should be used in that case)


% basic test for correctness of mpt_infset()
A = eye(2);
X = unitbox(2);
[Oinf,tstar,fd,isemptypoly] = mpt_infset(A, X, 10);
mbg_asserttrue(Oinf == X);

% this cases should be automatically handled by mpt_infsetPWA()
A = {eye(2), eye(2)};
X = unitbox(2);
Oinf = mpt_infset(A, X, 10);
mbg_asserttrue(Oinf == X);

% this cases should be automatically handled by mpt_infsetPWA()
A = eye(2);
X = [unitbox(2)+[-1; 0] unitbox(2)+[1; 0]];
Oinf = mpt_infset(A, X, 10);
mbg_asserttrue(Oinf == X);
