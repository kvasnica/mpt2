function mpt_infsetPWA_test1

% tests that we properly recognize when invariant set is empty
load invset_empty

% the invariant set for this example is empty:
%
% Mario Vasak, 27-Sep-2005:
%
% Consider the autonomous 
% system given with Pn, A and f   I send you in counterexample_old.mat.
% It is a 2D system, see:
% 
% plot(Pn)
% 
% In Pn(1) dynamics is such that you totally enter Pn(2) in one step, and 
% in Pn(2) it is such that you leave the whole Pn in one step. Thus, the 
% invariant set is an empty set.

I = mpt_infsetPWA(Pn,A,f);
mbg_assertfalse(isfulldim(I));
