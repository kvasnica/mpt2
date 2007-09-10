function mpt_removeOverlaps_test1

% tests mpt_removeOverlaps when input is a polytope array

p = unitbox(2,1);
% create an overlapping polyarray
P=[p+[1;1], p+[1;-1], p+[-1;1], p+[-1;-1], p];

Q = mpt_removeOverlaps(P);
mbg_assertequal(length(Q), 9);

% check that there are no overlaps
[a,ia,ib,iab]=dointersect(Q, Q);
mbg_assertequal(iab, [(1:9)' (1:9)'])



% now test case where we have several regions which are identical
P=[p+[1;0], p+[1;0], p+[-1;0], p+[-1;0]];
Q = mpt_removeOverlaps(P);
mbg_assertequal(length(Q), 2);
% check that there are no overlaps
[a,ia,ib,iab]=dointersect(Q, Q);
mbg_assertequal(iab, [(1:2)' (1:2)'])
