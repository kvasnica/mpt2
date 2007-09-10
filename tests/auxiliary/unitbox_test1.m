function unitbox_test1


u = unitbox(2,2.5);
e = (u==polytope([eye(2); -eye(2)],repmat(2.5, 4, 1)));
mbg_assertequal(e, 1);

% since 28.9.2005 it is allowed to call unitbox with just one input argument:
% http://control.ee.ethz.ch/~mpt/hg/mpt?cmd=changeset;node=028f049b527b7ee222e662a37275744a564d6974

u = unitbox(2);
e = (u==polytope([eye(2); -eye(2)],repmat(1, 4, 1)));
mbg_assertequal(e, 1);
