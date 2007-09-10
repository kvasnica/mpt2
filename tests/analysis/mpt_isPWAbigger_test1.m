function mpt_isPWAbigger_test1

% test that mpt_isPWAbigger() works correctly
load lti1d_ctrl_N1_norm1

Pn = ctrl.Pn;
Bi = ctrl.Bi;
Ci = ctrl.Ci;
J1.Pn = Pn;
J1.Bi = Bi;
J1.Ci = Ci;
J2 = J1;

% J2 and J1 are equal
a = mpt_isPWAbigger(J1, J2);
mbg_assertequal(a, 0);

% J2 will be bigger than J1
for ii=1:length(J2.Ci),
    J2.Ci{ii} = J2.Ci{ii}+1;
end
% J1 <= J2
a = mpt_isPWAbigger(J1, J2);
mbg_assertequal(a, 2);
% J1 >= J2
a = mpt_isPWAbigger(J2, J1);
mbg_assertequal(a, 1);

% J2 sometime bigger than J1, J1 sometimes bigger than J2
J2 = J1;
for ii=1:round(length(J2.Ci)/2),
    J2.Ci{ii} = J2.Ci{ii}+1;
end
for ii=round(length(J2.Ci)/2):length(J2.Ci),
    J2.Ci{ii} = J2.Ci{ii}-1;
end
a = mpt_isPWAbigger(J2, J1);
mbg_assertequal(a, 3);
