function mpt_optQuadCtrl_test7

% tests whether we properly deal with probStruct.y0bounds=0/1
%

Double_Integrator
probStruct.N = 2;
probStruct.norm = 2;
sysStruct.Uset = [-1 0 1];

probStruct.y0bounds=1;
ctrl1 = mpt_control(sysStruct, probStruct);
% here the solution must be defined in the box of +/- 5
good = unitbox(2, 5);
mbg_asserttrue(bounding_box(ctrl1.Pfinal) == good);

probStruct.y0bounds=0;
ctrl0 = mpt_control(sysStruct, probStruct);
% here the solution must be defined in the following box
good = polytope([eye(2); -eye(2)], [10.5; 5.5; 10.5; 5.5]);
mbg_asserttrue(bounding_box(ctrl0.Pfinal) == good);
