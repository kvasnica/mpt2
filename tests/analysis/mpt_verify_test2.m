function mpt_verify_test2

% tests mpt_verify() when input is a controller

load di_ctrl_N3_norm2

% feasible verification
X0=unitbox(2,1)+[3;0];
Xf=unitbox(2,0.1); 

f=mpt_verify(ctrl, X0, Xf, 100); 
mbg_asserttrue(f);


% infeasible verification
X0=unitbox(2,1)+[3;0];
Xf=unitbox(2,0.1)+[1; 0]; 

f=mpt_verify(ctrl, X0, Xf, 15); 
mbg_assertfalse(f);
