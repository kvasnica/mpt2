function mpt_verify_test1

% tests mpt_verify() when input is a system structure

A=[-1 -4; 4 -1]; 
B=[1;1]; 
C=[1 0]; 
D=0; 
di=ss(A,B,C,D); 
did=c2d(di,0.02); 
sst=mpt_sys(did); 
sst.ymax=10; sst.ymin=-10; 
sst.umax=1; sst.umin=-1; 
X0=polytope([0.9 0.1; 0.9 -0.1; 1.1 0.1; 1.1 -0.1]); 
U0=unitbox(1,0.1);

% feasible verification
Xf=unitbox(2,0.05)+[0.1; -0.4]; 
f=mpt_verify(sst,X0,Xf,100,U0); 
mbg_asserttrue(f);

% infeasible verification
Xf=unitbox(2,1)+[100; 0]; 
f=mpt_verify(sst,X0,Xf,20,U0); 
mbg_assertfalse(f);
