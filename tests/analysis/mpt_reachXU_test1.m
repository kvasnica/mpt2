function mpt_reachXU_test1

% tests mpt_reachXU()

X0 = unitbox(2,1)+[3;0];
U0 = unitbox(1,1);
Afull = [1 1; 0 1];
Alow = [1 1; 0 0];
B = [1; 0.5];
f = [0.5; 0];

% full-dimensional map, no affine term
R1=mpt_reachXU(X0, U0, Afull, B);

% full-dimensional map, affine term given
R2=mpt_reachXU(X0, U0, Afull, B, f);
mbg_asserttrue((R1+f) == R2);

% lower-dimensional map, no affine term
R3=mpt_reachXU(X0, U0, Alow, B);

% lower-dimensional map, affine term given
R4=mpt_reachXU(X0, U0, Alow, B, f);
mbg_asserttrue((R3+f)==R4);
