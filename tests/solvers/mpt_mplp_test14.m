function Pn=mpt_mplp_test14

% this one used to cycle with NAG lp due to cycling in the iterative hull
% projection method. fixed by:
% http://control.ee.ethz.ch/~mpt/hg/mpt-25x?cs=af2e36fa586f

% Data
A = [2 -1;1 0];nx = 2;
B = [1;0];nu = 1;
C = [0.5 0.5];

% Prediction horizon
N = 6;

% Alternative 
[Hx,Sx] = create_HS(A,B,N);
[H,S]   = create_CHS(A,B,C,N);

x = sdpvar(2,1);
U = sdpvar(N,1);

% Parameterized predictions
Y = H*x+S*U; 
X = Hx*x+Sx*U;

s = sdpvar(N,1);
%Control constraints
F = set(-1 < U < 1);
F = F+set(-1-s < Y < 1+s); 

% Exploration space
F = F + set(-5 < x < 5);

F = F + set(s>=0);

obj = norm([X;U],1) + sum(s);
Pn=solvemp(F,obj,[],x,U(1));
