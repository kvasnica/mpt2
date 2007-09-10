function mpt_mpqp_test9


% when looking for an initial feasible point, constraints in the QP should
% be first reduced, otherwise quadprog fails

load mpt_mpqp_test9
options.qpsolver = 1;      % problem occurs with quadprog
Pn = mpt_mpqp(Matrices);
mbg_assertequal(length(Pn), 2);


return

yalmip('clear')

A1 = [1 .4 .080;0 1 .4;0 0 1];
A2 = [1 .7 .245;0 1 .7;0 0 1];
A3 = [1 .8 .320;0 1 .8;0 0 1];

B1 = [.0107 .08 .4]';
B2 = [.0572 .245 .7]';
B3 = [.0853 .32 .8]';

% Data
nu = 1;
nx = 3;

Q = eye(nx);
R = .1;
P = eye(nx);

% Prediction horizon
N = 4;

% States x(k), ..., x(k+N)
x = sdpvar(repmat(nx,1,N),repmat(1,1,N));

% Inputs u(k), ..., u(k+N) (last one not used)
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));

% Binary for PWA selection
d = binvar(repmat(6,1,N),repmat(1,1,N),'full');

% Value functions
J = cell(1,N);

% Initialize value function at stage N
J{N} = 0;

% DP iteration from final state
F = set([]);
obj = x{N}'*P*x{N};
for k = N-1:-1:1

    % Control constraints
    F = F + set(-2 < u{k} < 2);
   
    % We are in a feasible region...
    F = F + set(-10 < x{k}   < 10);
    F = F + set(-10 < x{k+1}  < 10);

    % PWA Dynamics
    F = F + set(implies(d{k}(1),[x{k+1} == A1*x{k}+B1*u{k}, x{k}(2) + x{k}(3) < 0, -2 <= x{k}(1) <= 2]));
    F = F + set(implies(d{k}(2),[x{k+1} == A2*x{k}+B2*u{k}, x{k}(2) + x{k}(3) > 0, -2 <= x{k}(1) <= 2]));
   
    F = F + set(implies(d{k}(3),[x{k+1} == A3*x{k}+B3*u{k}, x{k}(2) + x{k}(3) < 0,       x{k}(1) <= -2]));
    F = F + set(implies(d{k}(4),[x{k+1} == A3*x{k}+B3*u{k}, x{k}(2) + x{k}(3) < 0,       x{k}(1) >=  2]));
    F = F + set(implies(d{k}(5),[x{k+1} == A3*x{k}+B3*u{k}, x{k}(2) + x{k}(3) > 0,       x{k}(1) <= -2]));
    F = F + set(implies(d{k}(6),[x{k+1} == A3*x{k}+B3*u{k}, x{k}(2) + x{k}(3) > 0,       x{k}(1) >=  2]));
        
    F = F + set(sum(d{k}) == 1);
   
    % L1 objective
    obj = obj + x{k}'*Q*x{k} + u{k}'*R*u{k};       
end
[mpsol{k},sol{k},aux{k},J{k},U{k}] = solvemp(F,obj,sdpsettings('debug',1),x{k},u{k});
