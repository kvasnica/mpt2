function mpt_mpqp_test6

% issue180

M.G = [    1.0000         0         0         0
         0         0         0    1.0000
   -1.0000         0         0         0
         0         0         0   -1.0000
         0    0.9950         0         0
         0         0    0.9901         0
         0   -0.9950         0         0
         0         0   -0.9901         0
   -1.0000   -1.0000   -1.0000   -1.0000
         0    0.0050         0         0
         0         0    0.0099         0
    1.0000    1.0000    1.0000    1.0000
   -1.0000         0         0         0
         0         0         0   -1.0000
         0   -0.9950         0         0
         0         0   -0.9901         0
         0   -0.0050         0         0
         0         0   -0.0099         0
   -0.0909    0.0909    0.1818    0.5455
    0.0909   -0.0909   -0.1818    0.4545
    0.1818   -0.1818   -0.3636   -0.0909
    0.7273    0.2727    0.5455    0.6364
   -0.2727   -0.7273   -0.4545   -0.3636
    0.2727   -0.2727    0.4545    0.3636
    0.0909   -0.0909   -0.1818   -0.5455
   -0.0909    0.0909    0.1818   -0.4545
   -0.1818    0.1818    0.3636    0.0909
   -0.7273   -0.2727   -0.5455   -0.6364
    0.2727    0.7273    0.4545    0.3636
   -0.2727    0.2727   -0.4545   -0.3636];
M.E = [ 0
         0
         0
         0
 -746.2687
 -643.5644
  746.2687
  643.5644
         0
  746.2687
  643.5644
  850.0000
         0
         0
  746.2687
  643.5644
    3.7313
    6.4356
         0
         0
         0
         0
         0
         0
         0
         0
         0
         0
         0
         0];
     
M.W = [2100
        4000
           0
           0
        2200
        2000
           0
           0
           0
           0
           0
           0
           0
           0
           0
           0
           0
           0
        1000
         100
        1000
        1000
         350
        1000
        1000
         100
        1000
        1000
         350
        1000];
M.H = [  -10.2000  -10.0000  -10.0000  -10.0000
  -10.0000  -10.0498  -10.0000  -10.0000
  -10.0000  -10.0000  -10.0990  -10.0000
  -10.0000  -10.0000  -10.0000  -10.0400];
M.F = [    8.5000    8.4627    8.4356    8.5000]*1e3;
M.Y = 4.8901e+006;
M.Cf = zeros(1, 4); M.Cx = 0; M.Cc = 0;
M.bndA = [1; -1];
M.bndb = [1.4; -0.6];

% it was observed that we need Options.zero_tol at least 5e-13 to get _some_
% resutls
[Pn, Fi, Gi, AC, Ph, D] = mpt_mpqp(M, struct('zero_tol', 5e-13));
Pfeas = projection(polytope([-M.E M.G], M.W), size(M.E, 2)) & polytope(M.bndA, M.bndb);
mbg_asserttrue(Pfeas==Pn);
if length(Pn)>1,
    % for this example we should just get one region (as with YALMIP), but as of
    % 04-Dec-2005 we obtain anywhere between 3 and 5 overlapping regions
    warning('Overlapping regions detected!');
end


% comparison to solution of a QP for a given point
C1 = [];
U1 = [];
C2 = [];
U2 = [];
% feasible set is from -0.6 to 1.4
for x0 = 0.6:0.05:1.4,
    
    A = M.G;
    B = M.W + M.E*x0;
    
    H = M.H;
    f = x0'*M.F + M.Cf;
    
    [xopt, lam, how, ef, fval] = mpt_solveQP(H, f, A, B);
    feasible = strcmpi(how, 'ok');
    U = xopt;
    cost = fval + x0'*M.Y*x0 + M.Cf*xopt + M.Cx*x0 + M.Cc;
    
    % identify region in which the cost is minimal. we have to do it like this
    % because regions can overlap for this example
    mincost = [];
    [isin, inwhich] = isinside(Pn, x0);
    for ii = 1:length(inwhich),
        reg = inwhich(ii);
        mincost = [mincost; x0'*D.Ai{reg}*x0 + D.Bi{reg}*x0 + D.Ci{reg}];
    end
    [aa,bb]=min(mincost);
    minreg = inwhich(bb); % region in which cost is minimal
    C1 = [C2; x0 aa];     % minimal cost
    C2 = [C2; x0 cost];
    U1 = [U1; x0 (Fi{minreg}*x0 + Gi{minreg})'];
    U2 = [U2; x0 xopt'];
end

% solutions must be identical
mbg_asserttolequal(C1, C2, 1e-6);
mbg_asserttolequal(U1, U2, 1e-9);
