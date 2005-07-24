function [xmin,fmin,how,exitflag]=mpt_solveMIQP(H,f,A,B,Aeq,Beq,lb,ub,vartype,param,options,solver)
%MPT_SOLVEMILP Interface to various MIQP solvers
%
% [xmin,fmin,how,exitflag]=mpt_solveMIQP(H,f,A,B,Aeq,Beq,lb,ub,vartype,param,options,solver)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Solves an MILP problem:
%
%     min  0.5 x' H x + f'x
%     s.t. A*x (<=,=) B
%          some 'x' integer/boolean
%
% by using the method specified in 'solver'
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% H          An (n x n) SYMETRIC, POSITIVE SEMIDEFINITE matrix (in full or sparse
%            format) containing the quadratic objective function coefficients.
%
% f          An (n x 1) vector containing the linear objective function coefficients.
%            REQUIRED INPUT ARGUMENT.
% 
% A          An (m x n) matrix (in full or sparse format) containing the constraint
%            coefficients. REQUIRED INPUT ARGUMENT.
% 
% b          An (m x 1) vector containing the right-hand side value for each
%            constraint in the constraint matrix. REQUIRED INPUT ARGUMENT.
% 
% Aeq        An (k x n) matrix (in full or sparse format) containing the constraint
%            coefficients for equality constraints (i.e. Aeq*x = Beq)
% 
% beq        An (k x 1) vector containing the right-hand side value for each
%            constraint in the constraint matrix for equality constraints
%
% LB         An (n x 1) vector containing the lower bound on each of the variables.
%            Any lower bound that is set to a value less than or equal to that of
%            the constant -CPX_INFBOUND will be treated as negative \infty.
%            CPX_INFBOUND is defined in the header file cplex.h.
%            Default: [], (lower bound of all variables set to -CPX_INFBOUND).
% 
% UB         An (n x 1) vector containing the upper bound on each of the variables.
%            Any upper bound that is set to a value greater than or equal to that of
%            the constant CPX_INFBOUND will be treated as \infty.
%            CPX_INFBOUND is defined in the header file cplex.h.
%            Default: [], (upper bound of all variables set to CPX_INFBOUND).
%
% VARTYPE    An (n x 1) vector containing the types of the variables
%            VARTYPE(i) = 'C' Continuous variable
%            VARTYPE(i) = 'B' Binary(0/1) variable
%            VARTYPE(i) = 'I' Integer variable
%            VARTYPE(i) = 'S' Semi-continuous variable
%            VARTYPE(i) = 'N' Semi-integer variable
%            (This is case sensitive).
%            Default: [], (all variables are continuous).
%
% solver     which solver to use:
%              0 - CPLEX 9 (cplexint)
%              1 - YALMIP Branch & Bound
%              2 - XPRESS
%              3 - MOSEK
%              4 - CPLEX 8 (cplexmex)
%
% Note: if 'solver' is not specified, mptOptions.miqpsolver will be used instead
%       (see help mpt_init)
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% xopt      - The optimizer
% fmin      - Value of the objective
% exitflag  - An integer specifying result of the optimization
%             (1 - optimal solution found, -1 - problem is infeasible)
% how       - States the result of optimization ('ok', 'unbounded', 'infeasible')
%
% see also MPT_SOLVEQP, MPT_SOLVEMILP

% $Id: mpt_solveMIQP.m,v 1.4 2005/03/13 16:50:52 kvasnica Exp $
%
%(C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%         kvasnica@control.ee.ethz.ch

% ---------------------------------------------------------------------------
% Legal note:
%          This program is free software; you can redistribute it and/or
%          modify it under the terms of the GNU General Public
%          License as published by the Free Software Foundation; either
%          version 2.1 of the License, or (at your option) any later version.
%
%          This program is distributed in the hope that it will be useful,
%          but WITHOUT ANY WARRANTY; without even the implied warranty of
%          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%          General Public License for more details.
%
%          You should have received a copy of the GNU General Public
%          License along with this library; if not, write to the
%          Free Software Foundation, Inc.,
%          59 Temple Place, Suite 330,
%          Boston, MA  02111-1307  USA
%
% ---------------------------------------------------------------------------

global mptOptions

error(nargchk(4,12,nargin));

if ~isstruct(mptOptions)
    mpt_error;
end

% set default values for parameters which have not been specified
if nargin <= 4,
    Aeq = [];
    Beq = [];
    lb = [];
    ub = [];
    vartype = [];
    param = [];
    options = [];
    solver = mptOptions.miqpsolver;
elseif nargin <= 5,
    error('mpt_solveMIQP: you must specify ''Beq'' as well!');
elseif nargin <= 6,
    lb = [];
    ub = [];
    vartype = [];
    param = [];
    options = [];
    solver = mptOptions.miqpsolver;
elseif nargin <= 7,
    error('mpt_solveMIQP: you must specify ''ub'' as well!');
elseif nargin <= 8,
    vartype = [];
    param = [];
    options = [];
    solver = mptOptions.miqpsolver;
elseif nargin <= 9,
    param = [];
    options = [];
    solver = mptOptions.miqpsolver;
elseif nargin <= 10,
    options = [];
    solver = mptOptions.miqpsolver;
elseif nargin <= 11,
    solver = mptOptions.miqpsolver;
end

if solver==0
    % use cplex 9
    indeq = [];
    if ~isempty(Aeq),
        nc = size(A,1);
        A = [A; Aeq];
        B = [B; Beq];
        indeq = (nc+1:size(A,1))';
    end
    [xmin,fmin,status,details]=cplexint(H, f(:), A, B, indeq, [], lb, ub, vartype, param, options);
    how = lower(details.statstring);
    exitflag = -1;
    if strcmp(how, 'optimal') | strcmp(how, 'optimaltol') | ...
            strcmp(how,'integer optimal solution') | strcmp(how, 'integer optimal, tolerance'),
        how = 'ok';
        exitflag = 1;
    end
   
elseif solver==1
    % use YALMIP / BNB
    [xmin,fmin,how,exitflag]=yalmipMIQP(H,f,A,B,Aeq,Beq,lb,ub,vartype,param,options,'bnb');

elseif solver==2
    % use YALMIP / XPress
    [xmin,fmin,how,exitflag]=yalmipMIQP(H,f,A,B,Aeq,Beq,lb,ub,vartype,param,options,'xpress');

elseif solver==3
    % use YALMIP / Mosek
    [xmin,fmin,how,exitflag]=yalmipMIQP(H,f,A,B,Aeq,Beq,lb,ub,vartype,param,options,'mosek');

elseif solver==4
    % use YALMIP / CPLEX8 (cplexmex)
    [xmin,fmin,how,exitflag]=yalmipMIQP(H,f,A,B,Aeq,Beq,lb,ub,vartype,param,options,'cplex-miqp-cplexint');

elseif solver==-1,
    % no solver available
    error('mpt_solveMIQP: no MIQP solver available!');
    
else
    error('mpt_solveMIQP: unknown solver');
end


%---------------------------------------------------------------------------
function [xmin,fmin,how,exitflag]=yalmipMIQP(H,f,A,B,Aeq,Beq,lb,ub,vartype,param,options,solver)

global mptOptions

[nc,nx] = size(A);
f = f(:);
x = sdpvar(nx,1);
F = set(A*x <= B);
if ~isempty(Aeq),
    F = F + set(Aeq*x == Beq);
end
if ~isempty(vartype),
    indbin = find(vartype=='B');
    if ~isempty(indbin),
        F = F + set(binary(x(indbin)));
    end
    indint = find(vartype=='I');
    if ~isempty(indint),
        F = F + set(integer(x(indint)));
    end
end
if ~isempty(lb),
    F = F + set(lb <= x);
end
if ~isempty(ub)
    F = F + set(x <= ub);
end
if ~isempty(options),
    options=sdpsettings(options,'Verbose',0,'warning',0,'solver',solver);
else
    options = mptOptions.sdpsettings;
    options.solver = solver;
end
solution = solvesdp(F, 0.5*x'*H*x + f'*x, options);
xmin = double(x);
fmin = 0.5*xmin'*H*xmin + f'*xmin;

if solution.problem==0,
    how = 'ok';
    exitflag = 1;
elseif solution.problem < 0,
    how = 'nosolver';
    exitflag = -3;
else
    how = 'infeasible';
    exitflag = -1;
end
