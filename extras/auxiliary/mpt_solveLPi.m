function [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f,A,B,Aeq,Beq,x0,lpsolver,rescue)
%MPT_SOLVELPi Interface to various LP solvers (version without errorchecks)
%
% [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f,A,B,Aeq,Beq,x0,lpsolver);
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Solves an LP problem:
%
%     min  f'x
%     s.t. A*x<=B,
%          Aeq*x=Beq
%
% by using the method specified in 'lpsolver'
%
% WARNING: This is a "fast" version of mpt_solveLP. No (or at least very few)
% error checks are being performed, all 8 input arguments must be specified, the
% objective vector "f" must be a row vector. You should always call
% "mpt_solveLP" instead of this function, unless you know what you are doing! 
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% f        - Optimization objective (must be a row vector!!!!!!!!)
% A,B      - Matrices defining inequality constraints
% Aeq,Beq  - Matrices defining equality constraints
% x0       - Initial value                         
% lpsolver - Which LP solver to use:
%              lpsolver=0: uses NAG (E04NAG.M)
%              lpsolver=9: uses NAG (E04MBF.M)
%              lpsolver=1: uses linprog.m
%              lpsolver=2: uses CPLEX9 (cplexint)
%              lpsolver=3: uses CDD Criss-Cross (cddmex)
%              lpsolver=4: uses GLPK (glpkmex)
%              lpsolver=5: uses CDD Dual Simplex (cddmex)
%              lpsolver=6: uses SeDuMi
%              lpsolver=7: uses QSopt (qsoptmex)
%              lpsolver=8: uses CPLEX8 (lp_cplex)
%              lpsolver=10: uses XPress
%              lpsolver=11: uses Mosek
%              lpsolver=12: uses OOQP
%              lpsolver=13: uses CLP (mexclp)
%              lpsolver=14: uses BPMPD (bpmpd_mex)
%              lpsolver=15: uses CPLEX (cplexmex)
%
% Note: if 'lpsolver' is not specified, mptOptions.lpsolver will be used instead
%       (see help mpt_init)
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% xopt      - The optimizer
% fval      - Value of the objective
% lambda    - Vector of Lagrangian multipliers
% exitflag  - An integer specifying result of the optimization (depends on the solver)
% how       - States the result of optimization ('ok', 'unbounded', 'infeasible')
%
% see also MPT_SOLVELP

% Copyright is with the following author(s):
%
%(C) 2003-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%              kvasnica@control.ee.ethz.ch
%(C) 2003 Mato Baotic, Automatic Control Laboratory, ETH Zurich,
%         baotic@control.ee.ethz.ch

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

if any(B<-1e9),
    error('mpt_solveLPi: Upper bound exceeded. Most probably the problem is unbounded.');
end

if nargin==8,
    f_orig = f;
    A_orig = A;
    B_orig = B;
    Aeq_orig = Aeq;
    Beq_orig = Beq;
    x0_orig = x0;
end

if (lpsolver==3)
    %=============
    % Fukuda: CDD - Criss-Cross Method
    %=============

    [sA1,sA2] = size(A);

    % bound each variable by 1e9 to avoid numerical problems
    % this will, however, make every problem bounded
    % FIXME: this code should in fact be moved to cddmex interface
    sA2times2 = sA2*2;
    IsA2 = eye(sA2);
    OsA2 = 1e9*ones(sA2times2,1);

    % using cat() is much slower:
    %     IsA2f = cat(1,IsA2,-IsA2);
    %     HA = cat(1, A, IsA2f, Aeq);
    %     HB = cat(1, B, OsA2, Beq);

    %     IsA2 = [IsA2; -IsA2];
    %     HA=[A; IsA2; Aeq];
    %     HB=[B; OsA2; Beq];

    HA=[A; IsA2; -IsA2; Aeq];
    HB=[B; OsA2; Beq];

    
    Hlin = sA1 + sA2times2 + (1:size(Aeq,1));     % prepare indices of equality constraints in H.A
    
    H.A = HA; H.B = HB; H.lin = Hlin; H.obj=f;

    Q=cddmex('solve_lp',H);
    xopt=Q.xopt;
    lambda=-Q.lambda;
    fval=Q.objlp;
    exitflag=Q.how;
    if exitflag==0,
        how = 'undecided';
    elseif exitflag == 1
        how = 'ok';
        return
    elseif exitflag<=5,
        % 3 = dual-inconsistent solution, solve the problem using linprog
        how = 'infeasible';
    elseif exitflag==6 | exitflag==7,
        how = 'unbounded';
    else
        error('mpt_solveLPi: other error code in "exitflag" from cdd')
    end
    
    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver,1);
        end
    end

elseif (lpsolver==0)
    %===============
    % NAG: e04naf.m
    %===============

    if (~isempty(Aeq)),
        % convert equality constraints Aeq x=Beq into Aeq x<=Beq and Aeq x>=Beq
        A = [A; Aeq; -Aeq];  
        B = [B; Beq; -Beq];
    end
    
    if any(isnan(B)),
        B(find(isnan(B)))=1e9; % convert NaN bounds to some large number
    end
    [m,n] = size(A);
    
    if isempty(x0)
        x0=zeros(n,1);
    end

    itmax=450;
   
    H = zeros(n);
    bl=-1e9*ones(m+n,1);
    bu=[1e9*ones(n,1);B];
    lp=1;   % this flag is important! states that e04naf should solve an LP
    cold=1;
    istate = zeros(length(bu),1);
    featol = sqrt(eps)*ones(length(bu),1);
    msglvl=-1;
    bigbnd = 1e10;
    orthog = 1;
    ifail=1;
    
    [xopt,iter,fval,clambda,istate,exitflag] = e04naf(bl,bu,'mpt_qphess',x0,f(:),A,H,...
        lp,cold,istate,featol,msglvl,itmax,bigbnd,orthog,ifail);

    lambda=-clambda(n+1:m+n);
    
    switch exitflag
        case {0,1,3}
            how = 'ok';
            exitflag = 0;
            return
        case 2
            how = 'unbounded';
        case 6
            how = 'infeasible';
        case {9}
            disp('mpt_solveLPi: An input parameter is invalid.');
            how = 'infeasible';
        otherwise
            how = 'infeasible';
    end
    
    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

elseif (lpsolver==9)
    %===============
    % NAG: e04mbf.m
    %===============

    [m,n]=size(A);

    if isempty(x0)
        x0=zeros(n,1);
    end
    
    itmax=450;

    aux1=m;
    aux2=n;
    
    if any(isnan(B)),
        B(find(isnan(B)))=1e9; % convert NaN bounds to some large number
    end

    [xopt,istate,fval,clambda,exitflag]=e04mbf([-1e9*ones(aux1+aux2,1);Beq],...
        [1e9*ones(aux2,1);B;Beq],x0(:),f(:),[A;Aeq],-1,itmax,1);

    [aux1,aux2]=size([A;Aeq]);
    lambda=-clambda(aux2+1:aux1+aux2);

    switch exitflag
        case 0
            how = 'ok';
            return
        case 2
            how = 'unbounded';
        case 1
            how = 'infeasible';
        case {3,4}
            % might also be considered as infeasible
            how = 'infeasible';
        case {5}
            disp('mpt_solveLPi: An input parameter is invalid.');
            how = 'infeasible';
        otherwise
            error('mpt_solveLPi: other error code in "ifail" from e04mbf')
    end

    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

    
    
elseif (lpsolver==1)
    %case 1,
    %=================
    % Matlab: linprog
    %=================

    [m,n]=size(A);
    if isempty(x0)
        x0=zeros(n,1);
    else
        if length(x0(:))~=n,
            error('mpt_solveLP: Wrong dimension of x0!');
        end
    end

    options = optimset('Display','off','LargeScale','off');
    [xopt,fval,exitflag,output,lambdav]=linprog(f(:),A,B,Aeq,Beq,[],[],x0,options);
    lambda=lambdav.ineqlin;
    if exitflag>0 %then LINPROG converged with a solution X.
        how = 'ok';
        return
    else
        how = 'infeasible';
    end

    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);          
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

    
elseif (lpsolver==2)
    % CPLEX 9
    
    H = [];
    nc = size(A,1);
    nx = size(A,2);
    A = [A; Aeq];
    B = [B; Beq];
    INDEQ = (nc+1:size(A,1))';  % indices of equality constraints
    LB = [];
    UB = [];
    OPTIONS.verbose = 0;
    PARAM.int = [1030, 1];  % enable presolver
    VARTYPE = []; % all variables are continuous
    PARAM = [];
    
    [xopt,fval,SOLSTAT,DETAILS] = cplexint([], f(:), A, B, INDEQ, [], LB, UB, VARTYPE, PARAM, OPTIONS);

    lambda = -DETAILS.dual;
    how = lower(DETAILS.statstring);
    exitflag = -1;
    if strcmpi(how, 'optimal')
        how = 'ok';
        exitflag = 0;
        return
    end
    
    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

    
    
elseif (lpsolver==8)
    %=============
    % ILOG: CPLEX 8
    %=============

    ctype= char('L'*ones(size(A,1)+size(Aeq,1),1));
    ctype(size(A,1)+1:size(A,1)+size(Aeq,1))=char('E');

    OPTIONS.int=[1062, 2];   % use the dual-simplex method
    [xopt,fval,exitflag,slack,lambda]=lp_cplex(1,f(:),[A;Aeq],[B;Beq],ctype,...
        -1e9*ones(size(A,2),1),1e9*ones(size(A,2),1),[],[],[],[],0,OPTIONS);

    % Matlab-to-Cplex interface ver 1.0 used this call:
    % [xopt,fval,exitflag,lambda] = lp_cplex(1,f(:),[A;Aeq],[B;Beq],ctype,...
    %     -1e9*ones(size(A,2),1),1e9*ones(size(A,2),1),[],[],[],1);
    %
    % Note: get the latest version of the interface from:
    % http://control.ee.ethz.ch/~hybrid/cplexint.msql

    % solve problem with the primal simplex method (last argument=0)
    % solve problem with the dual simplex method (last argument=1)
    % solve problem with the barrier method (last argument=2)
    % mptchg [xopt,fval,exitflag,lambda] = lp_cplex(1,f(:),A,B,ctype,[],[],[],[],[],1);

    % WHY DO WE CHOOSE DUAL SIMPLEX ?
    % Quote from ILOG CPLEX 7.0 User's manual (page 93):
    % If you are familiar with linear programming theory,
    % then you recall that a linear programming problem can
    % be stated in primal or dual form, and an optimal solution
    % (if one exists) of the dual has a direct relationship to
    % an optimal solution of the primal model.
    % CPLEX's dual simplex optimizer makes use of this relationship,
    % but still reports the solution in terms of the primal model.
    % Recent computational advances in the dual simplex method have
    % made it the first choice for optimizing a linear programming
    % problem. This is especially true for primal-degenerate problems
    % with little variability in the right-hand side coefficients but
    % significant variability in the cost coefficients.

    %     % old solution status codes (valid for CPLEX 7.0)
    %     switch exitflag
    %     case {1}   % feasible
    %         how = 'ok';
    %     case {0,2} % infeasible
    %         how = 'infeasible';
    %     case {3}   % unbounded
    %         how = 'unbounded';
    %     otherwise
    %         how = 'infeasible';
    %     end


    % new solution status codes (valid for CPLEX 8.0)
    switch exitflag
        case {1}   % feasible, optimal
            how = 'ok';
            return
        case {3}   % infeasible
            how = 'infeasible';
        case {2}   % unbounded
            how = 'unbounded';
        case {4}
            how = 'InfOrUnbd';
        otherwise
            how = 'infeasible';
    end

    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

    
elseif (lpsolver==4)
    %case 4,
    % GLPK solver interfaced by GLPKmex

    ctype= char('U'*ones(size(A,1)+size(Aeq,1),1));
    vartype=char('C'*ones(size(f(:),1),1));
    ctype(size(A,1)+1:size(A,1)+size(Aeq,1))=char('S');
    param.msglev=1;
    param.itlim=500;
    method=1; % 1 = solve problem with the revised simplex method
    % 2 = solve problem with the interior point method

    [xopt,fval,status,lambda_struct] = glpkmex(1,f(:),[A;Aeq],[B;Beq],ctype,...
        -1e9*ones(size(A,2),1),1e9*ones(size(A,2),1),vartype,param,method,0);

    lambda = lambda_struct.lambda;
    switch status
        case {180,181,151}   % optimal, feasible
            how = 'ok';
            exitflag=1;
            return
        case {182}   % infeasible
            how = 'infeasible';
            exitflag=3;
        case {184}   % unbounded
            how = 'unbounded';
            exitflag=2;
        case {183}   % problem has no feasible solution
            how = 'InfOrUnbd';
            exitflag=3;
        otherwise
            exitflag=4;
            how = 'infeasible';
    end

    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

   
elseif (lpsolver==5)
    %case 5
    %=============
    % Fukuda: CDD - Dual Simplex Method
    %=============

    H.A=[A;eye(size(A,2));-eye(size(A,2));Aeq];
    H.B=[B;1e9*ones(size(A,2),1);1e9*ones(size(A,2),1);Beq];
    H.lin=size(A,1)+2*size(A,2)+(1:size(Aeq,1));     % prepare indices of equality constraints in H.A
    H.obj=f; 
    Q=cddmex('solve_lp_DS',H);
    xopt=Q.xopt;
    lambda=-Q.lambda;
    fval=Q.objlp;
    exitflag=Q.how;
    switch exitflag
        case 0
            how = 'undecided';
        case 1
            how = 'ok';
            return
        case {2,3,4,5}
            % 3 = dual-inconsistent solution
            how = 'infeasible';
        case {6,7}
            how = 'unbounded';
        otherwise
            error('mpt_solveLPi: other error code in "exitflag" from cdd')
    end

    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

    
elseif (lpsolver==6)
    % SeDuMi
    [xopt,fval,lambda,exitflag,how]=yalmipLP(f,A,B,Aeq,Beq,x0,'sedumi');
    if nargin==8 & exitflag ~= 1,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

    
elseif (lpsolver==7)
    %case 7,
    %=======
    % QSopt
    %=======
    
    [m,n]=size(A);

    IsA2 = eye(n);
    OsA2 = 1e9*ones(2*n,1);

    A=[A; IsA2; -IsA2];
    B=[B; OsA2];

    options = optimset('Display','off');
    [xopt,lambda,status]=qsopt(f,A,B,Aeq,Beq);
    fval = f(:)'*xopt;
    if status==1 
        how = 'ok';
        exitflag=0;
        return
    else
        how = 'infeasible';
        exitflag=1;
    end
    
    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

elseif (lpsolver==10)

    %========
    % XPress
    %========
    [xopt,fval,lambda,exitflag,how]=yalmipLP(f,A,B,Aeq,Beq,x0,'xpress');
    if nargin==8 & exitflag ~= 1,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

elseif (lpsolver==11)
    
    %========
    % Mosek
    %========
    [xopt,fval,lambda,exitflag,how]=yalmipLP(f,A,B,Aeq,Beq,x0,'mosek');
    if nargin==8 & exitflag ~= 1,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

elseif (lpsolver==12)
    
    %========
    % OOQP
    %========
    [xopt,fval,lambda,exitflag,how]=yalmipLP(f,A,B,Aeq,Beq,x0,'ooqp');
    if nargin==8 & exitflag ~= 1,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end
    
elseif (lpsolver==13)
    
    %========
    % CLP
    %========
    
    [xopt, lambda, exitflag] = clp([], f, A, B, Aeq, Beq);
    
    fval = f(:)'*xopt;
    
    if exitflag==0,
        how = 'ok';
        return
    elseif exitflag==1,
        how = 'infeasible';
    else
        how = 'unbounded';
    end
    
    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

elseif (lpsolver==14)
    %========
    % BPMPD
    %========

    nA = size(A, 1);
    nAeq = size(Aeq, 1);
    An = sparse([A; Aeq]);
    Bn = [B; Beq];
    ind = [repmat(-1, nA, 1); repmat(0, nAeq, 1)];
    
    [xopt,y,s,w,howout] = bp([], An, Bn, f, ind);
    lambda = -y;
    
    switch howout,
        case 'optimal solution'
            exitflag = 0;
        otherwise
            exitflag = 1;
    end
    fval = f(:)'*xopt;
    
    if exitflag==0,
        how = 'ok';
        return
    elseif exitflag==1,
        how = 'infeasible';
    end
    
    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

elseif (lpsolver==15)
    % CPLEX interfaced with CPLEXMEX (by Nicolo Giorgetti)
    
    SENSE = 1; % minimize
    H = [];    % no quadratic term in objective function
    F = f(:);  % linear objective must be a column vector

    [nc, nx] = size(A);
    A = [A; Aeq];
    B = [B; Beq];
    
    % type of constraints
    % 'L' - less than or equal (<=)
    % 'E' - equality constraint (=)
    CTYPE = [repmat('L', nc, 1); repmat('E', size(Aeq, 1), 1)];
    
    % all variables are continuous
    VARTYPE = repmat('C', nx, 1);
    
    % lower an upper bounds on optimization variables
    LB = [];
    UB = [];

    PARAM.errmsg = 0;
    PARAM.niter = 1e4; % release CPLEX license after 10000 runs of the LP algorithm

    [xopt,fval,exitflag,DETAILS] = cplexmex(SENSE,H,F,A,B,CTYPE,LB,UB,VARTYPE,x0,PARAM);

    lambda = -DETAILS.lambda;
    
    if exitflag == 1,
        % optimal solution found
        how = 'ok';
        return
    elseif exitflag == 3,
        how = 'infeasible';
    elseif exitflag == 2,
        how = 'unbounded';
    else
        how = 'unknown';
    end
    
    % since we exit if optimal solution was found, we reach this code only if
    % the problem is infeasible or unbounded. in this case we try another
    % solver.
    if nargin==8,
        % solution is not optimal, try another solver
        global mptOptions
        lpsolvers = mptOptions.solvers.lp;
        
        % get position of curent solver
        curpos = find(lpsolvers==lpsolver);
        
        if curpos < length(lpsolvers)
            % if this is not the last solver in the list, try other solver
            nextsolver = lpsolvers(curpos+1);
            [xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f_orig, A_orig, B_orig, ...
                Aeq_orig, Beq_orig, x0_orig, nextsolver, 1);
        end
    end

else
    error('mpt_solveLP: unknown LP solver specified!');
end


%-------------------------------------------------------------------------------
function [xopt,fval,lambda,exitflag,how]=yalmipLP(f,A,B,Aeq,Beq,x0,solver)

global mptOptions

f = f(:);
nx = size(A,2);
x = sdpvar(nx,1);
F = set(A*x <= B);
if ~isempty(Aeq),
    F = F + set(Aeq*x == Beq);
end
options = mptOptions.sdpsettings;
options.solver = solver;
%options=sdpsettings('Verbose',0,'warning',0,'solver',solver);
solution = solvesdp(F, f'*x, options);
xopt = double(x);
lambda = [A;Aeq]*xopt - [B; Beq];
fval = f'*xopt;
if solution.problem==0,
    how = 'ok';
    exitflag = 1;
else
    how = 'infeasible';
    exitflag = -1;
end
