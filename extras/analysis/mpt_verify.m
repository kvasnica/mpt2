function [flag, N, Vreach] = mpt_verify(object, X0, Xf, N, U0, Options)
%MPT_VERIFY Verifies if states enter a given set in a given number of steps
%
% [flag, N, Vreach] = mpt_verify(sysStruct, X0, Xf, N, U0)
% [flag, N, Vreach] = mpt_verify(controller, X0, Xf, N)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Checks if states of a dynamical system subject to:
%  1. inputs driven by an explicit control law
%  2. inputs which belong to a set of admissible inputs U0
% enter a given set Xf, assuming x0 \in X0 in N steps.
%
% USAGE:
%   flag = mpt_verify(sysStruct, X0, Xf, N, U0)
%   flag = mpt_verify(controller, X0, Xf, N)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% object       - either a sysStruct structure, or an explicit controller
% X0           - set of initial states (a polytope or a polyarray)
% Xf           - target set (a polytope or a polyarray)
% N            - number of steps over which to compute reachable sets
% U0           - set of admissible inputs (polytope object)
% Options.usereachsets 
%              - if set to 0 (default is 1), verification question for systems
%                described by sysStruct structures will be performed by solving
%                an LP/MILP problem. Set this option to true if you want to base
%                the verification on reachable sets.
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% flag        - 1 if Xf is reachable from X0, 0 otherwise
% N           - number of steps in which Xf is reachable from X0 ([] if Xf is
%               not reachable)
% Vreach      - V-representation of reachable sets
%
% see also MPT_REACHSETS, MPT_REACHXU
%

% $Id: mpt_verify.m,v 1.3 2005/06/27 13:50:54 kvasnica Exp $
%
% (C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch

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
if ~isstruct(mptOptions),
    mpt_error;
end

error(nargchk(4,6,nargin));

if isa(object, 'mptctrl'),
    if ~isexplicit(object),
        error('This function supports only explicit controllers!');
    end
end

if ~isa(X0, 'polytope'),
    error('Set of initial states must be a polytope!');
end
if ~isa(N, 'double'),
    error('Number of steps must be a positive integer!');
end
if N<1 | any(size(N)~=1),
    error('Number of steps must be a positive integer!');
end
if ~isa(Xf, 'polytope'),
    error('Set of final states must be a polytope!');
end
if ~isfulldim(X0),
    error('Set of initial states must be a fully dimensional polytope!');
end
if ~isfulldim(Xf),
    error('Set of final states must be a fully dimensional polytope!');
end
if nargin<=4,
    Options = [];
elseif nargin == 5,
    if isa(U0, 'struct'),
        Options = U0;
    else
        Options = [];
    end
end

if ~isfield(Options, 'usereachsets'),
    Options.usereachsets = 1;
end
if ~isfield(Options, 'abs_tol'),
    Options.abs_tol = mptOptions.abs_tol;
end
if ~isfield(Options, 'verbose'),
    Options.verbose = mptOptions.verbose;
end
if ~isfield(Options, 'lpsolver'),
    Options.lpsolver = mptOptions.lpsolver;
end

Options.Xf = Xf;

if nargin==4 | (nargin==5 & ~isa(U0, 'polytope')),
    % verification with no U0
    havemld = 0;
    if mpt_isSysStruct(object) & Options.usereachsets==0,
        % input is a system structure. 
        sysStruct = object;

        if ~isfield(sysStruct, 'verified'),
            sysStruct = mpt_verifySysStruct(sysStruct);
        end
        [nx, nu, ny, ndyn, nbool] = mpt_sysStructInfo(sysStruct);
        
        for ii = 1:length(Xf)
            % pose an optimal control problem for each element of Xf as terminal
            % set
            probStruct.Tset = Xf(ii);
            probStruct.Tconstraint = 2;
            
            % weights can be arbitrary, we will solve feasibility problem
            probStruct.Q = eye(nx);
            probStruct.R = eye(nu);
            
            % use 2-norm to avoid generating of slacks, which we don't need.
            probStruct.norm = 2;
            
            % prediction horizon
            probStruct.N = N;
            probStruct.subopt_lev = 0;
            
            % create a controller object which has matrices of the optimal
            % control problem stored inside.            %
            
            ctrl = mptctrl(sysStruct, probStruct, struct('verbose', -1));
            if isfield(ctrl.sysStruct, 'data')
                if isfield(ctrl.sysStruct.data, 'MLD'),
                    havemld = 1;
                end
            end
            
            if havemld,
                sysStruct = ctrl.sysStruct;
                % we have MLD model, formulate the reachability problem as
                % a control problem with no cost (i.e. feasibility problem),
                % assuming Tset = Xf
                
                % the CFTOC is:
                %  min  v' S1 v + 2(S2 + x0' S3) v
                %  s.t. F1 v   <= F2  + F3 x0

                % extract matrices of constraints
                F1 = ctrl.details.Matrices.F1;
                F2 = ctrl.details.Matrices.F2;
                F3 = ctrl.details.Matrices.F3;
                
                % IntIndex holds indices of binary variables
                IntIndex = ctrl.details.Matrices.IntIndex;
                
                [H0, K0] = double(X0);
                
                % include x0 as a parameter, limit it such that x0 \in X0
                A = [F1 -F3; zeros(size(H0, 1), size(F1, 2)) H0];
                b = [F2; K0];
                
                % number of boolean states of the system
                nxbool = sysStruct.data.MLD.nxb;
                nxreal = sysStruct.data.MLD.nxr;
                
                % since we have added x0 as a parameter, add indices of boolean
                % states (if any)
                if nxbool > 0,
                    index_bool = (size(F1, 2) + nxreal) + (1:nxbool);
                    IntIndex = [IntIndex; index_bool];
                end
                
                % solve the feasibility MILP
                nvars = size(A, 2);
                vartype = repmat('C', nvars, 1);
                if ~isempty(IntIndex),
                    vartype(IntIndex) = 'B';
                    lb = repmat(1e-9, nvars, 1);
                    ub = repmat(1e9, nvars, 1);
                    lb(IntIndex) = 0;
                    ub(IntIndex) = 1;
                else
                    lb = [];
                    ub = [];
                end
                nlin = length(b);
                epsil = Options.abs_tol;
                
                % IMPORTANT! relax constraints little bit, otherwise we can
                % easily run into feasibility problems
                if Options.verbose > -1,
                    fprintf('Performing verification based on MILP...\n');
                end
                [umin, fmin, how, ef] = mpt_solveMILP(zeros(1, nvars), A, b+epsil*ones(nlin,1), [], [], lb, ub, vartype);
                if ef==1,
                    % feasible solution found
                    flag = 1;
                    Vreach = umin;
                    return
                else
                    % problem is infeasible, no states enter Xf in N steps
                    flag = 0;
                    Vreach = [];
                end
            else
                % otherwise we have a linear system
                G = ctrl.details.Matrices.G;
                W = ctrl.details.Matrices.W;
                E = ctrl.details.Matrices.E;
                
                if nbool > 0,
                    % check if all inputs are strictly boolean
                    [valid,nu,nbool,ubool,uint] = validboolinput(sysStruct);
                    if ~valid,
                        % not all inputs are stricly boolean! compute reachable
                        % sets in this case
                        if Options.verbose > -1,
                            fprintf('Performing verification based on reachable sets...\n');
                        end
                        [res, Vreach] = mpt_reachSets(object, X0, N, Options);
                        return
                    else
                        bvartype = repmat('C', nu, 1);
                        LB = -1e9*ones(size(G,2),1);
                        UB = 1e9*ones(size(G,2),1);
                        for ii=1:length(uint),
                            if uint{ii}.min==0 & uint{ii}.max==1,
                                % this input is boolean
                                bvartype(uint{ii}.uindex) = 'B';
                            else
                                % otherwise the input is integer
                                bvartype(uint{ii}.uindex) = 'I';
                            end
                            for in=1:probStruct.N,
                                % add proper bounds for boolean/integer variables
                                LB((in-1)*nu+uint{ii}.uindex) = uint{ii}.min;
                                UB((in-1)*nu+uint{ii}.uindex) = uint{ii}.max;
                            end
                        end
                        vartype = [repmat(bvartype, probStruct.N, 1)]; 
                        
                        % now add boolean states corresponding to x0
                        if isfield(sysStruct, 'xbool'),
                            xbool = sysStruct.xbool;
                        else
                            xbool = [];
                        end
                        xvartype = repmat('C', nx, 1);
                        if isfield(sysStruct, 'xmin') & isfield(sysStruct, 'xmax'),
                            lb = sysStruct.xmin;
                            ub = sysStruct.xmax;
                        else
                            lb = repmat(1e-9, nx, 1);
                            ub = repmat(1e-9, nx, 1);
                            lb(xbool) = 0;
                            ub(xbool) = 1;
                        end
                        if isfield(sysStruct, 'xbool'),
                            xvartype(xbool) = 'B';
                        end
                        vartype = [vartype; xvartype];
                        LB = [LB; lb];
                        UB = [UB; ub];
                    end
                end
                
                [H0, K0] = double(X0);
                
                % include x0 as a parameter, limit it such that x0 \in X0
                A = [G -E; zeros(size(H0, 1), size(G, 2)) H0];
                b = [W; K0];
                nlin = length(b);
                % relax constraints slightly
                b + Options.abs_tol*ones(nlin, 1);
                if nbool > 0,
                    % solve MILP
                    if Options.verbose > -1,
                        fprintf('Performing verification based on MILP...\n');
                    end
                    [U,fmin,how,exitflag]=mpt_solveMILP(zeros(size(A, 2), 1), A, b, [], [], LB, UB, vartype);
                else
                    % solve LP
                    if Options.verbose > -1,
                        fprintf('Performing verification based on LP...\n');
                    end
                    
                    % switch to a faster solver if possible
                    defaultLP = Options.lpsolver;
                    prefered_solvers = [2 8 15 0 9 10 11 12 13 14 15 7 4 1 3 5];
                    for ii = 1:length(prefered_solvers),
                        if any(ismember(mptOptions.solvers.lp_all, prefered_solvers(ii)))
                            lpsolver = prefered_solvers(ii);
                            if lpsolver~=defaultLP,
                                fprintf('Switching to solver %s (faster computation)...\n', mpt_solverInfo('lp', lpsolver));
                            end
                            mpt_options('lpsolver', lpsolver);
                            break
                        end
                    end
                    try
                        [U,fmin,lambda,exitflag,how]=mpt_solveLP(zeros(size(A, 2), 1), A, b);
                    end
                    % restore default LP solver
                    mpt_options('lpsolver', defaultLP);
                end
                if strcmp(how, 'ok') | strcmp(how, 'optimal'),
                    % feasible solution exists
                    flag = 1;
                    Vreach = U;
                    return
                else
                    flag = 0;
                    Vreach = [];
                end
            end
        end % length(Xf)
        return
    else
        % otherwise solve the verification problem using reach sets
        if Options.verbose > -1,
            fprintf('Performing verification based on reachable sets...\n');
        end
        [res, Vreach] = mpt_reachSets(object, X0, N, Options);
    end
else
    if ~isa(U0, 'polytope'),
        error('Set of admissible inputs must be a polytope!');
    end
    if ~isfulldim(U0),
        error('Set of admissible inputs must be a fully dimensional polytope!');
    end
    if Options.verbose > -1,
        fprintf('Performing verification based on reachable sets...\n');
    end
    [res, Vreach] = mpt_reachSets(object, X0, U0, N, Options);
end

if isa(res, 'double'),
    N = res;
    flag = 1;
else
    N = [];
    flag = 0;
end

%------------------------------------------------------------------------
function [valid,nu,nbool,ubool,uint] = validboolinput(sysStruct)
% checks if all inputs specified in sysStruct.Uset are purely boolean
% i.e. no finite alphabet involving doubles or integer inputs

[nx,nu,ny,ndyn,nbool,ubool] = mpt_sysStructInfo(sysStruct);
if ~isfield(sysStruct,'Uset')
    valid = 0;
    nbool = [];
    ubool = [];
    uint = {};
    return
end
uint = {};
for ii=1:length(ubool)
    ib = ubool(ii);
    Uset = sysStruct.Uset{ib};
    if any(Uset~=ceil(Uset)) | any(isinf(Uset))
        % first rule out non-integer inputs
        valid = 0;
        return
    else
        minU = min(Uset);
        maxU = max(Uset);
        if ~isempty(setdiff(minU:maxU,Uset))
            % rule out "gaps" in integers, e.g. [-2 -1 1 3] -> error
            valid = 0;
            return
        else
            uints.min = minU;
            uints.max = maxU;
            uints.uindex = ib;
            uint{ii} = uints;
        end
    end
end
valid = 1;