function [U,feasible,region,cost,inwhich,fullopt,runtime]=mpt_getInput(ctrl,x0,Options)
%MPT_GETINPUT For a given state, extracts the optimal output from controller structure
%
% [U,feasible,region,cost]=mpt_getInput(ctrlStruct,x0,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% For the given state x0, this function extracts the optimal output from a controller
% given by means of the controller structure ctrlStruct. If the controller partition
% is overlapping in the X space, the input U will be picked up such that an associated
% cost is minimized (i.e. if for x0 there are 2 or more associated control laws,
% only the one which minimizes a given criterion is returned. The criterion is
% either value of the objective function for optimal solution or minimum time
% for the time-optimal solution).
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl              - MPT controller
% x0                - initial state
% Options.openloop  - 0 by default. If set to 1, the full optimizer as obtained
%                     as a solution to the finite-time optimal control problem
%                     is returned, i.e. U = [u_0 u_1 ... u_N] where N is the
%                     prediction horizon
% Options.abs_tol   - absolute tolerance
% Options.verbose   - Level of verbosity
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% U         - control input computed as U=F*x0 + G,
% region    - index of a region which contains the optimal control input associated
%             to the given state x0
% cost      - value of the associated cost function
%             NOTE: The cost is not necessarily the cost of the objective function
%                   obtained for state x0, it can be also distance to the invariant
%                   set(s) in case of time-optimal controller!
% feasible  - returns 1 if  the there is at least one control law associated to
%             a given state x0, 0 otherwise
%
% see also MPT_COMPUTETRAJECTORY, MPT_PLOTTIMETRAJECTORY
%

% Copyright is with the following author(s):
%
% (C) 2003-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%               kvasnica@control.ee.ethz.ch
% (C) 2004 Arne Linder, Faculty of Electrical, Information and Media Engineering, 
%          Wuppertal University, alinder@uni-wuppertal.de

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

error(nargchk(2,3,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if ~isa(ctrl, 'mptctrl') & ~isstruct(ctrl)
    error('MPT_GETINPUT: first argument must be an MPTCTRL object!');
end

if ~isa(x0, 'double')
    error('MPT_GETINPUT: second argument must be vector!');
end

if nargin==3 & ~isstruct(Options)
    error('MPT_GETINPUT: third argument must be an Options structure!');
end

if nargin<3,
    Options=[];
end

if ~isfield(Options,'openloop')
    % If set to 1, the full optimizer as obtained as a solution to the finite-time optimal control problem
    % is returned, i.e. U = [u_0 u_1 ... u_N] where N is the prediction horizon for which the controller
    % was computed
    Options.openloop = 0;
end
if ~isfield(Options,'abs_tol') % absolute tolerance
    Options.abs_tol=mptOptions.abs_tol;
end
if ~isfield(Options,'verbose') % level of verbosity
    Options.verbose=mptOptions.verbose;
end

x0=x0(:);
inwhich = [];
fullopt = [];
runtime = [];

if isa(ctrl, 'mptctrl') & ~isexplicit(ctrl)
    % solve an QP/LP/MIQP/MILP for on-line controllers
    
    sysStruct = ctrl.sysStruct;
    probStruct = ctrl.probStruct;
    
    if ctrl.details.dims.nx~=length(x0),
        if ctrl.probStruct.tracking==1,
            disp('For tracking==1, the state vector x0 must be in form [x; u; ref] !');
        elseif ctrl.probStruct.tracking==2,
            disp('For tracking==2, the state vector x0 must be in form [x; ref] !');
        end
        error(sprintf('MPT_GETINPUT: state x0 should be a column vector with %d elements!',ctrl.details.dims.nx));
    end
    
    if iscell(sysStruct.A),
        % PWA system
        if Options.verbose<=1,
            Options.verbose=0;
        end
        if ~isfield(sysStruct.data,'MLD')
            error('MPT_GETINPUT: no MLD model available in system structure! Did you use ''mpt_sys(hysdelmodel)''?');
        end
        
        ny = sysStruct.data.MLD.ny;
        nx = sysStruct.data.MLD.nx;
        
        % define weights
        if isfield(probStruct, 'Qy')
            weights.Qy = probStruct.Qy;
        else
            weights.Qx = probStruct.Q;
        end
        if isfield(probStruct, 'Qz'),
            % penalty on "z" variables
            weights.Qz = probStruct.Qz;
        end
        if isfield(probStruct, 'Qd'),
            % penalty on "d" variables
            weights.Qd = probStruct.Qd;
        end
        weights.Qu = probStruct.R;

        % defined options
        if isfield(ctrl.details, 'Matrices'),
            % use pre-calculated problem matrices stored in the controller
            if ~isempty(ctrl.details.Matrices),
                if isfield(ctrl.details.Matrices, 'F1') & isfield(ctrl.details.Matrices, 'IntIndex'),
                    % double-check that the matrices contain appropriate fields
                    Options.problemmatrices = ctrl.details.Matrices;
                end
            end
        end
        Options.norm = probStruct.norm;
        Options.eps2 = 1e3; % tolerance on fulfillment of terminal state constraint
        Options.umin = sysStruct.umin;
        Options.umax = sysStruct.umax;
        if any(~isinf(sysStruct.dumax)) | any(~isinf(sysStruct.dumin))
            Options.dumax = sysStruct.dumax;
            Options.dumin = sysStruct.dumin;
        end
        
        Options.TerminalConstraint = 0;
        if probStruct.Tconstraint==2 & isfield(probStruct, 'Tset'),
            if isfulldim(probStruct.Tset),
                % tell mpc_mip to include terminal set constraint
                Options.Tset = probStruct.Tset;
            end
        end
        
        % define references
        if isfield(Options, 'reference') & probStruct.tracking>0,
            if isfield(probStruct, 'Qy')
                % output reference
                ref.y = Options.reference;
            else
                % state reference
                ref.x = Options.reference;
            end
        else
            if isfield(probStruct, 'Qy'),
                if isfield(probStruct, 'yref'),
                    ref.y = probStruct.yref;
                else
                    ref.y = zeros(ny,1);
                end
            else
                if isfield(probStruct, 'xref'),
                    ref.x = probStruct.xref;
                    if isfield(probStruct, 'uref')
                        ref.u = probStruct.uref;
                    end
                else
                    ref.x = zeros(nx,1);
                end
            end
        end
        
        if isfield(probStruct, 'xN'),
            clear ref
            ref.x = probStruct.xN;
            % tell mpc_mip to include terminal set constraint
            Options.TerminalConstraint = 1;
            % zero tolerance on satisfaction of terminal set constraint
            Options.eps2 = Options.abs_tol;
        end

        % in case of time-varying penalties, mpc_mip expects them to be in a
        % cell array
        %   weights{1}.Qx (.Qy, .Qu, .Qd, .Qz)
        %   ...
        %   weights{N}.Qx (.Qy, .Qu, .Qd, .Qz)
        % we make the conversion in a subfunction...
        weights = sub_fixweights(weights);
        
        % if time-varying penalties is used, mpc_mip wants to have one MLD
        % model per each sampling time, so we make it happy...
        S = {};
        for ii = 1:probStruct.N,
            S{ii} = sysStruct.data.MLD;
        end

        % the prediction horizon in mpc_mip is determined as sum(horizon)...
        horizon = repmat(1, 1, probStruct.N);
        
        [ut,dt,zt,Eflag] = mpc_mip(S, x0, ref, weights, horizon, horizon, Options);
        
        feasible = strcmp(Eflag.solverflag, 'ok');
        cost = Eflag.fopt;
        U = Eflag.u(:);
        fullopt = Eflag.full_xopt;
        runtime = Eflag.runtime;
    else
        % LTI system
        Matrices = ctrl.details.Matrices;
        [U, feasible, cost] = mpt_solveMPC(x0, sysStruct, probStruct, Matrices);
    end
    
    nu = ctrl.details.dims.nu;
    
    if ~Options.openloop & feasible,
        U = U(1:nu);
    end

    region = [];
    return
    
    
else
    ctrlStruct = ctrl;
end


% evaluate explicit control law for state x0

PA = ctrlStruct.Pn;

sysStruct = ctrlStruct.sysStruct;

if dimension(PA)~=length(x0),
    if ctrlStruct.probStruct.tracking==1,
        disp('For tracking==1, the state vector x0 must be in form [x; u; ref] !');
    elseif ctrlStruct.probStruct.tracking==2,
        disp('For tracking==2, the state vector x0 must be in form [x; ref] !');
    end
    error(sprintf('MPT_GETINPUT: state x0 should be a column vector with %d elements!',dimension(PA)));
end

%nu = ctrl.details.dims.nu;
if iscell(sysStruct.B),
    ispwa = 1;
    nu = size(sysStruct.B{1},2);
else
    ispwa = 0;
    nu = size(sysStruct.B,2);
end


U=[];
cost=-Inf;
feasible=0;
region=0;


locOpt.fastbreak = 1;

isin = isinside(ctrlStruct.Pn,x0,locOpt);

if ~isin,
    if Options.verbose > 0
        disp(sprintf('MPT_GETINPUT: NO REGION FOUND FOR STATE x = [%s]',num2str(x0')));
    end
    feasible = 0;
    return
end

if ~isfield(Options, 'fastbreak')
    Options.fastbreak = (ctrlStruct.overlaps==0);
end

if ctrlStruct.probStruct.norm~=2,
    % if mpLP was solved, some states are likely to end up on a boundary of
    % several regions, that's why we do not break prematurely in mpt_getInput
    Options.fastbreak = 0;
end

[isin, inwhich] = isinside(ctrlStruct.Pn,x0,Options);

if ~isin
    % no associated control law
    feasible=0;
    if Options.verbose>0,
        disp(sprintf('MPT_GETINPUT: NO REGION FOUND FOR STATE x = [%s]',num2str(x0')));
    end
    return
elseif isfield(ctrlStruct.details,'searchTree'),
    % use search tree for fast region identification
    % contributed by Arne Linder
    searchTree = ctrlStruct.details.searchTree;
    [lenST colST]=size(searchTree);
    node=1;
    while node>0  % node<0 means node is now number of control law
        H=searchTree(node,1:colST-3);
        K=searchTree(node,colST-2);
        if H*x0-K<0
            node=searchTree(node,colST);    % x0 on plus-side of the hyperplane
        else
            node=searchTree(node,colST-1);  % x0 on minus-side of the hyperplane
        end
    end

    node = -round(node);
    U = ctrlStruct.Fi{node}*x0+ctrlStruct.Gi{node};
    cost = x0'*ctrlStruct.Ai{node}*x0 + ctrlStruct.Bi{node}(:)'*x0 + ctrlStruct.Ci{node};
    region = node;
    feasible = 1;
    if ~Options.openloop
        % return just U at time 0
        U = U(1:nu);
    end
    return

elseif length(inwhich)==1,
    % x0 lies only in one region, return the control law and cost evaluated at x0
    U = ctrlStruct.Fi{inwhich}*x0 + ctrlStruct.Gi{inwhich};
    cost = x0'*ctrlStruct.Ai{inwhich}*x0 + x0'*ctrlStruct.Bi{inwhich}(:) + ctrlStruct.Ci{inwhich};
    feasible = 1;
    region=inwhich;
else
    % x0 belongs to several regions, go through all of them and isolate the control law with least associated cost
    mincost = Inf;      % minimal cost
    mincostregion = 0;  % index of a region in which cost associated to the state x0 is minimal
    for ii=length(inwhich):-1:1
        region = inwhich(ii);
        cost = x0'*ctrlStruct.Ai{region}*x0 + ctrlStruct.Bi{region}(:)'*x0 + ctrlStruct.Ci{region};
        if cost <= mincost,
            mincost = cost;
            mincostregion = region;
        end
    end
    U = ctrlStruct.Fi{mincostregion}*x0 + ctrlStruct.Gi{mincostregion};
    cost = mincost;
    feasible = 1;
    region = mincostregion;
end
if ctrlStruct.probStruct.feedback,
    % in case of pre-stabilization, actual control move is:
    % U = (Fi + K)*x0 + Gi
    U_nofeedback=U;
    x0_i=x0;
    for i=1:(length(U)/nu)
        U(((i-1)*nu+1):i*nu) = U(((i-1)*nu+1):i*nu) + ctrlStruct.probStruct.FBgain*x0_i;
        x0_i = ctrlStruct.sysStruct.A*x0_i + ctrlStruct.sysStruct.B*U(((i-1)*nu+1):i*nu);
    end
end

if isfield(ctrlStruct.probStruct,'inputblocking'),
    if(~isempty(ctrlStruct.probStruct.inputblocking))
        inputblocking=ctrlStruct.probStruct.inputblocking;
        u=[];
        x0_i=x0;
        for i=1:length(inputblocking)
            for j=1:inputblocking(i)
                if ~ctrlStruct.probStruct.feedback
                    u=[u; U(((i-1)*nu+1):i*nu)];    
                elseif ctrlStruct.probStruct.feedback
                    u = [u; U_nofeedback(((i-1)*nu+1):i*nu) + ctrlStruct.probStruct.FBgain*x0_i];
                    x0_i = ctrlStruct.sysStruct.A*x0_i + ctrlStruct.sysStruct.B*u(end+1-nu:end);    
                end
            end
        end
        U=u;
    end
end
if isfield(ctrlStruct.probStruct,'deltablocking'),
    if(~isempty(ctrlStruct.probStruct.deltablocking))
        deltablocking=ctrlStruct.probStruct.deltablocking;
        u=[];
        x0_i=x0;
        if ctrlStruct.probStruct.feedback
            U=U_nofeedback;
        end
        u=U(1:nu);
        for i=2:length(deltablocking)
            u(((deltablocking(i)-1)*nu+1):deltablocking(i)*nu)=U(((i-1)*nu+1):i*nu);
            for j=(deltablocking(i-1)+1):(deltablocking(i)-1)
                u(((j-1)*nu+1):j*nu)=u((((j-1)*nu+1):j*nu)-nu)+(U(((i-1)*nu+1):i*nu)-U((((i-1)*nu+1):i*nu)-nu))/(deltablocking(i)-deltablocking(i-1));
            end
        end
        if ctrlStruct.probStruct.feedback
            for i=1:(length(u)/nu)
                u(((i-1)*nu+1):i*nu) = u(((i-1)*nu+1):i*nu) + ctrlStruct.probStruct.FBgain*x0_i;
                x0_i = ctrlStruct.sysStruct.A*x0_i + ctrlStruct.sysStruct.B*u(((i-1)*nu+1):i*nu);
            end
        end
        
        U=u;
    end
end

if ~Options.openloop
    % return just U at time 0
    U = U(1:nu);
end

if nargout<5,
    clear inwhich
end
return


%---------------------------------------------------
function W = sub_fixweights(weights),
% in case of time-varying penalties, mpc_mip expects them to be in a
% cell array
%   weights{1}.Qx (.Qy, .Qu, .Qd, .Qz)
%   ...
%   weights{N}.Qx (.Qy, .Qu, .Qd, .Qz)
% we make the conversion here

Qx_cell = 0;
Qy_cell = 0;
Qu_cell = 0;
Qz_cell = 0;
Qd_cell = 0;
haveQy  = 0;
haveQz  = 0;
haveQd  = 0;
Qx_length = 0;
Qy_length = 0;
Qu_length = 0;
Qz_length = 0;
Qd_length = 0;

if isfield(weights, 'Qx'),
    if iscell(weights.Qx),
        Qx_cell = 1;
        Qx_length = length(weights.Qx);
    end
end
if isfield(weights, 'Qu'),
    if iscell(weights.Qu),
        Qu_cell = 1;
        Qu_length = length(weights.Qu);
    end
end
if isfield(weights, 'Qy'),
    haveQy = 1;
    if iscell(weights.Qy),
        Qy_cell = 1;
        Qy_length = length(weights.Qy);
    end
end
if isfield(weights, 'Qz'),
    haveQz = 1;
    if iscell(weights.Qz),
        Qz_cell = 1;
        Qz_length = length(weights.Qz);
    end
end
if isfield(weights, 'Qd'),
    haveQd = 1;
    if iscell(weights.Qd),
        Qd_cell = 1;
        Qd_length = length(weights.Qd);
    end
end

if Qx_cell | Qu_cell | Qy_cell | Qz_cell | Qd_cell,
    N = max([Qx_length Qu_length Qy_length Qz_length Qd_length]);
    W = {};
    for ii = 1:N,
        if Qx_cell,
            W{ii}.Qx = weights.Qx{ii};
        else
            W{ii}.Qx = weights.Qx;
        end
        if Qu_cell,
            W{ii}.Qu = weights.Qu{ii};
        else
            W{ii}.Qu = weights.Qu;
        end
        if haveQy,
            if Qy_cell,
                W{ii}.Qy = weights.Qy{ii};
            else
                W{ii}.Qy = weights.Qy;
            end
        end
        if haveQz,
            if Qz_cell,
                W{ii}.Qz = weights.Qz{ii};
            else
                W{ii}.Qz = weights.Qz;
            end
        end
        if haveQd,
            if Qd_cell,
                W{ii}.Qd = weights.Qd{ii};
            else
                W{ii}.Qd = weights.Qd;
            end
        end
    end
else
    W = weights;
end
