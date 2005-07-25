function [X,U,Y,D,cost,trajectory,feasible,dyns,reason,details]=mpt_computeTrajectory(ctrlStruct,x0,horizon,Options)
%MPT_COMPUTETRAJECTORY Calculates time evolution of state trajectories subject to control
%
% [X,U,Y,D,cost,trajectory,feasible]=mpt_computeTrajectory(ctrlStruct,x0,horizon,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% For the given state x0, the state, input and output evolution trajectories
% are computed.
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrlStruct        - Controller structure as generated by mpt_control
% x0                - initial state
% horizon           - for how many steps should the state evolution be computed
%                     If horizon=Inf, computes evolution of the state to origin
% Options.reference - If tracking is requested, provide the reference point
%                     in this variable (e.g. Options.reference = [5;0])
% Options.randdist  - If set to 1, randomly generated additive disturbance
%                       vector will be added to the state equation
% Options.openloop  - If 1, the open-loop solution will be computed, 0 for
%                       closed-loop trajectory (default is Options.openloop=0)
% Options.stopInTset- If set to 1 and user-specified terminal set was
%                     provided when computing the controller, evolution of
%                     states will be stopped as soon as all states lie in the
%                     terminal set for two consecutive steps (i.e. states will
%                     not be driven to the origin).
% Options.samplInTset - if Options.stopInTset is on, this option defines how
%                       many CONSECUTIVE states has to lie in the terminal set
%                       before the evolution terminates. Default is 2
% Options.minnorm   - If closed-loop trajectory is computed, we stope evolution
%                       if norm of a state decreases below this value
% Options.verbose   - Level of verbosity
% Options.lpsolver  - Solver for LPs (see help mpt_solveLP for details)
% Options.abs_tol   - absolute tolerance
% 
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% X, U       - matrices containing evolution of states and control moves
% Y, D       - matrices containing evolution of outputs and disturbances
% cost       - contains cost from the given initial state to the origin
% trajectory - vector of indices specifying in which region of Pn the given state lies
% feasible   - 1: the control law was feasible for all time instances, 0: otherwise
%
% see also MPT_GETINPUT, MPT_PLOTTRAJECTORY, MPT_PLOTTIMETRAJECTORY

% Copyright is with the following author(s):
%
%(C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%         kvasnica@control.ee.ethz.ch
%(C) 2003 Pascal Grieder, Automatic Control Laboratory, ETH Zurich,
%         grieder@control.ee.ethz.ch

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

error(nargchk(2,4,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<2,
    error('mpt_computeTrajectory: Wrong number of input arguments!');
end
if nargin<3 | isempty(horizon),
    horizon=Inf;     % if no horizon given, we assume infinity (evolution will be stopped if norm of the state vector drops below certain tolerance
end
if nargin<4,
    Options=[];
    if ~isa(horizon,'double'),
        error('mpt_computeTrajectory: Horizon must be an integer!');
    end
end

reason = '';
    
if ~isfield(Options,'lpsolver') % lpsolver to be used
    Options.lpsolver=mptOptions.lpsolver;
end
if ~isfield(Options,'abs_tol') % absolute tolerance
    Options.abs_tol=mptOptions.abs_tol;
end
if ~isfield(Options,'openloop') % compute the open-loop solution? (values 1/0, where 0 represents the closed-loop solution)
    Options.openloop=0;
end
if ~isfield(Options,'maxCtr') % maximum namber of iterations in the closed-loop control-law evaluation
    Options.maxCtr=200;
end
userMinNorm = 1;
if ~isfield(Options,'minnorm') % consider state reached the origin if norm of the state is less then Option.minnorm and abort trajectory update
    Options.minnorm=0.01;
    userMinNorm = 0;
end
if ~isfield(Options,'verbose') % level of verbosity
    Options.verbose=mptOptions.verbose;
end
if ~isfield(Options,'randdist') % if random disturbances should be added to the state vector
    Options.randdist=1;    
end
if ~isfield(Options,'stopInTset') % if user-provided terminal set should be used as a stopping critertion for state evolution
    Options.stopInTset=0;    
end
if ~isfield(Options,'samplInTset') % how many CONSECUTIVE states has to lie in the user defined terminal set
    Options.samplInTset=3;    
end
if ~isfield(Options,'checkOLcost')
    Options.checkOLcost=1;
end

reason = '';
details = {};
givedetails = (nargout==10);
if isa(ctrlStruct, 'mptctrl')
    ctrl = ctrlStruct;
    if ~isexplicit(ctrlStruct) & Options.verbose<=1,
        % to supress non-important output from mpc_mip
        Options.verbose = 0;
    end
    if givedetails
        [X,U,Y,D,cost,trajectory,feasible,dyns,details] = sub_computeTrajectory(ctrl, x0, horizon, Options);
    else
        [X,U,Y,D,cost,trajectory,feasible,dyns] = sub_computeTrajectory(ctrl, x0, horizon, Options);
    end
    return
end

if ~mpt_isValidCS(ctrlStruct)
    error('mpt_computeTrajectory: First argument has to be a valid controller structure! See mpt_control for details.');
end

if horizon<1,
    error('mpt_computeTrajectory: horizon MUST be a positive integer!');
end

X = [];
U = [];
Y = [];
D = [];
cost = -Inf;
trajectory = [];
feasible = 0;
dyns = [];
reason = 'error';

sysStruct = ctrlStruct.sysStruct;
probStruct = ctrlStruct.probStruct;

if iscell(sysStruct.A) & ~isfield(ctrlStruct.details,'Horizon')
    % don't check OL cost if open-loop solution is not stored in ctrlStruct
    Options.checkOLcost = 0;
end

if (~isfield(probStruct,'Qy') | isempty(probStruct.Qy) | all(all(probStruct.Qy==0)))
    ycost=0;
else
    ycost=1;
    Qy=probStruct.Qy;
end

if iscell(sysStruct.C),
    ny = size(sysStruct.C{1},1);
else
    ny = size(sysStruct.C,1);
end
if isfield(sysStruct, 'Cy')
    if iscell(sysStruct.Cy),
        ny = size(sysStruct.Cy{1}, 1);
    else
        ny = size(sysStruct.Cy, 1);
    end
end

if(isfield(probStruct,'yref'))
    yref=probStruct.yref;
else
    yref=zeros(ny,1);
    
end

opt.verbose=0;
if ~isfield(sysStruct,'verified'),
    sysStruct=mpt_verifySysStruct(sysStruct,opt);
end

if ~isfield(probStruct,'verified'),
    probStruct=mpt_verifyProbStruct(probStruct,opt);
end
    
userHorizon = 0;
if ~isinf(horizon),
    Options.minnorm=-1;           % enforce norm stopping criterion
    Options.maxCtr=horizon;
    userHorizon = 1;
end

if isfield(Options,'manualU')
    %Options.openloop=0;
    Options.maxCtr = length(Options.manualU)-1;
    Options.minnorm = -1;
end
if ~isfield(Options,'reference')
    if probStruct.tracking,
        error('For tracking, please provide the reference point in Options.reference !');
    end
    Options.reference = zeros(size(probStruct.Q,1),1);
end

if (Options.stopInTset & ~isfield(probStruct,'Tset')) | (Options.stopInTset & ~isfulldim(probStruct.Tset))
    error('mpt_computeTrajectory: User-provided terminal set is empty or non-existing, Options.Tset must be 0 in this case!');
end

if ~isfield(Options,'constrviol')
    % 1% toleration to detect constraint violation
    constrviol = 1.01;
else
    constrviol = Options.constrviol;
end

if isfield(ctrlStruct.sysStruct,'dumode'),
    % if this flag is set, solution has been computed for extended state-space
    % to guarantee fullfilment of deltaU constraints in closed-loop
    [nx,nu,ny] = mpt_sysStructInfo(ctrlStruct.sysStruct);
    if length(x0)<nx,
        % add 0 to the augmented state vector. keep in mind that for deltaU
        % constraints the state vector is augmented to include past input u(k-1)
        x0 = [x0; zeros(nu,1)];
    end
    ny = length(yref);
end

x0=x0(:);
nx=length(x0);
reason = '';

% get system description:
[A,B,C,D,Q,R,ymin,ymax,umin,umax,dumin,dumax,bndA,bndb]=mpt_evalSystem(sysStruct,probStruct);
if isfield(sysStruct, 'Cy') %& ((ycost & probStruct.tracking) | (ycost & isfield(probStruct,'yref')))
    Cy=sysStruct.Cy;
    Dy=sysStruct.Dy;
else
    Cy=sysStruct.C;
    Dy=sysStruct.D;
    sysStruct.Cy=Cy;
    sysStruct.Dy=Dy;
end

if givedetails,
    Options.fastbreak = 0;
end
X_CL{1} = x0;         %closed loop
cost_CL = 0;
cost_CL_exp = 0;
X_OL{1} = x0;         %open loop
cost_OL = 0;
nyd = ny;
if probStruct.tracking,
    if iscell(sysStruct.B),
        if isfield(sysStruct,'Dy') & ycost
            nu = size(sysStruct.B{1},2);
            nx = size(sysStruct.A{1},2)-size(sysStruct.Dy{1},1);
        else
            nu = size(sysStruct.B{1},2);
            if probStruct.tracking==1,
                nx = (size(sysStruct.A{1},2)-nu)/2;
            else
                nx = size(sysStruct.A{1},2)/2;
            end
        end
    else
        if isfield(sysStruct,'Dy') & ycost
            nu = size(sysStruct.B,2);
            nx = size(sysStruct.A,2)-size(sysStruct.Dy,1);
        else
            nu = size(sysStruct.B,2);
            if probStruct.tracking==1,
                nx = (size(sysStruct.A,2)-nu)/2;
            else
                nx = size(sysStruct.A,2)/2;
            end
        end
    end

    
    if(~ycost & length(Options.reference)<nx) | (ycost & length(Options.reference)<ny)
        if isfield(sysStruct,'dims'),
            nyd = sysStruct.dims.ny;
        else
            nyd = ny;
        end
        if ycost,
            xref = [Options.reference;zeros(nyd-length(Options.reference),1)];
        else
            xref=[Options.reference;zeros(nx-length(Options.reference),1)];
        end
    else
        xref=Options.reference;
    end
    if probStruct.tracking==1,
        X_CL{1} = [x0; zeros(nu,1); xref];
    else
        X_CL{1} = [x0; xref];
    end
    X_OL{1} = X_CL{1};
end

index=1;

PA = ctrlStruct.Pn;

% get the maximum value of noise (additive disturbance)
if isfulldim(sysStruct.noise) & Options.randdist,
    [Hnoise,Knoise]=double(sysStruct.noise);
    disp('Assuming noise is hyperrectangle... trajectory is wrong if this is not true.')
    if(length(Knoise)~=2*nx)
        error('Noise is not a hyperrectangle; it is not possible to consider noise when plotting; set Options.randdist=0')
    end
    deltaNoise=Knoise(1:length(Knoise)/2)+Knoise(length(Knoise)/2+1:end);
    maxNoise=Knoise(1:length(Knoise)/2);
else
    maxNoise=zeros(nx,1);
    deltaNoise=zeros(nx,1);
end

X = [];
U = [];
Y = [];
D = [];
dyns = [];
cost = -Inf;
trajectory = [];

feasible=1;
%+++++++++++++++++++++++++++++++++++++++
%   COMPUTE OPEN LOOP SOLUTION
%+++++++++++++++++++++++++++++++++++++++
if Options.openloop & iscell(sysStruct.A) & ~isfield(ctrlStruct.details,'Horizon')
    warning('open-loop solution is not available in ctrlStruct!');
end

if Options.openloop & ~isfield(ctrlStruct.details,'Horizon')
    Y_OL={};
    counter = 0;
    % determine number of inputs
    if iscell(sysStruct.B)
        nu=size(sysStruct.B{1},2);
    else
        nu=size(sysStruct.B,2);
    end
    
    %identify to which region x0 belongs to and extract the control input
    locOpt = Options;
    locOpt.verbose = 0;
    [U_OL,feasible,region,cost]=mpt_getInput(ctrlStruct,x0,locOpt);
    if isfield(Options,'manualU'),
        U_OL = Options.manualU;
        feasible = 1;
    end
    
    %store cost to compare with open-loop cost
    cost_getInput = cost;
    
    if ~feasible,
        disp(sprintf('mpt_computeTrajectory: NO REGION FOUND FOR STATE x = [%s]',num2str(x0')));
        return
    end  
    
    dyn = ctrlStruct.dynamics(region);
    if dyn==0,
        % no dynamics associated to this point, go through all and determine
        % active dynamics
        if ~iscell(sysStruct.A),
            % LTI system,
            dyn = 1;
        else
            for dd=1:nPWA,
                if max(sysStruct.guardX{dd}*x0+sysStruct.guardU{dd}*U_OL-sysStruct.guardC{dd})<Options.abs_tol,    
                    % check which dynamics is active in the region
                    dyn = dd;
                    break
                end
            end
        end
    end
    
    if iscell(sysStruct.A),
        A=sysStruct.A{dyn};
        B=sysStruct.B{dyn};
        f=sysStruct.f{dyn};
        C=sysStruct.C{dyn};
        D=sysStruct.D{dyn};
        g=sysStruct.g{dyn};
        Dy=sysStruct.Dy{dyn};
        Cy=sysStruct.Cy{dyn};
        
    else
        A=sysStruct.A;
        B=sysStruct.B;
        C=sysStruct.C;
        D=sysStruct.D;
        if isfield(sysStruct,'f'),
            f = sysStruct.f;
        else
            f=zeros(size(A,1),1);
        end
        if isfield(sysStruct,'g'),
            g = sysStruct.g;
        else
            g=zeros(size(C,1),1);
        end
    end
    dyns = [dyns; dyn];
    
    nu = size(B,2);
    
    U=[];%returned U
    for i=1:(length(U_OL)/nu)
        jj=((i-1)*nu+1):(i*nu);   %in case system has more than one input
        
        % check fulfillment of input constraints
        if any(U_OL(jj)>constrviol*(umax+(constrviol-1)/100)) | any(U_OL(jj)<constrviol*(umin-(constrviol-1)/100))
            disp(sprintf('mpt_computeTrajectory: INPUT CONSTRAINTS EXCEEDED FOR STATE x = [%s]',num2str(x0')))
            disp(num2str(U_OL))
        end
        U=[U;U_OL(jj)'];
        
        % state update:
        X_OL{i+1}=A*X_OL{i}+B*U_OL(jj)+f;
        
        % output function:
        Y_OL{i}=C*X_OL{i}+D*U_OL(jj)+g;
        
        if i>1,
            % check fulfillment of output constraints
            % if any(max(Y_OL{i})>constrviol*(ymax+(constrviol-1)/100)) | any(min(Y_OL{i})<constrviol*(ymin-(constrviol-1)/100))
            if any(Y_OL{i}>constrviol*(ymax+(constrviol-1)/100)) | any(Y_OL{i}<constrviol*(ymin-(constrviol-1)/100))
                disp(sprintf('mpt_computeTrajectory: OUTPUT CONSTRAINTS EXCEEDED FOR STATE x = [%s]',num2str(x0')))
                disp(num2str(Y_OL{i}(:)'))
            end
        end
        if probStruct.norm==2,
            if(ycost)
                cost_OL = cost_OL + (Y_OL{i}(1:ny)-yref)'*Qy*(Y_OL{i}(1:ny)-yref)+U_OL(jj)'*R*U_OL(jj);
            else
                cost_OL = cost_OL + X_OL{i}'*Q*X_OL{i}+U_OL(jj)'*R*U_OL(jj);
            end
            
        else
            if ycost,
                cost_OL = cost_OL + norm(Qy*(Y_OL{i}-yref),probStruct.norm) + ...
                    norm(R*U_OL(jj), probStruct.norm);
            else
                cost_OL = cost_OL + norm(Q*X_OL{i},probStruct.norm) + norm(R*U_OL(jj),probStruct.norm);
            end
        end
    end
    
    if isfield(probStruct,'P_N'),
        PLQR = probStruct.P_N;
    elseif(probStruct.norm==2 & ~ycost)
        [KLQ,PLQR,E] = mpt_dlqr(A,B,Q,R);
    elseif(ycost)
        PLQR=Qy;
    else
        PLQR=Q;
    end
    cost = cost_OL;
    if probStruct.norm==2,
        if(ycost)
            cost = cost_OL + (X_OL{end}(1:ny)-yref)'*PLQR*(X_OL{end}(1:ny)-yref);
            %cost = cost_OL; % + (Y_OL{end}-yref)'*probStruct.P_N*(Y_OL{end}-yref);
        else
            cost = cost_OL + X_OL{length(X_OL)}'*PLQR*X_OL{length(X_OL)};
        end
    else
        if ~ycost,
            cost=cost_OL + norm(PLQR*X_OL{length(X_OL)},probStruct.norm); % add cost from x_N to infinity
        end
    end
    
    % store trajectories in column vectors
    X=zeros(length(X_OL),size(X_OL{1},1));
    Y=zeros(length(Y_OL),size(Y_OL{1},1));
    for ii=1:length(X_OL),
        X(ii,:)=X_OL{ii}';
    end
    for ii=1:length(Y_OL),
        Y(ii,:)=Y_OL{ii}';
    end
    D=zeros(size(Y));
    trajectory=0;
end %if openloop
   
%% open-loop solution for PWA systems
if Options.openloop & isfield(ctrlStruct.details,'Horizon')
    HorizonSteps = ctrlStruct.details.Horizon;
    N = length(HorizonSteps);
    Y=[];
    X=x0';
    U=[];
    Dist=[];
    D=[];
    trajectory=[];
    counter = 0;
    cost_getInput = 0;

    locOpt = Options;
    locOpt.verbose = 0;
        
    [U_OL,feasible,region,cost]=mpt_getInput(ctrlStruct,x0,locOpt);
    cost_getInput = cost_getInput+cost;
    
    for ii = N:-1:1,
        %% identify to which region x0 belongs to and extract the control input
        x0 = X(end,:)';
        [U_OL,feasible,region,cost]=mpt_getInput(ctrlStruct.details.Horizon{ii},x0,locOpt);
        if ~feasible,
            D = Dist;
            disp(sprintf('mpt_computeTrajectory: NO REGION FOUND FOR STATE x = [%s]',num2str(x0')));
            return
        end  
        
        dyn = ctrlStruct.details.Horizon{ii}.dynamics(region);
        if dyn==0,
            % no dynamics associated to this point, go through all and determine
            % active dynamics
            if ~iscell(sysStruct.A),
                % LTI system,
                dyn = 1;
            else
                for dd=1:nPWA,
                    if max(sysStruct.guardX{dd}*x0+sysStruct.guardU{dd}*U_OL-sysStruct.guardC{dd})<Options.abs_tol,    
                        % check which dynamics is active in the region
                        dyn = dd;
                        break
                    end
                end
            end
        end
        
        if iscell(sysStruct.A),
            A=sysStruct.A{dyn};
            B=sysStruct.B{dyn};
            f=sysStruct.f{dyn};
            C=sysStruct.C{dyn};
            D=sysStruct.D{dyn};
            g=sysStruct.g{dyn};
            Dy=sysStruct.Dy{dyn};
            Cy=sysStruct.Cy{dyn};
        else
            A=sysStruct.A;
            B=sysStruct.B;
            C=sysStruct.C;
            D=sysStruct.D;
            f=zeros(size(A,1),1);
            g=zeros(size(C,1),1);
        end
        
        dyns = [dyns; dyn];
        nu = size(B,2);
        
        % check fulfillment of input constraints
        if any(U_OL>constrviol*(umax+(constrviol-1)/100)) | any(U_OL<constrviol*(umin-(constrviol-1)/100))
            disp(sprintf('mpt_computeTrajectory: INPUT CONSTRAINTS EXCEEDED FOR STATE x = [%s]',num2str(x0')))
            disp(num2str(U_OL))
        end

        %% update cost
        if ycost,
            YOLi = C*x0 + D*U_OL + g;
            if isfield(probStruct, 'yref'),
                cost_OL = cost_OL + norm(probStruct.Qy*(YOLi-probStruct.yref),probStruct.norm) + norm(R*U_OL,probStruct.norm);
            else
                cost_OL = cost_OL + norm(probStruct.Qy*(YOLi),probStruct.norm) + norm(R*U_OL,probStruct.norm);
            end
        else
            cost_OL = cost_OL + norm(Q*x0,probStruct.norm) + norm(R*U_OL,probStruct.norm);
        end
        
        % state update equation:
        X_OL = A*x0+B*U_OL+f;
        x0 = X_OL;
        Y_OL = C*x0 + D*U_OL + g;
        Y = [Y; Y_OL'];
        X = [X; X_OL'];
        U = [U; U_OL'];
        trajectory = [trajectory; region];
        Dist = [Dist; zeros(1,length(X_OL))];
        D = Dist;
        
        %% check fulfillment of output constraints
        if any(Y_OL>constrviol*(ymax+(constrviol-1)/100)) | any(Y_OL<constrviol*(ymin-(constrviol-1)/100))
            disp(sprintf('mpt_computeTrajectory: OUTPUT CONSTRAINTS EXCEEDED FOR STATE x = [%s]',num2str(x0')))
            disp(num2str(Y_OL))
        end
        
    end
    
    if isfield(probStruct,'P_N'),
        PLQR = probStruct.P_N;
    elseif(probStruct.norm==2)
        [KLQ,PLQR,E] = mpt_dlqr(A,B,Q,R);
    else
        PLQR=Q;
    end
    if probStruct.norm==2,
        if(ycost)
            cost = cost_OL + (Cy*X_OL'+Dy*U_OL-yref)*Qy*(Cy*X_OL'+Dy*U_OL-yref);
        else
            cost = cost_OL + X_OL'*PLQR*X_OL;
        end
    else
        if ~ycost,
            cost=cost_OL + norm(PLQR*X_OL,probStruct.norm); % add cost from x_N to infinity
        else
            cost=cost_OL;
        end
    end
    
end

if iscell(sysStruct.A),
    nPWA = length(sysStruct.A);
else
    nPWA = 0;
end

%+++++++++++++++++++++++++++++++++++++++
%   COMPUTE CLOSED LOOP SOLUTION
%+++++++++++++++++++++++++++++++++++++++
if probStruct.tracking,
    if isfield(sysStruct, 'dims')
        nx = sysStruct.dims.nx;
        ny = sysStruct.dims.ny;
        nu = sysStruct.dims.nu;
    else
        if iscell(sysStruct.B),
            if isfield(sysStruct,'Dy') & ycost
                nu = size(sysStruct.B{1},2);
                nx = size(sysStruct.A{1},2)-size(sysStruct.Dy{1},1);
            elseif ycost
                nu = size(sysStruct.B{1},2);
                nx = size(sysStruct.A{1},2) - nu - (size(sysStruct.C{1},1)-nu)/2;
            else
                nu = size(sysStruct.B{1},2);
                nx = (size(sysStruct.A{1},2)-nu)/2;
            end
        else
            if isfield(sysStruct,'Dy') & ycost
                nu = size(sysStruct.B,2);
                nx = size(sysStruct.A,2)-size(sysStruct.Dy,1);
            elseif ycost,
                nu = size(sysStruct.B,2);
                nx = size(sysStruct.A,2) - nu - (size(sysStruct.C,1)-nu)/2;
            else
                nu = size(sysStruct.B,2);
                nx = (size(sysStruct.A,2)-nu)/2;
            end
        end
    end
else
    [nx,nu,nyprev] = mpt_sysStructInfo(sysStruct);
end

if isfield(probStruct,'yref')
    ny = length(probStruct.yref);
end
uprev = zeros(nu,1);

if ~userMinNorm & ~userHorizon,
    yd = zeros(nyd,1);
    for iy = 1:nyd,
        yd(iy) = (ymax(iy) - ymin(iy))/2;
    end
    ydmin = min(yd);
    if isinf(ydmin),
        ydmin = 1;
    end
    Options.minnorm = ydmin*0.01;
end

if(~Options.openloop)
    i=1;
    U_CL=[];
    Y_CL={};
    DIST=[];
    locOpt = Options;
    locOpt.verbose = 0;


    % if iterative solution was computed, regions are already order in such a way,
    % that the first region found is in fact the one with least step distance to the origin,
    % thus we can break in mpt_getInput as soon as at least one region is found
    % (results in much faster run-time!
    locOpt.fastbreak = (probStruct.subopt_lev>0);

    finalbox = polytope;
    finalbox_track = polytope;
    % repeat evolution until norm of state is higher that certain value or maximum number iterations is reached
    if Options.stopInTset
        finalbox = probStruct.Tset;
    else
        finalbox = unitbox(dimension(PA),Options.minnorm)+sysStruct.noise;
    end
    if probStruct.tracking & isinf(horizon)
        if ycost,
            finalbox_track = unitbox(nyd,Options.minnorm)+sysStruct.noise;
            [Hf,Kf] = double(finalbox_track);
            finalbox_track = polytope(Hf, Kf+Hf*xref(1:nyd));
        else
            finalbox_track = unitbox(nx,Options.minnorm)+sysStruct.noise;
            [Hf,Kf] = double(finalbox_track);
            finalbox_track = polytope(Hf, Kf+Hf*xref(1:nx));
        end
    end
    if isfield(probStruct,'xref') & isinf(horizon),
        [Hf,Kf] = double(finalbox);
        finalbox = polytope(Hf, Kf + Hf*probStruct.xref);
    end
    boxtype = 1;
    if isfield(probStruct,'Qy') & isinf(horizon) & ~probStruct.tracking,
        finalbox_y = unitbox(ny,Options.minnorm);
        if isfield(probStruct,'yref'),
            finalbox_y = finalbox_y + probStruct.yref;
        end
        boxtype = 2;
    end
    boxctr=0;

    while (boxctr<Options.samplInTset & i<=Options.maxCtr+1)
        reason = 'not converged';
        if boxtype==1 & isfulldim(finalbox),
            if(isinside(finalbox, X_CL{i}))
                boxctr=boxctr+1;
                if isfield(probStruct,'xref'),
                    reason = 'converged to fixed state reference';
                else
                    if Options.stopInTset,
                        reason = 'target set reached';
                    else
                        reason = 'converged to origin';
                    end
                end
            else
                boxctr=0;
            end
        elseif boxtype==2 & isfulldim(finalbox_y)
            if i>1,
                if ycost,
                    Ycheck = Y_CLy{end};
                else
                    Ycheck = Y_CL{end}(1:ny);
                end
                if isinside(finalbox_y, Ycheck),
                    boxctr = boxctr + 1;
                    reason = 'converged to fixed output reference';
                else
                    boxctr = 0;
                end
            else
                boxctr = 0;
            end
        end
        if boxctr>=Options.samplInTset,
            break
        end
        if probStruct.tracking,
            if (ycost & i>1 & isinside(finalbox_track, Y_CL{i-1}(1:nyd))) | ...
                    (~ycost & isinside(finalbox_track, X_CL{i}(1:nx))),
                reason = 'converged to free state reference';
                break
            end
        end

        %identify to which region x0 belongs to and extract the control input
        if isfield(Options,'manualU'),
            U = Options.manualU(i,:)';
            feasible = 1;
            region = 0;
            cost = 0;
            dynamics = 0;
            if isfield(ctrlStruct.sysStruct,'guardX')
                for ii=1:length(ctrlStruct.sysStruct.guardX),
                    if all(ctrlStruct.sysStruct.guardX{ii}*X_CL{i} + ctrlStruct.sysStruct.guardU{ii}*U <= ctrlStruct.sysStruct.guardC{ii}),
                        dynamics = ii;
                        break
                    end
                end
            else
                dynamics = 1;
            end
            regions = find(ctrlStruct.dynamics == dynamics);
            if isempty(regions),
                error('mpt_computeTrajectory: no associated dynamics found!');
            end
            region = regions(1);
        else
            if givedetails,
                [U,feasible,region,cost,inwhich]=mpt_getInput(ctrlStruct,X_CL{i},locOpt);
                if givedetails,
                    details{end+1} = inwhich;
                end
            else
                [U,feasible,region,cost]=mpt_getInput(ctrlStruct,X_CL{i},locOpt);
            end
            cost_CL_exp = cost_CL_exp + cost;
        end

        if ~feasible,
            % if no region found, the controller does not guarantee feasibility for all time, thus we abort
            if Options.verbose >= 0
                disp(sprintf('mpt_computeTrajectory: NO REGION FOUND FOR STATE x = [%s]',num2str(X_CL{i}')));
                if boxctr==1,
                    disp('State trajectory entered user defined target set, but no associated control law exists.');
                elseif boxctr>1,
                    disp('State trajectory went through user defined target set but didn''nt stop there.');
                end
            end
            break
        end

        % pick up dynamics active for given x0 and optimal U
        dyn = ctrlStruct.dynamics(region);
        if dyn==0,
            % no dynamics associated to this point, go through all and determine
            % active dynamics
            if ~iscell(sysStruct.A),
                % LTI system,
                dyn = 1;
            else
                for dd=1:nPWA,
                    if max(sysStruct.guardX{dd}*X_CL{i}+sysStruct.guardU{dd}*U-sysStruct.guardC{dd})<Options.abs_tol,
                        % check which dynamics is active in the region
                        dyn = dd;
                        break
                    end
                end
            end
        end

        if iscell(sysStruct.A),
            A=sysStruct.A{dyn};
            B=sysStruct.B{dyn};
            f=sysStruct.f{dyn};
            C=sysStruct.C{dyn};
            D=sysStruct.D{dyn};
            g=sysStruct.g{dyn};
            if isfield(sysStruct, 'Cy'),
                Cy=sysStruct.Cy{dyn};
                Dy=sysStruct.Dy{dyn};
            else
                Cy = C;
                Dy = D;
            end
        else
            A=sysStruct.A;
            B=sysStruct.B;
            C=sysStruct.C;
            D=sysStruct.D;
            if isfield(sysStruct,'f'),
                f = sysStruct.f;
            else
                f=zeros(size(A,1),1);
            end
            if isfield(sysStruct,'g'),
                g = sysStruct.g;
            else
                g=zeros(size(C,1),1);
            end
            if isfield(sysStruct, 'Cy'),
                Cy = sysStruct.Cy;
                Dy = sysStruct.Dy;
                gy = g(1:size(Cy,1));
            else
                Cy = C;
                Dy = D;
            end
        end

        trajectory(i)=region;
        dyns = [dyns; dyn];
        U_CL=[U_CL; U'];
        deltaU = U - uprev;
        uprev = U;

        noise=maxNoise-rand(nx,1).*deltaNoise; % if noise is specified, we take some random value within of noise bounds
        DIST=[DIST; noise'];
        if probStruct.tracking==1,
            if ycost,
                noise = [noise; zeros(nu+nyd,1)];
            else
                noise = [noise; zeros(nu+size(noise,1),1)];
            end
        elseif length(noise)<length(f)
            if ycost
                noise = [noise; zeros(nyd,1)];
            else
                noise = [noise; zeros(nx,1)];
            end
        end

        % state update function
        X_CL{i+1} = A*X_CL{i} + B*U(1:nu) + f + noise;
        % output equation
        if ycost,
            % use Cy and Dy if they are given
            if ~exist('gy','var')
                gy = g(1:size(Cy,1));
            end
            Y_CLy{i} = Cy*X_CL{i} + Dy*U(1:nu) + gy;
        end
        Y_CL{i} = C*X_CL{i} + D*U(1:nu) + g;
        
        % check fulfillment of input constraints
        % if any(max(U(1:nu))>constrviol*(umax+(constrviol-1)/100)) | any(min(U(1:nu))<constrviol*(umin-(constrviol-1)/100))
        if any(U(1:nu)>constrviol*(umax+(constrviol-1)/100)) | any(U(1:nu)<constrviol*(umin-(constrviol-1)/100))
            disp(sprintf('mpt_computeTrajectory: INPUT CONSTRAINTS EXCEEDED FOR STATE x = [%s]',num2str(X_CL{i}')))
            disp(num2str(U'))
        end

        % check fulfillment of output constraints
        if i>1,
            if probStruct.tracking,
                if any(Y_CL{i}(1:nx)>constrviol*(ymax(1:nx)+(constrviol-1)/100)) | ...
                        any(Y_CL{i}(1:nx)<constrviol*(ymin(1:nx)-(constrviol-1)/100))
                    disp(sprintf('mpt_computeTrajectory: OUTPUT CONSTRAINTS EXCEEDED FOR STATE x = [%s]',num2str(X_CL{i}')))
                    disp(num2str(Y_CL{i}'))
                end
            else
                % if any(max(Y_CL{i})>constrviol*(ymax+(constrviol-1)/100)) | any(min(Y_CL{i})<constrviol*(ymin-(constrviol-1)/100))
                if any(Y_CL{i}>constrviol*(ymax+(constrviol-1)/100)) | any(Y_CL{i}<constrviol*(ymin-(constrviol-1)/100))
                    disp(sprintf('mpt_computeTrajectory: OUTPUT CONSTRAINTS EXCEEDED FOR STATE x = [%s]',num2str(X_CL{i}')))
                    disp(num2str(Y_CL{i}'))
                end
            end
        end

        % update the closed-loop cost
        if probStruct.norm==2,
            if(ycost)
                %%%cost_CL = cost_CL + (Cy*X_CL{i}+Dy*U(1:nu)-yref)'*Qy*(Cy*X_CL{i}+Dy*U(1:nu)-yref) + U(1:nu)'*R*U(1:nu);
                cost_CL = cost_CL;
            else
                cost_CL = cost_CL + X_CL{i}'*Q*X_CL{i} + U(1:nu)'*R*U(1:nu);
            end
        else
            if ycost,
                % FIXME
            else
                cost_CL = cost_CL + norm(Q*X_CL{i},probStruct.norm) + norm(R*U(1:nu),probStruct.norm);
            end
        end
        if isfield(probStruct,'Rdu'),
            cost_CL = cost_CL + norm(probStruct.Rdu*deltaU,probStruct.norm);
        end
        i=i+1;
    end
    if i>=Options.maxCtr & isinf(horizon) & ~isfield(Options,'manualU')
        disp('mpt_computeTrajectory: Maximum number of steps reached!')
    end

    %compute infinite horizon cost, i.e. cost from final state X_N to infinity

    if feasible
        if isfield(probStruct,'P_N'),
            Q = probStruct.P_N;
        end
        if iscell(Cy),
            dyn = ctrlStruct.dynamics(region);
            Cy=sysStruct.Cy{dyn};
            Dy=sysStruct.Dy{dyn};
        end
        if probStruct.norm==2,
            if(ycost & ~probStruct.tracking)
                cost = cost_CL + X_CL{end}(1:nx)'*Cy'*Qy*Cy*X_CL{end}(1:nx); %not possible to compute final cost for ouput tracking => computing dummy
            elseif ycost & probStruct.tracking==1
                cost = cost_CL + X_CL{end}'*Cy'*Qy*Cy*X_CL{end}; %not possible to compute final cost for ouput tracking => computing dummy
            elseif ycost & probStruct.tracking==2
                % FIXME
                cost = cost_CL;
            else
                cost = cost_CL + X_CL{end}'*Q*X_CL{end};
            end
        else
            if(ycost)
                cost = cost_CL + norm(Qy*Cy*X_CL{end},probStruct.norm);
            else
                cost = cost_CL + norm(Q*X_CL{end},probStruct.norm);
            end
        end
    end

    % store trajectories to matrices|
    X=zeros(length(X_CL),size(X_CL{1},1));
    if length(Y_CL)>0,
        if ycost,
            Y=zeros(length(Y_CLy),size(Y_CLy{1},1));
        else
            Y=zeros(length(Y_CL),size(Y_CL{1},1));
        end
    else
        Y=[];
    end
    for ii=1:length(X_CL),
        X(ii,:)=X_CL{ii}';
    end
    U=U_CL;
    if ycost & exist('Y_CLy', 'var'),
        Y_CL = Y_CLy;
    end
    for ii=1:length(Y_CL),
        Y(ii,:)=Y_CL{ii}';
    end
    D=DIST;
end %if closedloop
cost = cost_CL_exp;
if ctrlStruct.probStruct.tracking | isfield(ctrlStruct.sysStruct,'dumode')
    % crop down X and Y matrices to return only "true" states
    % change U which currently contains deltaU
    if isfield(ctrlStruct.sysStruct,'dumode'),
        nx = nx - nu;
    end
    X = X(:,1:nx);
    if ctrlStruct.probStruct.tracking,
        if ycost,
            Y = Y(:,1:nyd);
            for dind = 1:size(Y,1),
                Y(dind,:) = Y(dind,:) + xref(:)';
            end
            %Y = Y(:,1:nyd)+xref';
        else
            if probStruct.tracking==1,
                Y = Y(:,1:size(Y,2)-nx-nu);
            else
                Y = Y(:,1:size(Y,2)-nx);
            end
        end
    else
        % ycost case
        % Y is already of proper dimension
        if size(Y,2)-nu > 0,
            Y = Y(:,1:size(Y,2)-nu);
        end
    end
    if probStruct.tracking~=2,
        deltaU = U;
        if Options.openloop,
            U = X_OL{1}(nx+1:nx+nu)';
        else
            U = X_CL{1}(nx+1:nx+nu)';
        end
        for ii=1:size(deltaU,1),
            U = [U; U(ii,:)+deltaU(ii,:)];
        end
        U = U(2:end,:);
    end
end

if(Options.openloop) & ~isfield(Options,'manualU') & Options.checkOLcost,
    if abs(cost_getInput)>Options.abs_tol | abs(cost)>Options.abs_tol,
        if(abs((cost_getInput-cost)/(cost_getInput+cost))>Options.abs_tol)
            [cost cost_getInput]
            disp('Value function mismatch! You may be experiencing numerical difficulties.')
            disp('Please contact the MPT support team at mpt@control.ee.ethz.ch')
            disp('and include your system and problem definition');
            fprintf('\n')
            fprintf('Difference: %f\n',cost_getInput-cost)
        end
    end
end