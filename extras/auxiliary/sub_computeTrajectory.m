function [X,U,Y,D,cost,trajectory,feasible,dyns,details] = sub_computeTrajectory(ctrl, x0, N, Options)
%computeTrajectory(ctrl, x0, N, Options)

% Copyright is with the following author(s):
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
if ~isstruct(mptOptions)
    mpt_error;
end

error(nargchk(2,4,nargin));

if nargin<1
    help sub_computeTrajectory
elseif nargin < 3
    N = Inf;
    Options = [];
elseif nargin < 4
    Options = [];
end

sysStruct = ctrl.sysStruct;
probStruct = ctrl.probStruct;

x0 = x0(:);
nu = ctrl.details.dims.nu;
nx = ctrl.details.dims.nx;
ny = ctrl.details.dims.ny;
nut = nu;
nxt = nx;
nyt = ny;

%=====================================================================
% set default values of Options
if ~isfield(Options, 'openloop')
    Options.openloop = 0;
end
if ~isfield(Options, 'maxCtr'),
    Options.maxCtr = 1000;
end
if ~isinf(N),
    Options.maxCtr = N;
end
if ~isfield(Options,'minnorm') 
    % consider state reached the origin if norm of the state is less then Option.minnorm and abort trajectory update
    Options.minnorm=0.01;
    userMinNorm = 0;
end
if ~isfield(Options,'samplInTset') % 
    %how many CONSECUTIVE states has to lie in the user defined terminal set
    Options.samplInTset=3;    
end
if ~isfield(Options,'stopInTset') % if user-provided terminal set should be used as a stopping critertion for state evolution
    Options.stopInTset=0;    
end
if ~isfield(Options,'reference')
    if probStruct.tracking,
        error('For tracking, please provide the reference point in Options.reference !');
    end
    Options.reference = zeros(size(probStruct.Q,1),1);
end
if ~isfield(Options,'randdist') % if random disturbances should be added to the state vector
    Options.randdist=1;    
end
if ~isfield(Options, 'verbose')
    Options.verbose = mptOptions.verbose;
end
if ~isfield(Options, 'usexinit')
    % if set to true, uses feasible solution from previous step as an initial
    % guess for the MILP/MIQP program to speed it up
    Options.usexinit = 1;
end

% this flag is need for a different call to getInput for explicit controllers
% (one more output than getInput for on-line controllers)
isEXPctrl = isexplicit(ctrl);

X = [];
U = [];
Y = [];
D = [];
dyns = [];
trajectory = [];
cost = 0;
details = {};
givedetails = (nargout==9);

if givedetails,
    Options.fastbreak = 0;
end

% do we have deltaU constraints without tracking?
if probStruct.tracking==0 & isfield(sysStruct, 'dumode') & sysStruct.dumode==1,
    isDUmode = 1;
else
    isDUmode = 0;
end

nx = ctrl.details.dims.nx;
nu = ctrl.details.dims.nu;

if length(x0)>nx,
    error('Wrong dimension of X0!');
end
u_prev = zeros(nu, 1); % default u(k-1)
if isDUmode,
    if length(x0) == nx,
        % extract u(k-1) from x0
        u_prev = x0(nx-nu+1:end); 
        
        % exclude u(k-1) from x0
        x0 = x0(1:nx-nu);
    end
end

X = [x0'];
x0_orig = x0;



%================================================================
% open-loop solution
if Options.openloop,
    x0 = extend_x0(ctrl, x0, u_prev, Options.reference, isEXPctrl, isDUmode);
    if isEXPctrl,
        if isstruct(sysStruct.A),
            % we have a PWA system, call mpt_computeTrajectory which calculates
            % open-loop trajectory for PWA system
            [X,U,Y,D,cost,trajectory,feasible,dyns] = mpt_computeTrajectory(struct(ctrl), x0, N, Options);
            return
        end
        [Uol, feasible, cost, trajectory] = mpt_getInput(ctrl, x0, Options);
    else
        [Uol, feasible, cost] = mpt_getInput(ctrl, x0, Options);
    end
    Uol = reshape(Uol, nu, [])';
    
    
    % switch from deltaU formluation if necessary
    deltaU = Uol;
    U = Uol;
    u_prev = u_prev(:)';
    if ~(iscell(sysStruct.A) & ~isEXPctrl)
        % only for explicit controllers and on-line controllers for LTI systems
        % (on-line controllers for PWA systems don't use deltaU formulation yet
        if isDUmode | probStruct.tracking==1,
            % only change U if deltaU constraints are present, or tracking with
            % deltaU formulation requested
            U = [];
            for ii=1:size(Uol,1),
                U = [U; Uol(ii,:)+u_prev];
                u_prev = U(ii,:);
            end
        end
    end

    % simulate evolution of the original system (the one which was not augmented
    % for tracking or deltaU formulation)
    [X,U,Y,dyns] = mpt_simSys(ctrl.details.origSysStruct, x0_orig, U);
    X = [x0_orig'; X];
    
    % that's it
    
else
    %================================================================
    % closed-loop solution
    Options.openloop = 0;
    
    
    finalboxtype = 0;
    % determine stoping criterion:
    %  1. state regulation towards free reference
    %  2. state regulation towards origin
    %  3. state regulation towards fixed reference
    %  4. output regulation towards free reference
    %  5. output regulation towards zero output
    %  6. output regulation towards fixed reference

    if Options.stopInTset,
        finalboxtype = 2;
        finalbox = probStruct.Tset;
        if ~isfulldim(finalbox),
            error('probStruct.Tset must be a fully dimensional set if Options.stopInTset=1 !');
        end
        
    elseif ~isfield(probStruct, 'Qy') & Options.minnorm > 0,
        %  state regulation
        
        if isfield(ctrl.sysStruct, 'dumode'),
            [nx, nu] = mpt_sysStructInfo(ctrl.sysStruct);
            nx = nx - nu;
        end
        
        if probStruct.tracking>0,
            %  1. state regulation towards free reference
            nx = ctrl.sysStruct.dims.nx;
            if nx~=length(Options.reference)
                error(sprintf('Options.reference must be a %dx1 vector, you provided a %dx1 !', ...
                    nx, length(Options.reference)));
            end
            finalbox = unitbox(nx, Options.minnorm) + Options.reference;
            finalboxtype = 1;
            
        elseif isfield(probStruct, 'xref')
            %  3. state regulation towards fixed reference
            finalbox = unitbox(nx, Options.minnorm) + probStruct.xref(1:nx);
            finalboxtype = 3;
           
        else
            %  2. state regulation towards origin
            finalbox = unitbox(nx, Options.minnorm);
            finalboxtype = 2;
        end

    elseif Options.minnorm > 0
        % output regulation
        if isfield(ctrl.sysStruct, 'dumode'),
            [nx, nu, ny] = mpt_sysStructInfo(ctrl.sysStruct);
            ny = ny - nu;
        end
        
        if probStruct.tracking>0,
            %  4. output regulation towards free reference
            ny = ctrl.sysStruct.dims.ny;
            if ny~=length(Options.reference)
                error(sprintf('Options.reference must be a %dx1 vector, you provided a %dx1 !', ...
                    ny, length(Options.reference)));
            end
            finalbox = unitbox(ny, Options.minnorm) + Options.reference;
            finalboxtype = 4;
            
        elseif isfield(probStruct, 'yref')
            %  6. output regulation towards fixed reference
            finalbox = unitbox(ny, Options.minnorm) + probStruct.yref(1:ny);
            finalboxtype = 6;
            
        else
            %  5. output regulation towards zero output
            finalbox = unitbox(ny, Options.minnorm);
            finalboxtype = 5;
            
        end

    else
        finalboxtype = 0;
    end
    
    % get the maximum value of noise (additive disturbance)
    addnoise = 0;
    if Options.randdist & mpt_isnoise(sysStruct.noise),
        % noise is added only for state regulation, otherwise we cannot
        % guarantee convergence detection
        if Options.verbose>0,
            disp('Assuming noise is hyperrectangle... trajectory is wrong if this is not true.')
        end
        if isa(sysStruct.noise, 'polytope'),
            Vnoise = extreme(sysStruct.noise);
        else
            % NOTE! remember that a V-represented noise has vertices stored
            % column-wise!
            Vnoise = sysStruct.noise';
        end
        noisedim = size(Vnoise, 2);
        deltaNoise = zeros(noisedim, 1);
        middleNoise = zeros(noisedim, 1);
        for idim = 1:noisedim,
            Vdim = Vnoise(:, idim);
            deltaNoise(idim) = (max(Vdim) - min(Vdim))/2;
            middleNoise(idim) = (max(Vdim) + min(Vdim))/2;
        end
        addnoise = 1;
        if (finalboxtype==2 | finalboxtype==3),
            if dimension(finalbox)==noisedim,
                finalbox = finalbox + sysStruct.noise;
                addnoise = 1;
            end
        end
    end
    
    
    if ~isinf(N)
        % we have user-define number of simulation steps, do not stop in finalbox
        finalbox = polytope;
        finalboxtype = 0;
    end

    
    % counter for how many consecutive sampling instances has the state/output
    % remained in a given target set
    samplesInFinalBox = 0;
    
    
    % will be set to 1 if system evolves to the target in less than
    % Options.maxCtr steps
    converged = 0;
    

    for iN = 1:Options.maxCtr
        
        %-----------------------------------------------------------------------
        % exted state vector for tracking
        x0 = extend_x0(ctrl, x0, u_prev, Options.reference, isEXPctrl, isDUmode);
        

        %-----------------------------------------------------------------------
        % record previous input (for MPC for MLD systems, such that deltaU
        % constraints are satisfied in closed-loop
        if (~isEXPctrl & iscell(sysStruct.A)),
            if iN == 1,
                Options.Uprev = u_prev;
            else
                Options.Uprev = Ucl;
            end
        end
        
        
        %-----------------------------------------------------------------------
        % obtain control input
        [Ucl, feasible, region, costCL, inwhich, fullopt] = mpt_getInput(ctrl, x0, Options);
        if Options.usexinit,
            Options.usex0 = fullopt;
        end
        if isEXPctrl
            trajectory = [trajectory region];
            if givedetails,
                details{end+1} = inwhich;
            end
        end

        
        %-----------------------------------------------------------------------
        % return if no feasible control law found
        if ~feasible
            if Options.verbose>-1,
                disp(['COMPUTETRAJECTORY: no feasible control law found for state x0=' mat2str(x0') ' !']);
            end
            cost = -Inf;
            return
        end
        cost = cost + costCL;

        
        %-----------------------------------------------------------------------
        % simulate the system for one time step, starting from x0 and applying
        % control move Ucl
        dyn = 0;
        if isEXPctrl,
            dyn = ctrl.dynamics(region);
        end
        dyn = 0;
        simSysOpt.dynamics = dyn;
        [xn, un, yn, mode] = mpt_simSys(ctrl.details.origSysStruct, x0_orig, Ucl(:)+u_prev, simSysOpt);

        
        %-----------------------------------------------------------------------
        % if noise is specified, we take some random value within of noise bounds
        if addnoise,
            noiseVal = randn(nx, 1);
            % bound noiseVal to +/- 1
            noiseVal(find(noiseVal>1)) = 1;
            noiseVal(find(noiseVal<-1)) = -1;
            % compute the noise, use safety scaling of 0.99 to be sure that we
            % do not exceed allowed limits
            noise = middleNoise + 0.99*noiseVal.*deltaNoise;
            xn = (xn(:) + noise)';
            D = [D; noise'];
        end
        
        
        %-----------------------------------------------------------------------
        % store data
        X = [X; xn(:)'];
        U = [U; un(:)'];
        Y = [Y; yn(:)'];
        dyns = [dyns; mode];
        x0 = xn(:);
        y0 = yn(:);
        x0_orig = x0;
        if probStruct.tracking==1 | isDUmode,
            if ~(~isEXPctrl & iscell(sysStruct.A))
                u_prev = un(:);
            end
        end
        
        
        %---------------------------------------------------------------------------
        % determine if system state (output) has reached it's reference
        if finalboxtype>=1 & finalboxtype<=3
            % we have state regulation, determine if state has reached finalbox
            isinfinal = isinside(finalbox, x0);
            
        elseif finalboxtype>=4
            % we have output regulation, determine if output has reached finalbox
            isinfinal = isinside(finalbox, y0);
            
        elseif finalboxtype==0
            isinfinal = 0;
        end
        
        
        %---------------------------------------------------------------------------
        % finalbox reached, increase counter
        if isinfinal,
            % we have reached finalbox, increase counter
            samplesInFinalBox = samplesInFinalBox + 1;
        else
            samplesInFinalBox = 0;
        end
        
        
        %---------------------------------------------------------------------------
        % test if state/output remained in the finalbox for a given number of
        % consecutive time instances, if so, we have converged
        if samplesInFinalBox >= Options.samplInTset
            converged = 1;
            break
        end

        
    end % for iN = 1:Options.maxCtr

    if isinf(N) & ~converged,
        if Options.verbose>-1,
            disp('COMPUTETRAJECTORY: maximum number of iterations reached!');
        end
        cost = -Inf;
    end

    if nargout > 4,
        % compute also closed-loop cost
        try
            cost = sub_computeCost(X, U, Y, sysStruct, probStruct, Options);
        catch
            % cost computation failed, please report such case to
            % mpt@control.ee.ethz.ch
            cost = -Inf;
        end
    end
    
end %end Options.openloop==0


%---------------------------------------------------------------------------
% if no noise was added, fill D with zeros
if isempty(D)
    D = zeros(size(U,1),1);
else
    D = D(1:size(U,1),:);
end

return


%==========================================================================
function x0 = extend_x0(ctrl, x0, u_prev, reference, isEXPctrl, isDUmode)
% tracking: extend state vector

if length(x0)==ctrl.details.dims.nx,
    return;
end

if ~(iscell(ctrl.sysStruct.A) & ~isEXPctrl)
    % however, we only do this for explicit controllers for PWA/LTI systems and
    % for MPC controllers for LTI systems. MPC controllers for PWA systems don't
    % augment system matrices, since tracking is handled explicitly in MLD
    % optimization routine
    
    if ctrl.probStruct.tracking>0
        nyt=ctrl.sysStruct.dims.ny;
        nut=ctrl.sysStruct.dims.nu;
        nxt=ctrl.sysStruct.dims.nx;
    elseif isDUmode, 
        % augment state vector to xn = [x; u(k-1)] if we have deltaU constraints
        x0 = [x0; u_prev(:)];
    end
    if ctrl.probStruct.tracking==1 & length(x0)<(nxt+nut+length(reference))
        x0 = [x0; u_prev(:); reference(:)];
    elseif ctrl.probStruct.tracking==2 & length(x0)<(nxt+length(reference))
        % probStruct.tracking=2 -> no deltaU formulation
        x0 = [x0; reference(:)];
    end
end

%==========================================================================
function cost = sub_computeCost(X, U, Y, sysStruct, probStruct, Options)
% computes closed-loop cost

nx = size(X, 2);
ny = size(Y, 2);
nu = size(U, 2);

if isfield(probStruct, 'Qy'),
    ycost = 1;
    Qy = probStruct.Qy;
    Qy = Qy(1:min(size(Qy, 1), ny), 1:min(size(Qy, 2), ny));
else
    ycost = 0;
end

Q = probStruct.Q;
Q = Q(1:min(size(Q, 1), nx), 1:min(size(Q, 2), nx));
R = probStruct.R;

if isfield(probStruct, 'Rdu'),
    Rdu = probStruct.Rdu;
else
    Rdu = probStruct.R;
end
dumode = 0;
if isfield(sysStruct, 'dumode'),
    dumode = sysStruct.dumode;
end
norm = probStruct.norm;

deltaU = diff(U);
if dumode | probStruct.tracking == 1,
    N = size(deltaU, 1);
else
    N = size(U, 1);
end

cost = 0;

switch probStruct.tracking
    case 0,
        % regulation problem

        if ycost,
            if isfield(probStruct, 'yref'),
                % mpt_prepareDU can extend yref, we need to crop it down to
                % original dimension
                reference = probStruct.yref(1:ny);
            else
                reference = zeros(ny, 1);
            end
        else
            if isfield(probStruct, 'xref'),
                % mpt_prepareDU can extend xref, we need to crop it down to
                % original dimension
                reference = probStruct.xref(1:nx);
            else
                reference = zeros(nx, 1);
            end
            if isfield(probStruct, 'uref'),
                uref = probStruct.uref(1:nu);
            else
                uref = zeros(nu, 1);
            end
        end
        
        if ycost 
            if dumode,
                % || Qy * (y - ref) || + || Rdu * deltaU ||
                
                for iN = 1:N,
                    cost = cost + sub_norm(Qy, Y(iN, :)' - reference, norm) + ...
                        sub_norm(Rdu, deltaU(iN, :)', norm);
                end
                
            else
                % || Qy * (y - ref) || + || R * u ||
                
                for iN = 1:N,
                    cost = cost + sub_norm(Qy, Y(iN, :)' - reference, norm) + ...
                        sub_norm(R, U(iN, :)', norm);
                end
                
            end
            
        else
            if dumode,
                % || Q * (x - ref) || + || Rdu * deltaU ||
                
                for iN = 1:N,
                    cost = cost + sub_norm(Q, X(iN, :)' - reference, norm) + ...
                        sub_norm(Rdu, deltaU(iN, :)', norm);
                end
                
            else
                % || Q * (x - ref) || + || R * (u - uref) ||
                
                for iN = 1:N,
                    cost = cost + sub_norm(Q, X(iN, :)' - reference, norm) + ...
                        sub_norm(R, U(iN, :)' - uref, norm);
                end
                
            end
        end
        
    case 1,
        % tracking with deltaU formulation
        reference = Options.reference;
        if ycost,
            % || Qy * (y - ref) || + || Rdu * deltaU ||
            
            for iN = 1:N,
                cost = cost + sub_norm(Qy, Y(iN, :)' - reference, norm) + ...
                    sub_norm(Rdu, deltaU(iN, :)', norm);
            end
            
        else
            % || Q * (x - ref) || + || Rdu * deltaU ||
        
            for iN = 1:N,
                cost = cost + sub_norm(Q, X(iN, :)' - reference, norm) + ...
                    sub_norm(Rdu, deltaU(iN, :)', norm);
            end
            
        end
        
    case 2,
        % tracking without deltaU formulation
        reference = Options.reference;
        
        if ycost,
            % || Qy * (y - ref) || + || R * u ||
            
            for iN = 1:N,
                cost = cost + sub_norm(Qy, Y(iN, :)' - reference, norm) + ...
                    sub_norm(R, U(iN, :)', norm);
            end
            
        else
            % || Q * (x - ref) || + || R * u ||
            
            for iN = 1:N,
                cost = cost + sub_norm(Q, X(iN, :)' - reference, norm) + ...
                    sub_norm(R, U(iN, :)', norm);
            end
            
        end
        
end



%==========================================================================
function result = sub_norm(Q, x, p)
% computes a weighted p-norm of a vector

x = x(:);
if p==2
    result = x'*Q*x;
else
    result = norm(Q*x, p);
end
