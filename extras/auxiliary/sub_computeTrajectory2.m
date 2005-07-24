function [X,U,Y,D,cost,trajectory,feasible,dyns,details] = sub_computeTrajectory(ctrl, x0, N, Options)
%sub_computeTrajectory(ctrl, x0, N, Options)

% $Id: sub_computeTrajectory2.m,v 1.2 2005/02/28 19:27:35 kvasnica Exp $
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
    Options.samplInTset=2;    
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

x0_orig = x0;

%=====================================================================
% tracking: extend state vector
if ~(iscell(sysStruct.A) & ~isEXPctrl)
    % however, we only do this for explicit controllers for PWA/LTI systems and
    % for MPC controllers for LTI systems. MPC controllers for PWA systems don't
    % augment system matrices, since tracking is handled explicitly in MLD
    % optimization routine
    
    if probStruct.tracking>0
        nyt=ctrl.sysStruct.dims.ny;
        nut=ctrl.sysStruct.dims.nu;
        nxt=ctrl.sysStruct.dims.nx;
    elseif isDUmode, 
        % augment state vector to xn = [x; u(k-1)] if we have deltaU constraints
        x0 = [x0; zeros(nu,1)];
    end
    if probStruct.tracking==1 & length(x0)<(length(x0)+nut+length(Options.reference))
        x0 = [x0; zeros(nut,1); Options.reference(:)];
    elseif probStruct.tracking==2 & length(x0)<(length(x0)+length(Options.reference))
        % probStruct.tracking=2 -> no deltaU formulation
        x0 = [x0; Options.reference(:)];
    end
end
X = [x0'];


%================================================================
% open-loop solution
if Options.openloop,
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
    Uol = reshape(Uol, nu, length(Uol)/nu)';
    [X,U,Y,dyns] = mpt_simSys(sysStruct, x0, Uol);
    X = [x0'; X];
    % that's all
    
    
    
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
    
    if ~isfield(probStruct, 'Qy'),
        %  state regulation
        
        if probStruct.tracking>0,
            %  1. state regulation towards free reference
            nx = ctrl.sysStruct.dims.nx;
            finalbox = unitbox(nx, Options.minnorm) + Options.reference;
            finalboxtype = 1;
            
        elseif ~isfield(probStruct, 'xref')
            %  1. state regulation towards origin
            finalbox = unitbox(nx, Options.minnorm);
            finalboxtype = 2;
            
        else
            %  2. state regulation towards fixed reference
            finalbox = unitbox(nx, Options.minnorm) + probStruct.xref;
            finalboxtype = 3;
        end
        
    else
        % output regulation
        
        if probStruct.tracking>0,
            %  4. output regulation towards free reference
            ny = ctrl.sysStruct.dims.ny;
            finalbox = unitbox(ny, Options.minnorm) + Options.reference;
            finalboxtype = 4;
            
        elseif ~isfield(probStruct, 'yref') | all(probStruct.yref==0)
            %  3. output regulation towards zero output
            finalbox = unitbox(ny, Options.minnorm);
            finalboxtype = 5;
            
        else
            %  4. output regulation towards fixed reference
            finalbox = unitbox(ny, Options.minnorm) + probStruct.yref;
            finalboxtype = 6;
        end
    end
    
    % get the maximum value of noise (additive disturbance)
    addnoise = 0;
    if (finalboxtype==2 | finalboxtype==3) & Options.randdist & isfulldim(sysStruct.noise),
        % noise is added only for state regulation, otherwise we cannot
        % guarantee convergence detection
        [Hnoise,Knoise]=double(sysStruct.noise);
        if Options.verbose>0,
            disp('Assuming noise is hyperrectangle... trajectory is wrong if this is not true.')
        end
        deltaNoise=Knoise(1:length(Knoise)/2)+Knoise(length(Knoise)/2+1:end);
        maxNoise=Knoise(1:length(Knoise)/2);
        if dimension(finalbox)==dimension(sysStruct.noise),
            finalbox = finalbox + sysStruct.noise;
            addnoise = 1;
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
    
    u_prev = [];
    
    for iN = 1:Options.maxCtr
        
        % obtain control input
        if isEXPctrl
            [Ucl, feasible, region, costCL, inwhich] = mpt_getInput(ctrl, x0, Options);
            trajectory = [trajectory region];
            if givedetails,
                details{end+1} = inwhich;
            end
        else
            [Ucl, feasible, costCL] = mpt_getInput(ctrl, x0, Options);
        end
        if ~feasible
            if Options.verbose>-1,
                disp(['COMPUTETRAJECTORY: no feasible control law found for state x0=' mat2str(x0') ' !']);
            end
            return
        end
        cost = cost + costCL;

        
        % simulate the system for one time step, starting from x0 and applying
        % control move Ucl
        dyn = 0;
        if isEXPctrl,
            dyn = ctrl.dynamics(region);
        end
        dyn = 0;
        simSysOpt.dynamics = dyn;
        [xn, un, yn, mode] = mpt_simSys(sysStruct, x0, Ucl, simSysOpt);
        
        if addnoise,
            % if noise is specified, we take some random value within of noise
            % bounds
            noise=maxNoise-rand(nx,1).*deltaNoise;
            xn = (xn(:) + noise)';
            D = [D; noise'];
        end
        
        X = [X; xn];
        U = [U; un];
        Y = [Y; yn];
        dyns = [dyns; mode];
        
        x0 = xn(:);
        y0 = yn(:);
        
        %---------------------------------------------------------------------------
        % determine if system state (output) has reached it's reference
        if finalboxtype>=1 & finalboxtype<=3
            % we have state regulation, determine if state has reached finalbox
            if probStruct.tracking,
                xf = x0(1:nxt);
            else
                xf = x0;
            end
            isinfinal = isinside(finalbox, xf);
            
        elseif finalboxtype>=4
            % we have output regulation, determine if output has reached finalbox
            if probStruct.tracking,
                yf = y0(1:nyt);
            else
                yf = y0;
            end
            isinfinal = isinside(finalbox, yf);
            
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
    end
    
end %end Options.openloop==0


%---------------------------------------------------------------------------
% if no noise was added, fill D with zeros
if isempty(D)
    D = zeros(size(U,1),1);
else
    D = D(1:size(U,1),:);
end


%---------------------------------------------------------------------------
% probStruct.tracking=1 introduces a deltaU formulation and the state vector
% is of the form xn = [x; u(k-1); xref]
% we therefore:
%  1. switch deltaU to U
%  2. return only the "true states" in X
%  3. return only the "true outputs" in Y
if ~(iscell(sysStruct.A) & ~isEXPctrl)
    % however, we only do this for explicit controllers for PWA/LTI systems and
    % for MPC controllers for LTI systems. MPC controllers for PWA systems don't
    % augment system matrices, since tracking is handled explicitly in MLD
    % optimization routine
    
    if probStruct.tracking==1 | isDUmode
        deltaU = U;
        if probStruct.tracking==1,
            U = X(1,nxt+1:nxt+nut)';
        else
            U = X(1,nx-nu+1:end)';
        end
        for ii=1:size(deltaU,1),
            U = [U; U(ii,:)+deltaU(ii,:)];
        end
        U = U(2:end,:);
    end
    if probStruct.tracking,
        X = X(:, 1:nxt);
        Y = Y(:, 1:nyt);
    elseif isDUmode,
        X = X(:, 1:nx-nu);
        Y = Y(:, 1:ny-nu);
    end
end