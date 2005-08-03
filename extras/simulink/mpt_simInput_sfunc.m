function [sys,x0,str,ts] = mpt_simInput_sfunc(t,x,u,flag,ctrl,Ts,infbreak)
%MPT_SIMINPUT_SFUNC S-function to evaluate a control law
%
% mpt_simInput_sfunc(t,x,u,flag,sysStruct,X0,Ts)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% For the given state x0, this function extracts the optimal output from a controller
% given by means of the controller structure ctrlStruct. If the controller partition
% is overlapping in the X space, the input U will be picked up such that an associated
% cost is minimized (i.e. if for x0 there are 2 or more associated control laws,
% only the one which minimizes a given criterion is returned. The criterion is
% either value of the objective function for optimimal solution or minimum time
% for the time-optimal solution).
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% t           - time
% x           - state
% u           - input
% flag        - S-function flag
% sysStruct   - system structure
% X0          - initial condition
% Ts          - sampling time
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% sys         - state update
% x0          - initial condition
% ts          - sampling time
%
% see also MPT_GETINPUT
%

% Copyright is with the following author(s):
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

persistent Uprev

dumode = isfield(ctrl.sysStruct, 'dumode');
isPWA = iscell(ctrl.sysStruct.A);
tracking = ctrl.probStruct.tracking;
ycost = isfield(ctrl.probStruct, 'Qy');

dimref = 0;

if ctrl.probStruct.tracking>0
    nx = ctrl.sysStruct.dims.nx;
    nu = ctrl.sysStruct.dims.nu;
    ny = ctrl.sysStruct.dims.ny;
else
    nx = ctrl.details.dims.nx;
    nu = ctrl.details.dims.nu;
    ny = ctrl.details.dims.ny;
end

if tracking>0,
    if ycost
        dimref = ny;
    else
        dimref = nx;
    end
    
elseif dumode
    nx = nx - nu;
        
end

dumode = dumode | tracking==1;

switch flag,
    
    %%%%%%%%%%%%%%%%%%
    % Initialization %
    %%%%%%%%%%%%%%%%%%
    case 0,
        [sys,x0,str,ts,Uprev] = mdlInitializeSizes(ctrl, Ts, nu, nx, dimref);
        
        %%%%%%%%%%
        % Update %
        %%%%%%%%%%
    case 2,
        sys = mdlUpdate(t,x,u,ctrl, nu, nx, dimref, dumode, tracking, isPWA);
        
        %%%%%%%%%%
        % Output %
        %%%%%%%%%%
    case 3,
        [sys, Uprev] = mdlOutputs(t,x,u,ctrl, nu, nx, dimref, dumode, tracking, isPWA, Uprev, infbreak);
        
        %%%%%%%%%%%%%
        % Terminate %
        %%%%%%%%%%%%%
    case 9,
        sys = []; % do nothing
        
        %%%%%%%%%%%%%%%%%%%%
        % Unexpected flags %
        %%%%%%%%%%%%%%%%%%%%
    otherwise
        error(['unhandled flag = ',num2str(flag)]);
end

%end dsfunc

%
%=======================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=======================================================================
%
function [sys,x0,str,ts,Uprev] = mdlInitializeSizes(ctrl, Ts, nu, nx, dimref)

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = nu + 1;  % control move + feasibility flag
if dimref==0,
    dimref = 1;
end
sizes.NumInputs      = nx+dimref;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

x0 = zeros(sizes.NumDiscStates, 1);
str = [];
ts  = [Ts 0]; 

Uprev = zeros(nu, 1);

% end mdlInitializeSizes

%
%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=======================================================================
%
function sys = mdlUpdate(t,x,u,ctrl, nu, nx, dimref, dumode, tracking, isPWA)

sys = [];

%
%=======================================================================
% mdlOutputs
% Return the output vector for the S-function
%=======================================================================
%
function [sys, Uprev] = mdlOutputs(t,x,u,ctrl, nu, nx, dimref, dumode, tracking, isPWA, Uprev, infbreak)

if isempty(Uprev),
    Uprev = zeros(nu, 1);
end

xm = u(1:nx);
ref = u(nx+1:nx+dimref);

isMLD = ~isexplicit(ctrl) & iscell(ctrl.sysStruct.A);

if isMLD,
    % MPC for MLD systems does NOT work with extended state vector. we just need
    % to provide previous input in Options.Uprev, in order to satisfy deltaU
    % constraints in closed-loop.
    x0 = xm;
    opt.Uprev = Uprev;
else
    % augment state vector to include previous input and/or reference
    if tracking==0 & dumode,
        x0 = [xm; Uprev];
    elseif tracking==2,
        x0 = [xm; ref];
    elseif tracking==1,
        x0 = [xm; Uprev; ref];
    else
        x0 = xm;
    end
end

opt.verbose = -1;
[U,feasible,region,cost]=mpt_getInput(ctrl,x0, opt);
if isempty(U) | ~feasible,
    if infbreak,
        error(sprintf('No feasible control law found for state %s', mat2str(x0(:)')));
        U = 0;
        set_param(gcs, 'SimulationCommand', 'stop');
        return
    else
        % use previous control action if no feasible control law was found
        U = Uprev;
        sys = [U; feasible];
        return
    end
end
U = U(:);

% for tracking problems, U returned by the controller is deltaU, we need to
% convert it back
if dumode
    if ~isPWA,
        % however we only do it if the controller is not an on-line controller
        % for PWA systems
        U = U + Uprev;
    end
end
Uprev = U;
sys = [U; feasible];


%end mdlOutputs

