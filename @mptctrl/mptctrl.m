function ctrl = mptctrl(varargin)
%MPTCTRL Constructor for the MPT controller object
%
% ctrl = mptctrl(ctrlStruct)
% ctrl = mptctrl(sysStruct, probStruct)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
%
% Creates an MPT controller object. The controller can represent both an
% explicit control law (i.e. a Piecewise-Affine feedback law defined over a
% polyhedral partition of a state-space), or an implicit controller (i.e. an MPC
% controller where the optimization problem is being solved at every time step
% for current measurements of the states).
%
% USAGE:
%
%   ctrl = mptctrl(ctrlStruct)
%
%     If called with one input parameter, the input is assumed to be a
%     controller structure, as defined in the MPT manual. The output will then
%     be an explicit controller.
%
%   ctrl = mptctrl(sysStruct, probStruct)
%
%     If called with two input parameters, creates an on-line controller.
%
% ---------------------------------------------------------------------------
% REPRESENTATION OF CONTROLLERS
% ---------------------------------------------------------------------------
%
% Each MPTCTRL object is internally represented as a structure with following
% fields:
%
%   .type        - type of the controller {'explicit' | 'online'}
%   .sysStruct   - system structure
%   .probStruct  - problem structure
%   .Pn          - polyhedral partition over which the explicit controller is
%                  defined (POLYTOPE object)
%   .Pfinal      - feasible set of the controller, i.e. set of states for which
%                  there exists a control law (POLYTOPE object) 
%
%   .Fi          - if the given state "x" is in region Pn(i), the associated
%   .Gi          /    control law is given by U = Fi{i}*x + Gi{i}
%
%   .Ai          \
%   .Bi          - value of the objective function at point 'x' is given by
%   .Ci          /    J = x'*Ai{i}*x + Bi{i}*x + Ci{i}
%
%   .dynamics    - a vector which associates dynamics of a PWA system to a given
%                     region of controller parition. 
%   .details     - additional details about the solution
%   .overlaps    - binary flag, takes a "true" value if regions of the
%                     controller partition overlap, "false" otherwise
%   .simplified  - binary flag, takes a "true" value if regions of the
%                     controller partition have been simplified using greedy or
%                     optimal merging, "false" otherwise 
%
%
% ---------------------------------------------------------------------------
% ACCESING INTERNAL DATA OF A CONTROLLER OBJECT
% ---------------------------------------------------------------------------
%
% Even though the MPT controller is represented as an object, it's fields can
% still be accessed as it were a structure. That means that you can do, for
% instance, the following:
%
%   ctrl.Pn
%   ctrl.details
%
% and so on. There are also couple of short-cut functions defined to quickly
% access important fields:
%
% nR = length(ctrl)    - returns number of regions of the controller partition
% time = runtime(ctrl) - returns runtime
% isexplicit(ctrl)     - returns 1 if the controller is an explicit one, 0 if it
%                          is an on-line controller
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% either: 
%   ctrlStruct  - valid controller structure (see manual for more details)
%
% or:
%   sysStruct   - valid system structure
%   probStruct  - valid problem structure
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% ctrl          - an MPTCTRL object
%
%
% see also MPTCTRL/ANALYZE, MPTCTRL/ISEXPLICIT, MPTCTRL/LENGTH, MPTCTRL/PLOT
%

% $Id: mptctrl.m,v 1.12 2005/06/23 14:27:28 kvasnica Exp $
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

if nargin > 0,
    % if input is an MPTCTRL object, do nothing
    if isa(varargin{1}, 'mptctrl')
        ctrl = varargin{1};
        return
    end
end

% =========================================================================
% only these fields are allowed to be present in a ctrlStruct structure
allowedfields = {'sysStruct', 'probStruct', 'Pn', 'Pfinal', 'Fi', 'Gi', 'Ai', 'Bi', ...
        'Ci', 'dynamics', 'details', 'overlaps', 'simplified', 'type'};

% =========================================================================
% set default values
ctrl.sysStruct = [];
ctrl.probStruct = [];
ctrl.Pn = polytope;
ctrl.Pfinal = polytope;
ctrl.Fi = {};
ctrl.Gi = {};
ctrl.Ai = {};
ctrl.Bi = {};
ctrl.Ci = {};
ctrl.dynamics = [];
ctrl.details = [];
ctrl.overlaps = 0;
ctrl.simplified = 0;
ctrl.type = [];

if nargin==0,
    ctrl = class(ctrl, 'mptctrl');
    return
end

% =========================================================================
% first input has always to be a structure
if ~isstruct(varargin{1})
    error('MPTCTRL: first input must be a structure!');
end


if isfield(varargin{1}, 'Pn') & isfield(varargin{1}, 'Fi')
    
    % =========================================================================
    % this structure is a controller structure
    
    ctrlStruct = varargin{1};
    
    if mpt_isValidCS(ctrlStruct, struct('nowarnings',1)),
        % copy only allowed fields from ctrlStruct to ctrl, any remaining fields
        % will be stored to ctrl.details.additional
        fnames = fields(ctrlStruct);
        for ii=1:length(fnames)
            fval = getfield(ctrlStruct, fnames{ii});
            ctrl = setfield(ctrl, fnames{ii}, fval);
        end
        ctrl = copyvalidfields(ctrl, allowedfields);
        
    else
        error('MPTCTRL: first input is not a valid controller structure!');
    end
    
    ctrl.type = 'explicit';
    
elseif nargin==2 | nargin==3
    
    % =========================================================================
    % two inputs, possibly sysStruct + probStruct case, verify that
    sysStruct = varargin{1};
    probStruct = varargin{2};
    if ~isstruct(sysStruct) | ~isstruct(probStruct)
        error('MPTCTRL: both input arguments must be structures!');
    end

    % =========================================================================
    % verify sysStruct and probStruct, catch any errors
    try
        [sysStruct, probStruct] = mpt_verifySysProb(sysStruct, probStruct);
    catch
        rethrow(lasterror);
    end
    
    ctrl.sysStruct = sysStruct;
    ctrl.probStruct = probStruct;
    ispwa = iscell(sysStruct.A);
    
    % =========================================================================
    % check if on-line solution can be computed
    if isfulldim(sysStruct.noise) & iscell(sysStruct.A)
        error('MPTCTRL: no on-line solution for PWA systems with additive noise!');
    elseif isfield(sysStruct, 'Aunc') & iscell(sysStruct.A)
        error('MPTCTRL: no on-line solution for PWA systems with parametric uncertainty!');
    elseif probStruct.subopt_lev==1,
        error('MPTCTRL: no on-line solution to minimum-time problems! Please set ''probStruct.subopt_lev=0''.');
    elseif probStruct.subopt_lev==2,
        error('MPTCTRL: no on-line solution to low-complexity problems! Please set ''probStruct.subopt_lev=0''.');
    elseif isinf(probStruct.N)
        error('MPTCTRL: no on-line solution to infinite-horizon problems! Please set ''probStruct.N=finite''.');
    end
    
    ctrl.details.origSysStruct = sysStruct;
    ctrl.details.origProbStruct = probStruct;
    
    % =========================================================================
    % some additional tests for conditions under which an on-line controller
    % cannot be computed
    if probStruct.feedback 
        if iscell(sysStruct.A),
            error('MPTCTRL: Feedback Pre-Stabilization not supported for PWA systems!');
        end
        if ~isfield(probStruct,'FBgain')
            % compute the pre-stabilization feedback for LTI system if it is not given
            [FB,S,E] = mpt_dlqr(sysStruct.A,sysStruct.B,probStruct.Q,probStruct.R);
            probStruct.FBgain = -FB;
        end
    end

    
    
    if iscell(sysStruct.A)
    
        % =========================================================================
        % we have a PWA system, check if MLD model is available in sysStruct
        
        haveMLD = 0;
        if isfield(sysStruct, 'data')
            if isfield(sysStruct.data, 'MLD')
                haveMLD = 1;
            end
        end
        if ~haveMLD
            % =========================================================================
            % no MLD model available, obtain it by doing a PWA2MLD conversion
            disp('======================================================================');
            disp('WARNING: No MLD model available in system structure! Trying to convert');
            disp('         PWA model to MLD representation. This conversion can be very');
            disp('         inefficient! You should always opt for HYSDEL model instead.');
            disp('======================================================================');
            fprintf('\n'); 
            fprintf('Converting PWA system into MLD representation...\n');
            if ~isfield(sysStruct, 'xmax') | ~isfield(sysStruct, 'xmin')
                % xmax and xmin must be defined for pwa2mld translation
                error('State constraints must be defined!');
            end
            
            % do the conversion...
            S = mpt_pwa2mld(sysStruct);
            
            % store MLD matrices to sysStruct...
            sysStruct.data.MLD = S;
            ctrl.sysStruct.data.MLD = S;
        end
        % =========================================================================
        % add state and output constraints as defined in
        % sysStruct.{xmax|xmin|ymax|ymin}
        ctrl.sysStruct.data.MLD = addMLDconstraints(ctrl.sysStruct.data.MLD, sysStruct);
        if any(~isinf(sysStruct.dumax)) | any(~isinf(sysStruct.dumin))
            fprintf('\nWARNING: deltaU constraints will only be satisfied in open-loop solutions!\n\n');
        end
        [nx,nu,ny] = mpt_sysStructInfo(sysStruct);
        sysStruct.dims = struct('nx', nx, 'nu', nu, 'ny', ny);
        
    else
        % =========================================================================
        % we have an LTI system, construct matrices

        % first augment matrices for tracking if necessary
        if probStruct.tracking==2 & (any(~isinf(sysStruct.dumin)) | any(~isinf(sysStruct.dumax)))
            % if we have deltaU constraints and tracking without deltaU formulation is
            % enabled, we use tracking with deltaU formulation which satisfies both
            % needs
            probStruct.tracking = 1;
        end
        
        if (probStruct.tracking | isfield(probStruct,'xref') | isfield(probStruct,'uref')) & ...
                ~iscell(sysStruct.A)
            % if tracking is requested, augment system and problem matrices
            [sysStruct, probStruct] = mpt_prepareTracking(sysStruct, probStruct);
        end
        
        if (any(~isinf(sysStruct.dumin)) | any(~isinf(sysStruct.dumax))) & ~probStruct.tracking,
            % augment state vector for deltaU constraints case to guarantee fullfilment
            % of those constraints in closed loop
            [sysStruct, probStruct] = mpt_prepareDU(sysStruct, probStruct);
        end

        if nargin==3,
            Options = varargin{end};
        else
            Options = [];
        end
        if ~isfield(Options, 'noConstraintReduction'),
            Options.noConstraintReduction = 1;
        end
        if (isfield(probStruct,'inputblocking') | isfield(probStruct,'deltablocking'))
            opt = Options;
            opt.noConstraintReduction = 1;
            % do not reduce constraints yet, it will be done (if requested) in
            % mpt_blockingMatrices
            ctrl.details.Matrices = mpt_constructMatrices(sysStruct, probStruct, opt);
            ctrl.details.Matrices = mpt_blockingMatrices(ctrl.details.Matrices,sysStruct,probStruct, Options);
        else
            ctrl.details.Matrices = mpt_constructMatrices(sysStruct, probStruct, Options);
        end

    end
    ctrl.sysStruct = sysStruct;
    ctrl.probStruct = probStruct;

    ctrl.type = 'online';
else

    error('MPTCTRL: unknown type of input argument(s)!');
end


% =========================================================================
% store dimensions
[nx,nu,ny] = mpt_sysStructInfo(ctrl.sysStruct);
ctrl.details.dims = struct('nx', nx, 'nu', nu, 'ny', ny);


% =========================================================================
% create the object
ctrl = class(ctrl, 'mptctrl');