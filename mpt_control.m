function ctrl=mpt_control(sysStruct,probStruct,ctrltype,Options)
%MPT_CONTROL Main control routine. Computes explicit controller for a given problem
%
% ctrl=mpt_control(sysStruct,probStruct)
% ctrl=mpt_control(sysStruct,probStruct, 'on-line')
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Mother of all control functions. Main control routine of the MPT toolbox.
% Based on the problem definition it calls the appropriate multiparametric 
% controller function to compute explicit controller for both LTI
% and PWA systems. This should be the only function the user needs to call. 
%
% ---------------------------------------------------------------------------
% USAGE
% ---------------------------------------------------------------------------
%
% To compute an explicit controller, call:
%   ctrl = mpt_control(sysStruct, probStruct)
%
% To compute an on-line controller, call:
%   ctrl = mpt_control(sysStruct, probStruct, 'on-line')
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% sysStruct    - System structure in the sysStruct format
% probStruct   - Problem structure in the probStruct format
% ctrltype     - Type of controller, can be either 'explicit' or 'on-line'.
%                If ommited, assuming 'explicit'
% Options      - Optional: User options to be passed to individual control functions
% Options.autoTracking
%           If set to 0, system and problem matrices are assumed to be
%           augmented for tracking purpose by the user. By default,
%           matrices will be extended automatically to guarantee tracking.
% Options.noExtreme
%           If set to 0 (default), extreme points of each polytope in the
%           controller partition will be computed for faster plotting. Set
%           it to 1 if you don't want to compute the extreme points.
%
%	See the MPT Manual for additional details on the structure format or 
%   consult one of the example systems (e.g. Double_Integator) which were
%	provided with this package.                          
%                        
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% ctrl   - explicit controller (see 'help mptctrl' for more details)
%
% see also MPT_OPTCONTROL, MPT_OPTINFCONTROL, MPT_ITERATIVE, MPT_ITERATIVEPWA
%

% Copyright is with the following author(s):
%
%(C) 2003-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%              kvasnica@control.ee.ethz.ch

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

if ~isstruct(sysStruct)
    error('MPT_CONTROL: First input argument must be a system structure!');
end
if ~isstruct(probStruct)
    error('MPT_CONTROL: First input argument must be a problem structure!');
end

if nargin==2,
    ctrltype = 'explicit';
    Options = [];
elseif nargin==3 & ischar(ctrltype)
    Options = [];
elseif nargin==3 & (isstruct(ctrltype) | isempty(ctrltype))
    Options = ctrltype;
    ctrltype = 'explicit';
elseif nargin==3 & ~ischar(ctrltype)
    error('MPT_CONTROL: Third input argument must be a string! Allowed values are ''explicit'' and ''on-line''.');
elseif nargin==4 & ~ischar(ctrltype)
    error('MPT_CONTROL: Third input argument must be a string! Allowed values are ''explicit'' and ''on-line''.');
elseif nargin==4 & ~isstruct(Options)
    error('MPT_CONTROL: Fourth input argument must be an Options structure!');
end

if isempty(inputname(1)),
    Options.sysstructname = 'sysStruct';
else
    Options.sysstructname = inputname(1);
end
if isempty(inputname(2)),
    Options.probstructname = 'probStruct';
else
    Options.probstructname = inputname(2);
end

if ~isfield(sysStruct,'verified') | ~isfield(probStruct,'verified'),
    verOpt.verbose=1;
    verOpt.sysstructname = inputname(1);
    verOpt.probstructname = inputname(2);
    [sysStruct,probStruct]=mpt_verifySysProb(sysStruct,probStruct,verOpt);
end

if strcmpi(ctrltype, 'on-line') | strcmpi(ctrltype, 'online')
    % compute matrices of on-line controller and return
    ctrl = mptctrl(sysStruct, probStruct, Options);
    return
end

if ~isfield(Options,'noExtreme')
    Options.noExtreme=0;
end
if ~isfield(Options, 'autoTracking')
    Options.autoTracking = 1;
end
if ~isfield(Options, 'statusbar'),
    Options.statusbar = 0;
end
if ~isfield(Options, 'qpsolver'),
    Options.qpsolver = mptOptions.qpsolver;
end

if isfield(sysStruct, 'data'),
    if isfield(sysStruct.data, 'onlymld'),
        if sysStruct.data.onlymld,
            % cannot compute an explicit controller if PWA model is not
            % available
            fprintf('\nPWA representation of the hybrid system must be available in order to compute explicit solutions.\n');
            fprintf('Call "%s = mpt_sys(%s.data.MLD)" to get the PWA representation.\n\n', Options.sysstructname, Options.sysstructname);
            error('Cannot compute explicit solution if PWA representation is not available in sysStruct.');
        end
    end
end

origSysStruct = sysStruct;
origProbStruct = probStruct;

if probStruct.tracking==2 & (any(~isinf(sysStruct.dumin)) | any(~isinf(sysStruct.dumax)))
    % if we have deltaU constraints and tracking without deltaU formulation is
    % enabled, we use tracking with deltaU formulation which satisfies both
    % needs
    probStruct.tracking = 1;
end

if (probStruct.tracking & Options.autoTracking) | isfield(probStruct,'xref') | ...
        isfield(probStruct,'uref'),
    % if tracking is requested, augment system and problem matrices
    [sysStruct, probStruct] = mpt_prepareTracking(sysStruct, probStruct);
end

useDUmode = 0;
if isfield(probStruct, 'Rdu'),
    if any(probStruct.Rdu~=0),
        useDUmode = 1;
    end
end
if (any(~isinf(sysStruct.dumin)) | any(~isinf(sysStruct.dumax)))
    useDUmode = 1;
end
if useDUmode & ~probStruct.tracking,
    % augment state vector for deltaU constraints case to guarantee fullfilment
    % of those constraints in closed loop
    [sysStruct, probStruct] = mpt_prepareDU(sysStruct, probStruct);
end

if probStruct.feedback 
    if iscell(sysStruct.A),
        error('Feedback Pre-Stabilization not supported for PWA systems!');
    end
    if ~isfield(probStruct,'FBgain')
        % compute the pre-stabilization feedback for LTI system if it is not given
        [FB,S,E] = mpt_dlqr(sysStruct.A,sysStruct.B,probStruct.Q,probStruct.R);
        probStruct.FBgain = -FB;
    end
end

if length(sysStruct.Pbnd)>1 & ~iscell(sysStruct.A),
    % Pbnd is (possibly) non-convex and the system is LTI, convert it to PWA
    % format with sysStruct.guardX and sysStruct.guardC corresponding to each
    % element of sysStruct.Pbnd
    sysStruct = sub_expandPbnd(sysStruct);
end

[nx,nu,ny,ndyn,nbool,ubool] = mpt_sysStructInfo(sysStruct);

if iscell(sysStruct.A) & probStruct.norm==2 & probStruct.subopt_lev==2,
    error('2-norm problems not allowed for subopt_lev=2 and PWA systems!');
end

if nbool>0,
    if nbool==nu,
        % all inputs are boolean
        switch probStruct.subopt_lev
            case 0,
                if isinf(probStruct.N),
                    error('No infinite-time solution available for systems with discrete inputs!');
                end
                if probStruct.norm==2,
                    ctrlStruct = mpt_optQuadCtrl(sysStruct, probStruct, Options);
                else
                    ctrlStruct = mpt_optBoolCtrl(sysStruct, probStruct, Options);
                end
                
            case 1,
                ctrlStruct = mpt_boolMinTime(sysStruct, probStruct, Options);
            case 2,
                error('probStruct.subopt_lev = 2 not supported for systems with discrete inputs!');
        end
    else
        % some inputs are continuous, some integer
        switch probStruct.subopt_lev
            case 0,
                if isinf(probStruct.N),
                    error('No infinite-time solution available for systems with discrete inputs!');
                end
                if probStruct.norm==2,
                    ctrlStruct = mpt_optQuadCtrl(sysStruct, probStruct, Options);
                else
                    ctrlStruct = mpt_optMixedCtrl(sysStruct, probStruct, Options);
                end
            case 1,
                ctrlStruct = mpt_mixedMinTime(sysStruct, probStruct, Options);
            case 2,
                error('probStruct.subopt_lev = 2 not supported for systems with integer inputs!');
        end
    end
    
else
    % only continuous inputs
    
    
    if ~iscell(sysStruct.A),
        % LTI systems
        details=[];
        if probStruct.subopt_lev==0,
            % cost-optimal solution
            if probStruct.N==inf,
                % infinite time problem
                if probStruct.norm~=2,
                    % convert system to PWA
                    ctrlStruct = mpt_optInfControlPWA(sysStruct,probStruct,Options);
                else
                    ctrlStruct=mpt_optInfControl(sysStruct,probStruct,Options);
                end
            else
                % finite horizon problem
                if probStruct.norm==2,
                    ctrlStruct=mpt_optControl(sysStruct,probStruct,Options);
                else
                    if isfield(Options, 'mplpver'),
                        mplpver = Options.mplpver;
                    else
                        % choose the fastest
                        mplpver = Inf;
                    end
                    useoptcontrol = 0;
                    % solve the problem in one shot if:
                    %  - older version of the MPLP solver is requested, or
                    %  - no QP solver is available (in which case we cannot
                    %    use newer version of MPLP solver)
                    %  - system has uncertainty
                    if isfield(sysStruct, 'Aunc') | isfield(sysStruct, 'Bunc'),
                        useoptcontrol = 1;
                    elseif mplpver < 4 | Options.qpsolver==-1,
                        useoptcontrol = 1;
                    elseif isfield(sysStruct, 'noise'),
                        if mpt_isnoise(sysStruct.noise),
                            useoptcontrol = 1;
                        end
                    end
                    
                    if useoptcontrol,
                        ctrlStruct = mpt_optControl(sysStruct, probStruct, Options);
                        ctrlStruct.overlaps = 0;
                    else
                        % use DP approach with PWA cost-to-go (faster than
                        % mpt_optControl for linear objectives)
                        ctrlStruct = mpt_optControlPWA(sysStruct, probStruct, Options);
                        ctrlStruct.overlaps = 0;
                    end
                end
            end
        else
            % iterative (low-complexity) solution
            if probStruct.tracking,
                error('Tracking not supported for probStruct.subopt_lev>1 and 1/Inf norm problems');
            end

            if probStruct.subopt_lev == 1,
                [ctrlStruct]=mpt_iterative(sysStruct,probStruct,Options);
            else
                [ctrlStruct,feasible] = mpt_oneStepCtrl(sysStruct, probStruct, Options);
                if ~feasible,
                    disp('No PWQ Lyapunov function was found for this system, the final result may not be a stabilizing controller!!!');
                else
                    disp('Lyapunov function found, controller is stabilizing');
                end
            end
        end
    else
        % PWA systems
        if probStruct.subopt_lev==0,
            if probStruct.norm~=2,
                % linear performance index
                if isinf(probStruct.N),
                    % infinite-time solution
                    ctrlStruct = mpt_optInfControlPWA(sysStruct, probStruct, Options);
                else
                    % finite horizon solution
                    ctrlStruct = mpt_optControlPWA(sysStruct, probStruct, Options);
                end
            else
                % CFTOC for quadratic performance index with PWA systems
                if isinf(probStruct.N),
                    error('No infinite-time solution for PWA systems and quadratic cost.');
                end
                ctrlStruct = mpt_optQuadCtrl(sysStruct, probStruct, Options);
            end
        else
            % iterative solution
            ctrlStruct=mpt_iterativePWA(sysStruct,probStruct,Options);
        end
    end
end

if ~exist('ctrlStruct', 'var'),
    error('mpt_control: Problem is infeasible...');
end
if isempty(ctrlStruct.Pn),
    error('mpt_control: Problem is infeasible...');
end
if ~isfulldim(ctrlStruct.Pn) | length(ctrlStruct.Fi)==0,
    error('mpt_control: Problem is infeasible...');
end
if isempty(ctrlStruct.Fi{1}),
    error('mpt_control: Problem is infeasible...');
end

nR = length(ctrlStruct.Fi);
if isa(ctrlStruct, 'mptctrl'),
    ctrlStruct = struct(ctrlStruct);
end

if isfield(probStruct,'xref') | isfield(probStruct,'uref'),
    % for fixed-state tracking, do the substitution back
    Pn = [];
    xref = probStruct.xref;
    uref = probStruct.uref;
    for reg = 1:nR,
        nu = round(size(ctrlStruct.Fi{reg}, 1) / length(uref(:)));
        ctrlStruct.Gi{reg} = ctrlStruct.Gi{reg} - ...
            ctrlStruct.Fi{reg}*xref + repmat(uref,nu,1);
        if probStruct.subopt_lev==0,
            % for cost-optimal solution, translate also the cost
            ctrlStruct.Ci{reg} = ctrlStruct.Ci{reg} - ctrlStruct.Bi{reg}*xref + xref'*ctrlStruct.Ai{reg}*xref;
            ctrlStruct.Bi{reg} = ctrlStruct.Bi{reg} - 2*xref'*ctrlStruct.Ai{reg};
            % quadratic term remains identical
        end
        % translate regions:
        [H,K] = double(ctrlStruct.Pn(reg));
        Pn = [Pn polytope(H, K + H*xref)];
    end
    
    % translate feasible set:
    Pfinal = polytope;
    for fin = 1:length(ctrlStruct.Pfinal),
        if ~isfulldim(ctrlStruct.Pfinal(fin)), continue, end
        [Hf,Kf] = double(ctrlStruct.Pfinal(fin));
        Pfinal = [Pfinal polytope(Hf, Kf + Hf*xref)];
    end
    if ~isfulldim(Pfinal),
        Pfinal = Pn;
    end
    
    % store the new data
    ctrlStruct.Pfinal = Pfinal;
    ctrlStruct.Pn = Pn;
    ctrlStruct.details.origSysStruct = origSysStruct;
    ctrlStruct.details.origProbStruct = origProbStruct;
    ctrlStruct.sysStruct = sysStruct;    
    ctrlStruct.probStruct = probStruct;
end

if dimension(ctrlStruct.Pn)<=3 & length(ctrlStruct.Pn)<1000 & Options.noExtreme==0,
    % if dimension is <= 3, we compute and store also extreme points for faster plotting
    Pn = ctrlStruct.Pn;
    for ii=1:length(Pn),
        [V,R,Pn(ii)]=extreme(Pn(ii));
    end
    ctrlStruct.Pn = Pn;
end

if isfield(ctrlStruct.details,'Bi'),
    ctrlStruct.details = rmfield(ctrlStruct.details,'Bi');
end
if isfield(ctrlStruct.details,'Ci'),
    ctrlStruct.details = rmfield(ctrlStruct.details,'Ci');
end
if isfield(ctrlStruct.details,'Pn'),
    ctrlStruct.details = rmfield(ctrlStruct.details,'Pn');
end

ctrlStruct.details.origSysStruct = origSysStruct;
ctrlStruct.details.origProbStruct = origProbStruct;

fprintf('\nSolution consists of %d regions\n\n',nR);

try
    ctrl = mptctrl(ctrlStruct);
catch
    fprintf('An unexpected error occured when creating an MPT controller object\n');
    fprintf('Please be so kind and send your system and problem definition to:\n');
    fprintf('   mpt@control.ee.ethz.ch\n\n');
    fprintf('We will be happy to investigate the problem.\n');
    fprintf('Meanwhile, the controller is returned as a ctrlStruct structure...\n\n');
    ctrl = ctrlStruct;
end




%--------------------------------------------------------------------------
function sysStruct = sub_expandPbnd(sysStruct)

if length(sysStruct.Pbnd)>1 & ~iscell(sysStruct.A),
    % Pbnd is (possibly) non-convex and the system is LTI, convert it to PWA
    % format with sysStruct.guardX and sysStruct.guardC corresponding to each
    % element of sysStruct.Pbnd
    warning('sysStruct.Pbnd is a polyarray, converting LTI system to PWA form...');
    sysStruct = mpt_lti2pwa(sysStruct);
    npbnd = length(sysStruct.Pbnd);
    for ii = 2:npbnd,
        sysStruct.A{ii} = sysStruct.A{1};
        sysStruct.B{ii} = sysStruct.B{1};
        sysStruct.C{ii} = sysStruct.C{1};
        sysStruct.D{ii} = sysStruct.D{1};
        sysStruct.f{ii} = sysStruct.f{1};
        sysStruct.g{ii} = sysStruct.g{1};
        sysStruct.guardX{ii} = sysStruct.guardX{1};
        sysStruct.guardU{ii} = sysStruct.guardU{1};
        sysStruct.guardC{ii} = sysStruct.guardC{1};
    end
    for ii = 1:npbnd,
        [sysStruct.guardX{ii},sysStruct.guardC{ii}] = double(sysStruct.Pbnd(ii));
    end
    sysStruct.Pbnd = hull(sysStruct.Pbnd);
end