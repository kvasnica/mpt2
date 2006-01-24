function varargout = mpt_owncost(sysStruct, probStruct, F, obj, vars)
% MPT_OWNCOST The "Design your own cost" function
%
% mpt_owncost(sysStruct, probStruct)
% [C, O, V] = mpt_owncost(sysStruct, probStruct)
% ctrl = mpt_owncost(CON, OBJ, VAR)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% "Design Your Own Cost". This function operates in two modes:
% 
% 1. Problem construction phase:
%      In this step, matrices defining constraints and objective of a given MPC
%      problem are formulated. The matrices, together with variables which
%      define them, are stored to your workspace as CON (constraints), OBJ
%      (objective) and VAR (variables) objects.
% 2. Computation phase:
%      In this step, a control law is calculated according to constraints and
%      objective provided.
%
% Example:
%   We would like to impose polytopic constraints on all predicted states, but
%   not on the initial condition x0:
%
%     [H, K] = double(unitbox(2, 2));   % polytopic constraints
%     Double_Integrator
%     mpt_owncost(sysStruct, probStruct);
%     for k = 2:length(VAR.x)
%       % k==1 corresponds to x0, k==2 to x1 etc.
%       CON = CON + set(H*VAR.x{k} <= K);
%     end
%     ctrl = mpt_owncost(sysStruct, probStruct, CON, OBJ, VAR)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% sysStruct     - System structure in the sysStruct format
% probStruct    - Problem structure in the probStruct format
% CON           - Constraints
% OBJ           - Objective
% VAR           - Variables
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% ctrl          - MPTCTRL object 
% C             - Constraints (optional)
% O             - Objective (optional)
% V             - Variables (optional)
%
% see also MPT_CONTROL, MPT_YALMIPCFTOC
%

% Copyright is with the following author(s):
%
% (C) 2006 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
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

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end
Options = [];
Options = mpt_defaultOptions(Options, ...
    'verbose', mptOptions.verbose, ...
    'dont_solve', 1, ...
    'dp', 0);

if ~(nargin==2 | nargin==5),
    error('Wrong number of input arguments.');
end

ctrl = [];
if nargin==2,
    % generate constraints, objective and variables

    if ~(iscell(sysStruct) | isstruct(sysStruct)),
        error('First input must be a structure.');
    end
    if ~isstruct(probStruct),
        error('Second input must be a structure.');
    end
    
    try
        [dummy, C, O, V] = mpt_yalmipcftoc(sysStruct, probStruct, Options);
    catch
        error(lasterr);
    end
    
    if nargout==0,
        % assign varibales in caller's workspace
        
        % check for existence of certain variables in caller's workspace
        checkvars = {'CON', 'OBJ', 'VAR'};
        for ic = 1:length(checkvars),
            evalcmd = sprintf('exist(''%s'', ''var'')', checkvars{ic});
            exists = evalin('caller', evalcmd);
            if exists,
                fprintf('Variable "%s" was overwritten...\n', checkvars{ic});
            end
        end
        
        assignin('caller', 'CON', C);
        assignin('caller', 'OBJ', O);
        assignin('caller', 'VAR', V);
        fprintf('Variables CON, OBJ and VAR are now stored in your workspace.\n');
        
    elseif nargout==3,
        % return constraints, objective and variables
        varargout{1} = C;
        varargout{2} = O;
        varargout{3} = V;
        
    else
        error('Wrong number of output arguments.');
        
    end
    
else
    % compute controller based on custom constraints/objective
    
    if ~(isa(F, 'lmi') | isa(F, 'set')),
        error('First input must be a set of constraints.');
    end
    if ~isa(obj, 'sdpvar'),
        error('Second input must be an optimization objective.');
    end
    if ~isstruct(vars),
        error('Third input must be a structure.');
    end
    if ~isfield(vars, 'x') | ~isfield(vars, 'u') | ~isfield(vars, 'y'),
        error('Wrong type of third input argument.');
    end
    
    if iscell(sysStruct),
        error('Multi-model systems not supported by this function.');
    end

    % verify sysStruct and probStruct
    verOpt.ybounds_optional = 1;
    verOpt.verbose = -1;    
    verOpt.useyalmip = 1; % to tell mpt_verifyProbStruct we can deal with move blocking
    [sysStruct, probStruct] = mpt_verifySysProb(sysStruct, probStruct, verOpt);
    origSysStruct = sysStruct;
    origProbStruct = probStruct;
    if probStruct.tracking > 0 & ~isfield(probStruct, 'tracking_augmented')
        [sysStruct, probStruct] = mpt_yalmipTracking(sysStruct, probStruct, verOpt);
    end
    
    
    % compute the controller
    yalmipOptions = mptOptions.sdpsettings;
    yalmipOptions.debug = 1;
    yalmipOptions.verbose = 1;
    starttime = cputime;
    [sol, diagnost, Uz] = solvemp(F, obj, yalmipOptions, vars.x{1}, [vars.u{1:end-1}]);
    if length(sol)==1,
        ctrl = sol{1};
        overlaps = 0;
        
    else
        if isempty(sol),
            fprintf('\n\n');
            fprintf('========================================================================\n');
            fprintf('Please send your system and problem definition to mpt@control.ee.ethz.ch\n');
            fprintf('========================================================================\n\n');
            error('mpt_owncost: an error has occurred, see message above.');
        end
        
        if probStruct.norm==2,
            ctrl = mpt_mergeCS(sol);
            overlaps = 1;
        else
            ctrl = mpt_removeOverlaps(sol);
            overlaps = 0;
        end
    end

    if isempty(ctrl),
        error('Problem infeasible or an error occurred!');
    end

    ctrl.details.runTime = cputime - starttime;
    if probStruct.norm~=2,
        nx = length(ctrl.Bi{1});
        for ii = 1:length(ctrl.Ai),
            ctrl.Ai{ii} = zeros(nx);
        end
    end

    ctrl.overlaps = overlaps;
    ctrl.details.origSysStruct = origSysStruct;
    ctrl.details.origProbStruct = origProbStruct;
    ctrl.sysStruct = sysStruct;
    ctrl.probStruct = probStruct;
    
    try
        ctrl = mptctrl(ctrl);
    catch
        fprintf('Error while creating controller object, result returned as a structure\n');
    end

    varargout{1} = ctrl;
    
end
