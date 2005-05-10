function [simpleCtrl,details] = mpt_simplify(ctrl, how, Options)
%MPT_SIMPLIFY simplifies a given explicit controller by merging regions with identical control law
%
%   simplectrl = mpt_simplify(ctrl)
%   simplectrl = mpt_simplify(ctrl, how)
%   simplectrl = mpt_simplify(ctrl, Options)
%   simplectrl = mpt_simplify(ctrl, how, Options)
%   [simplectrl, details] = mpt_simplify(ctrl, how, Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Simplifies a given explicit controller by merging regions which have the
% same control law. By doing so, all stability and feasibility properties
% of the controller are maintained, but complexity is greatly reduced.
% Information about the cost function is lost.
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl             - Explicit controller (an MPTCTRL object)
% how              - which method to use for merging. allowed values are:
%                      'greedy'  - greedy merging based on heuristics 
%                                  (default setting) 
%                      'optimal' - optimal merging based on boolean minimization
% Options.trials   - for greedy merging, defines number of trials to
%                    improve the solution (default is 1, corresponds to 1 run)
% Options.verbose  - level of verbosity {0|1|2}
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% simpleCtrl       - simplified explicit controller
% details.before   - number of polytopes before merging
% details.after    - number of polytopes after merging
% details.runTime  - run time of the algorithm
% details.alg      - string, either 'greedy' or 'optimal'
%
% see also MERGE
%

% $Id: mpt_simplify.m,v 1.8 2005/05/10 20:17:58 kvasnica Exp $
%
% (C) 2004-2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%               kvasnica@control.ee.ethz.ch

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

error(nargchk(1,3,nargin));

if isa(ctrl, 'mptctrl')
    if ~isexplicit(ctrl)
        disp('MPT_SIMPLIFY: No simplification can be performed for on-line controllers.');
        simpleCtrl = ctrl;
        details = [];
        return
    end
end

if nargin==1,
    how = 'greedy';
    Options = [];
elseif nargin==2,
    if isstruct(how)
        Options = how;
        how = 'greedy';
    elseif ischar(how)
        Options = [];
    else
        error('MPT_SIMPLIFY: Second argument must be either a string or an Options structure!');
    end
else
    if ~ischar(how)
        error('MPT_SIMPLIFY: Second argument must be a string! Allowed values are ''greedy'' and ''optimal''.');
    end
    if ~isstruct(Options)
        error('MPT_SIMPLIFY: Third argument must be an Options structure!');
    end
end

how = lower(how);
if ~(strcmp(how, 'greedy') | strcmp(how, 'optimal')),
    error('MPT_SIMPLIFY: Unknown simplification method! Allowed values are ''greedy'' and ''optimal''.');
end

if ~isfield(Options, 'merging')
    Options.merging = how;
end
if ~isfield(Options, 'verbose')
    Options.verbose = 0;
end

if ~isfield(Options, 'statusbar'),
    Options.statusbar = 0;
end

if ~isfield(Options,'trials')
    % number of attempts to improve solution of greedy merging
    Options.trials = 1;
end
if strcmpi(Options.merging,'optimal')
    Options.greedy = 0;
else
    Options.greedy = 1;
end

if isa(ctrl, 'mptctrl')
    ctrlStruct = struct(ctrl);
else
    ctrlStruct = ctrl;
end

if ~mpt_isValidCS(ctrlStruct),
    error('mpt_simplify: input argument must be a valid controller structure!');
end

if ctrlStruct.overlaps %& Options.greedy==1,
    disp('Overlaps detected, removing overlaps...');
    ctrlStruct = mpt_removeOverlaps(ctrlStruct);
end

[nx,nu,ny,ndyn] = mpt_sysStructInfo(ctrlStruct.sysStruct);
if size(ctrlStruct.Fi{1},1)>nu,
    % if the controller was obtained by solving a CFTOC problem for LTI
    % systems, extract only control law assigned to the first step.
    % after doing so, information about the open-loop trajectory is lost.
    nR = length(ctrlStruct.Fi);
    newFi = cell(1, nR);
    newGi = cell(1, nR);
    for ir = 1:nR
        newFi{ir} = ctrlStruct.Fi{ir}(1:nu,:);
        newGi{ir} = ctrlStruct.Gi{ir}(1:nu);
    end
    ctrlStruct.Fi = newFi;
    ctrlStruct.Gi = newGi;
end

% isloate regions which have the same value of control law
sameregions = sub_preparecolors(ctrlStruct);

newPn = mptOptions.emptypoly;

if Options.statusbar,
    Options.verbose = -1;
end

if Options.verbose > -1,
    fprintf('%d regions with %d different control laws\n',...
        length(ctrlStruct.Pn), length(sameregions.Table));
end

simpleCtrlStruct = ctrlStruct;
simpleCtrlStruct.Pn = mptOptions.emptypoly;
simpleCtrlStruct.Fi = {};
simpleCtrlStruct.Gi = {};
simpleCtrlStruct.Ai = {};
simpleCtrlStruct.Bi = {};
simpleCtrlStruct.Ci = {};
simpleCtrlStruct.dynamics = [];
simpleCtrlStruct.simplified = 1;

zAi = zeros(size(ctrlStruct.Ai{1}));
zBi = zeros(size(ctrlStruct.Bi{1}));
zCi = zeros(size(ctrlStruct.Ci{1}));
simpl_time = 0;

% added for [issue91]
wstatus = warning;
warning off

Options.closestatbar = 0;

if Options.greedy,
    if Options.statusbar,
        Options.status_handle = mpt_statusbar('Computing...');
    end
    
    for it = 1:length(sameregions.Table),
        
        if Options.statusbar,
            prog_min = (it - 1) / length(sameregions.Table);
            prog_max = it / length(sameregions.Table);
            Options.status_min = prog_min;
            Options.status_max = prog_max;
            if isempty(mpt_statusbar(Options.status_handle, 0, prog_min, prog_max)),
                mpt_statusbar;
                error('Break...');
            end
        end
        
        % first extract regions with identical control law
        Pn = ctrlStruct.Pn(sameregions.Table{it});

        lenPn = length(Pn);
        if lenPn==1,
            regstr = 'region';
        else
            regstr = 'regions';
        end
        if Options.verbose==0,
            fprintf('control law # %d, %d %s --> ',it,lenPn,regstr);
        end

        % try to merge regions
        [Pm,mdetails] = merge(Pn, Options);
        simpl_time = simpl_time + mdetails.runTime;

        lenPm = length(Pm);
        if lenPm==1,
            regstr = 'region';
        else
            regstr = 'regions';
        end
        if Options.verbose==0,
            fprintf('%d %s\n',lenPm,regstr);
        end

        % write data back to new controller
        simpleCtrlStruct.Pn = [simpleCtrlStruct.Pn Pm];
        for ir = 1:length(Pm),
            simpleCtrlStruct.Fi{end+1} = sameregions.Fi{it};
            simpleCtrlStruct.Gi{end+1} = sameregions.Gi{it};
        end
    end
    if Options.statusbar,
        if isempty(mpt_statusbar(Options.status_handle, 1, prog_min, prog_max)),
            mpt_statusbar;
            error('Break...');
        end
    end

else
    % optimal merging - call mpt_optMerge
    startt = cputime;
    Options.verbose = 1;
    Options.algo = 0;
    Options.color = sameregions.Reg;
    Options.Table = sameregions.Table;
    if length(ctrlStruct.Pfinal)==1,
        % feasible set is convex
        Options.PAdom = ctrlStruct.Pfinal;
        Options.PAcompl = mptOptions.emptypoly;
    else
        % feasible set is (presumably) non-convex
        Options.PAdom = hull(ctrlStruct.Pfinal);
        Options.PAcompl = Options.PAdom \ ctrlStruct.Pfinal;
    end
    [PAmer, colorMer] = mpt_optMerge(ctrlStruct.Pn, Options);
    simpleCtrlStruct.Pn = PAmer;
    for ii=1:length(PAmer),
        ind = colorMer.Reg(ii);
        simpleCtrlStruct.Fi{end+1} = sameregions.Fi{ind};
        simpleCtrlStruct.Gi{end+1} = sameregions.Gi{ind};
    end
    simpl_time = cputime - startt;
end

% added for [issue91]
warning(wstatus);

nRnew = length(simpleCtrlStruct.Fi);
simpleCtrlStruct.Ai = cell(1, nRnew);
simpleCtrlStruct.Bi = cell(1, nRnew);
simpleCtrlStruct.Ci = cell(1, nRnew);
[simpleCtrlStruct.Ai{:}] = deal(zAi);
[simpleCtrlStruct.Bi{:}] = deal(zBi);
[simpleCtrlStruct.Ci{:}] = deal(zCi);

simpleCtrlStruct.dynamics = repmat(0,1,length(simpleCtrlStruct.Pn));
simpleCtrlStruct.details.before_simpl = length(ctrlStruct.Pn);
simpleCtrlStruct.details.after_simpl = length(simpleCtrlStruct.Pn);
simpleCtrlStruct.details.simpl_time = simpl_time;
if Options.greedy,
    simpleCtrlStruct.details.alg = 'greedy';
else
    simpleCtrlStruct.details.alg = 'optimal';
end
simpleCtrlStruct.simplified = 1;

if isfield(simpleCtrlStruct.details,'searchTree'),
    simpleCtrlStruct.details = rmfield(simpleCtrlStruct.details,'searchTree');
    disp('recomputing search tree...');
    simpleCtrlStruct = mpt_searchTree(simpleCtrlStruct);
end

if Options.verbose>=0,
    fprintf('controller partition reduced to %d regions\n',length(simpleCtrlStruct.Pn));
end

details. before = simpleCtrlStruct.details.before_simpl;
details.after = simpleCtrlStruct.details.after_simpl;
details.runTime = simpleCtrlStruct.details.simpl_time;
if Options.greedy,
    details.alg = 'greedy';
else
    details.alg = 'optimal';
end

if nargout < 2,
    clear details
end

if Options.statusbar,
    mpt_statusbar;
end

simpleCtrl = mptctrl(simpleCtrlStruct);

% assign simplified controller in caller's workspace
if ~isempty(inputname(1)),
    assignin('caller',inputname(1),simpleCtrl);
end

%------------------------------------------------------------
function color = sub_preparecolors(ctrlStruct)
% identifies regions which have the same control law


P = ctrlStruct.Pn;
Fi = ctrlStruct.Fi;
Gi = ctrlStruct.Gi;
if iscell(ctrlStruct.sysStruct.B),
    nu = size(ctrlStruct.sysStruct.B{1}, 2);
else
    nu = size(ctrlStruct.sysStruct.B, 2);
end
color.Reg = NaN*ones(1,length(P));
color.Table = {}; 
color.Fi = {};
color.Gi = {};
reg_set = 1:length(P);

d=0;
while length(reg_set) > 0
    
    % add the first region in the set to the new entry in the table 
    % and remove it from the set
    r = reg_set(1);
    d = d+1;
    color.Table{d} = r;
    color.Reg(r) = d;
    reg_set(1) = [];
    
    % check all the regions in the set if they have the same PWA dynamics.
    % if so, add them
    tolEq = 10e-10;
    for k = reg_set
        % fast implementation
        if all(all(abs(Fi{r}(1:nu,:)-Fi{k}(1:nu,:))<=tolEq)) & ... 
                all(all(abs(Gi{r}(1:nu,:)-Gi{k}(1:nu,:))<=tolEq))
            color.Table{d}(end+1) = k;
            color.Reg(k) = d;
            reg_set(find(reg_set==k)) = [];
        end;
    end;
    
end; 

for ii = 1:length(color.Table),
    color.Fi{ii} = ctrlStruct.Fi{color.Table{ii}(1)}(1:nu,:);
    color.Gi{ii} = ctrlStruct.Gi{color.Table{ii}(1)}(1:nu,:);
end
    