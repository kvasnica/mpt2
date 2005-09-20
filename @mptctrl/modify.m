function ctrl = modify(ctrl, action, indeces)
%MODIFY Modifies an MPTCTRL object
%
%   ctrl = modify(ctrl, action, indeces)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Modifies a given MPTCTRL object by either removing and adding regions.
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% ctrl     - explicit controller (an MPTCTRL object)
% action   - what to do:
%               'remove' - remove selected regions
%               'pick'   - pick a subset of regions
% indeces  - indeces of regions to remove/pick
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
%
% ctrl     - updated MPTCTRL object with given regions removed
%

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

if ~isa(ctrl, 'mptctrl')
    error('First input must be an MPTCTRL object.');
end
if ~isa(action, 'char')
    error('Second input must be a string');
end
if ~isa(indeces, 'double')
    error('Third input must be a double.');
end
if ~isexplicit(ctrl)
    error('First input must be an explicit controller.');
end

if any(indeces > length(ctrl.Pn))
    error('Index exceeds total number of regions');
end
if any(indeces <= 0)
    error('Indeces must be only positive.');
end

% be sure to catch cases like indeces=[1 2 3 1]
indeces = unique(indeces);

switch lower(action)
    case 'remove',
        % remove given regions
        ctrl = sub_keep(ctrl, setdiff(1:length(ctrl.Pn), indeces));
    case 'pick',
        % pick selected regions
        ctrl = sub_keep(ctrl, indeces);
    otherwise
        error(sprintf('Unknown operation ''%s''.', action));
end


%---------------------------------------------------------------
function ctrl = sub_keep(ctrl, keep)
% keeps regions given by indeces 'keep'

nkeep = length(keep);

if nkeep > 0,
    Fi = cell(1, nkeep);
    Gi = cell(1, nkeep);
    Ai = cell(1, nkeep);
    Bi = cell(1, nkeep);
    Ci = cell(1, nkeep);
    
    % just keep elements at indeces 'keep'
    [Fi{:}] = deal(ctrl.Fi{keep});
    [Gi{:}] = deal(ctrl.Gi{keep});
    [Ai{:}] = deal(ctrl.Ai{keep});
    [Bi{:}] = deal(ctrl.Bi{keep});
    [Ci{:}] = deal(ctrl.Ci{keep});
    
    ctrl.Pn = ctrl.Pn(keep);
    % note that Pfinal cannot be a single polytope any longer, because we don't
    % know which polytopes we are removing. we _could_ compute a set difference
    % between Pfinal and the polytopes to be removed, but that would simply be
    % expensive
    ctrl.Pfinal = ctrl.Pn;
    ctrl.Fi = Fi;
    ctrl.Gi = Gi;
    ctrl.Ai = Ai;
    ctrl.Bi = Bi;
    ctrl.Ci = Ci;
    ctrl.dynamics = ctrl.dynamics(keep);
    ctrl.details.modified = 'removed';
    
    % now handle fields which are specific to certain control strategies. these
    % fields are mostly important for plotting purposes.
    if isfield(ctrl.details, 'regionHorizon'),
        % infinite-time solution for LTI systems
        ctrl.details.regionHorizon = ctrl.details.regionHorizon(keep);
    end
    if isfield(ctrl.details, 'IterStore'),
        % minimum-time solution for LTI systems
        ctrl.details.IterStore = ctrl.details.IterStore(keep);
    end
    if isfield(ctrl.details, 'activeConstraints'),
        % mpqp solutions
        ac = cell(1, nkeep);
        [ac{:}] = deal(ctrl.details.activeConstraints{keep});
        ctrl.details.activeConstraints = ac;
    end
    if isfield(ctrl.details, 'Horizon'),
        % cftoc for pwa systems - open loop solution
        if iscell(ctrl.details.Horizon),
            fprintf('Warning: Open-loop solution is stored but can''t be modified. This can lead to problems.\n');
        end
    end
    
else
    % no regions to keep
    ctrl = mptctrl;
end
