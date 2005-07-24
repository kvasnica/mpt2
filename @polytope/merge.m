function [Pm,details] = merge(Pn,Options)
%MERGE merges polytopes together
%
% [Pm, details] = merge(Pn, Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Uses either greedy merging or optimal merging to join regions.
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% Pn  - polytope array
% Options.greedy   - if set to 1 (default), uses greedy merging. If set to
%                    0, optimal merging will be used (may be slow, requires
%                    espresso solver to be installed)
% Options.verbose  - level of verbosity (0/1/2)
% Options.trials   - for greedy merging, defines number of trials to
%                    improve the solution (default is 1, corresponds to 1 run)
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% Pm               - merged polytopes
% details.before   - number of polytopes before merging
% details.after    - number of polytopes after merging
% details.runTime  - run time of the algorithm
% details.alg      - string, either 'greedy' or 'optimal'
%
% see also MPT_GREEDYMERGING, MPT_OPTMERGE

% $Id: merge.m,v 1.2 2004/12/05 13:39:23 kvasnica Exp $
%
% (C) 2004 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
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

if ~isa(Pn, 'polytope'),
    error('ISINSIDE: input argument MUST be a polytope');
end

if isempty(Pn.Array),
    % nothing to merge, input is single polytope
    Pm = Pn;
    if nargout > 1,
        details.before = 1;
        details.after = 1;
        details.runTime = 0;
        details.alg = 'merge';
    end
    return
end

if nargin<2,
    Options = [];
end

if ~isfield(Options,'greedy')
    % by default use greedy merging. Set this option to 1 for Optimal merging 
    Options.greedy = 1;
end

if ~isfield(Options,'verbose')
    Options.verbose=0;
end

if ~isfield(Options,'trials')
    % number of attempts to improve solution of greedy merging
    Options.trials = 1;
end

if Options.greedy,
    [Pm, details] = mpt_greedyMerging(Pn, Options);
else
    error('Optimal merging not yet available!');
    [Pm] = mpt_optMerge(Pn, Options);
    details.runTime = cputime - startt;
    details.before = length(Pn);
    details.after = length(Pm);
    details.alg = 'mpt_optMergeDivCon';
end

if nargout < 2,
    clear details
end
