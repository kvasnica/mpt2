function [R,fulldim] = intersect(P1,P2,Options)
%INTERSECT Intersection of 2 polytopes
%
% R = intersect(P1,P2,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Returns normalized minimal representation of intersection of input arguments
%
% USAGE:
%   R=intersect(P1,P2)
%   R=intersect(P1,P2,Options)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P1,P2   - Polytopes or polyarrays
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% R - Polytope containing the intersection of input polytopes
%
% see also AND
%

% $Id: intersect.m,v 1.1.1.1 2004/11/24 10:09:57 kvasnica Exp $
%
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch
% (C) 2003 Mato Baotic, Automatic Control Laboratory, ETH Zurich,
%          baotic@control.ee.ethz.ch

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

if nargin<3
    Options = [];
end

if ~isfield(Options,'abs_tol')
    Options.abs_tol=mptOptions.abs_tol;    % absolute tolerance
end
if ~isfield(Options,'verbose')
    Options.verbose=mptOptions.verbose;    % level of verbosity
end
if ~isfield(Options,'reduce_intersection') % if set to 0, the intersection will not be reduced, i.e. no removal of redundant constraints takes place 
    Options.reduce_intersection=1;
end

normal = 0;
if Options.reduce_intersection==0,
    reduceit=2;
    Options.simplecheck=1;         % for fast regiondiff call
    normal=1;
else
    reduceit=0;
end
    
maxdimP=0;


normal=1;

if ~isa(P1,'polytope') | ~isa(P2,'polytope')
    error('INTERSECT: arguments MUST be a polytope object!');
end

lenP1 = length(P1.Array);
lenP2 = length(P2.Array);
fulldim = 0;

if dimension(P1)~=dimension(P2),
    error('INTERSECT: Polytopes must be of same dimension!');
end

if lenP1>0 | lenP2>0,
    R = mldivide(P1,mldivide(P1,P2,Options),Options);
    fulldim = isfulldim(R(1));
    return
else
    if isempty(P1.RCheb),
        [P1.xCheb, P1.RCheb] = chebyball_f(P1.H,P1.K, Options);
    end
    if isempty(P2.RCheb),
        [P2.xCheb, P2.RCheb] = chebyball_f(P2.H,P2.K, Options);
    end
    
    if P1.RCheb < Options.abs_tol
        P1.H=[];
        P1.K=[];
    end
    if P2.RCheb < Options.abs_tol
        P2.H=[];
        P2.K=[];
    end
    HH=[P1.H; P2.H];
    KK=[P1.K; P2.K];
    if ~normal,
        normal = P1.normal & P2.normal;
    end
    if P1.RCheb < Options.abs_tol | P2.RCheb < Options.abs_tol,
        if Options.verbose>0,
            disp('INTERSECT warning: empty polytope detected!');
        end
        fulldim = 0;
        R=polytope;
        return
    end
    R = polytope(HH, KK, normal, reduceit);   % intersection is obtained by eliminating redundant constraints from the system of inequalities
    fulldim = (R.RCheb > Options.abs_tol);
end
