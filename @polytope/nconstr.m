function nc = nconstr(P)
%NCONSTR Returns number of constraints that form an H-representation of a polytope
%
% nx = nconstr(P)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Returns number of constraints (i.e. rows of the H matrix)
%
% NOTE: returns a vector of dimensions if P is a polytope array
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P   - Polytope
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% nc  - number of constraints for the given polytope P
%       (returns a vector of dimensions if P is a polytope array)
%
% see also DIMENSION, LENGTH
%

% $Id: nconstr.m,v 1.1.1.1 2004/11/24 10:09:57 kvasnica Exp $
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

% if ~isa(P, 'polytope')
%   error('NCONSTR: Argument MUST be a polytope object');
% end

% number of constraints is just number of rows of the H matrix
if length(P.Array)>0,
    nc=size(P.Array{1}.H,1);
else
    nc = size(P.H,1);
end
