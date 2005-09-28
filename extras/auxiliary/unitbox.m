function P=unitbox(dimension, boxsize)
%UNITBOX Creates a unit box centered at origin
%
% P=unitbox(dimension, boxsize)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Creates a unit box (hypercube) of dimension 'dimension' centered at origin
%   with size 'boxsize'
%
% i.e. returns a polytope
%   P=polytope([eye(dimension); -eye(dimension)], boxsize*ones(2*dimension,1))
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% dimension  - dimension of hypercube
% boxsize    - size of the box (Optional, if not given, we assume 1)
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% P     - hypercube in H-representation
%
% see also POLYTOPE
%

% Copyright is with the following author(s):
%
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich
%     kvasnica@control.ee.ethz.ch

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

error(nargchk(1,2,nargin));

if dimension<1,
    error('unitbox: dimension of the hypercube must be greater zero!');
end
if nargin < 2,
    boxsize = 1;
end

P=polytope([eye(dimension); -eye(dimension)], boxsize*ones(2*dimension,1));
