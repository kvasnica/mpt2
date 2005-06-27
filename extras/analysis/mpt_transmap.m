function [tmap, P1, P2]=mpt_transmap(P1, A, f, P2, Options)
%MPT_TRANSMAP Computes transition map
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Computes map of transitions which are NOT feasible. Specifically:
%   domain(P2(j), A{i}, f{i}, P1(i)) is NOT feasible if tmap(i, j) is non-zero
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P1          - polytope array
% A, f        - cells containing affine map: x(k+1) = A{i}*x + f{i}
% P2          - polytope array
% Options.maxsplanes - maximum number of generated separating hyperplanes when
%                      computing transition map (default is 1000)
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% tmap        - sparse matrix with non-zero elements on indexes which correspond
%               to transitions whiche are NOT FEASIBLE. e.g.:
%               tmap(i, j) means domain(P2(j), A, f, P1(j)) does NOT exist
% P1          - input polyarray P1 with extreme points stored inside for each
%               polytope
% P2          - input polyarray P2 with extreme points stored inside for each
%               polytope
%

% $Id: mpt_transmap.m,v 1.1 2005/06/27 20:19:42 kvasnica Exp $
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

global mptOptions
if ~isstruct(mptOptions),
    mpt_error;
end

error(nargchk(3,5,nargin));

if nargin < 5,
    Options = [];
end
p1p2equal = 0;
if nargin==4,
    if isa(P2, 'struct'),
        Options = P2;
        P2 = P1;
        p1p2equal = 1;
    end
end
if nargin < 4,
    p1p2equal = 1;
    P2 = P1;
    Options = [];
end

if ~isfield(Options, 'maxsplanes'),
    % gives maximum number of auto-generated separating hyperplanes
    Options.maxsplanes = 1000;
end
if isfield(Options, 'p1p2equal'),
    p1p2equal = Options.p1p2equal;
end

maxsplanes = Options.maxsplanes;
lpsolver = mptOptions.lpsolver;
abs_tol = mptOptions.abs_tol;

if ~isa(P1, 'polytope'),
    error('First input must be a polytope object.');
end
if dimension(P1) ~= dimension(P2),
    error('Dimensions of polytopes must be identical.');
end
if ~isa(P2, 'polytope'),
    error('Fourth input must be a polytope object.');
end

lenP1 = length(P1);
lenP2 = length(P2);
if length(A) ~= lenP1,
    error('Length of A must equal length of P1.');
end
if length(f) ~= lenP1,
    error('Length of f must equal length of P1.');
end

EP1 = cell(1, lenP1);
EP2 = cell(1, lenP2);

% compute extreme points of each polytopes
for ii = 1:lenP1,
    [EP1{ii}, R, P1(ii)] = extreme(P1(ii));
end
if p1p2equal,
    % if P1 == P2, don't recompute extreme points twice
    EP2 = EP1;
    P2 = P1;
else
    for ii = 1:lenP2,
        [EP2{ii}, R, P2(ii)] = extreme(P2(ii));
    end
end

% compute bounding boxes of the two polyarrays
[R, bbox1_low, bbox1_up] = bounding_box(P1, struct('noPolyOutput', 1));
if p1p2equal,
    bbox2_low = bbox1_low;
    bbox2_up = bbox1_up;
else
    [R, bbox2_low, bbox2_up] = bounding_box(P1, struct('noPolyOutput', 1));
end

% compute initial separating hyperplanes by taking the bounding box, computing
% it's center and taking axis through the center orthogonal to x1, x2, ..., xn
dim = dimension(P1);
bbox1_center = (bbox1_up - bbox1_low) / 2;
bbox2_center = (bbox2_up - bbox2_low) / 2;

splanes = [];
for idim = 1:dim,
    % each hyperplane is of the form [A b], where A*x <= b
    oneplane = [zeros(1, dim) bbox1_center(dim)];
    oneplane(idim) = 1;
    splanes = [splanes; oneplane];
end

% if P1 ~= P2 and their bounding boxes are not identical, include also
% separating hyperplanes based on center of the second bounding box
if ~p1p2equal | any(bbox1_center ~= bbox2_center),
    for idim = 1:dim,
        % each hyperplane is of the form [A b], where A*x <= b
        oneplane = [zeros(1, dim) bbox2_center(dim)];
        oneplane(idim) = 1;
        splanes = [splanes; oneplane];
    end
end

newsplanescount = 0;

% alocate the transition map as a sparce matrix initialized to 0
% tmap(index_p1, index_p2) will be 1 if NO transition exists
tmap = spalloc(lenP1, lenP2, lenP1*lenP2);

% now try to rule out as many transitions as possible
for i1 = 1:lenP1,
    P1_v = EP1{i1};
    % affine transformation of polytope P1(i1)
    nv = size(EP1{i1}, 1);
    P1_v = (A{i1}*P1_v' + repmat(f{i1}, 1, nv))';
    for i2 = 1:lenP2,
        % vertices of the second polytope
        P2_v = EP2{i2};

        if newsplanescount < maxsplanes,
            % compute new separating hyperplane
            [sephp, xopt] = sub_sephp(P1_v, P2_v, lpsolver);
            if sephp,
                % separating hyperplane found, add it to the stack
                splanes = [splanes; (xopt(:))'];
                newsplanescount = newsplanescount + 1;
                
                % moreover we know that no transtion exists, since separating
                % hyperplane can be found
                tmap(i1, i2) = 1;
                
                continue
            end
        end

        % check if sets of points are separated by an existing separating
        % hyperplane
        separated = sub_areseparated(P1_v, P2_v, splanes, abs_tol);
        if separated,
            % yes, points are separated, hence no transition between these two
            % sets of points exists 
            tmap(i1, i2) = 1;
            continue
        end
        
    end
end

%----------------------------------------------------------------------
function result = sub_areseparated(V1, V2, hypplanes, abs_tol)
% checks if sets of points V1 and V2 are separated by a hyperplane

result = 0;
nv1 = size(V1, 1);
nv2 = size(V2, 1);
V1 = V1';
V2 = V2';
for iplanes = 1:size(hypplanes, 1),
    % check all hyperplanes
    hplane = hypplanes(iplanes, :);
    a = hplane(1:end-1);
    b = hplane(end);
    if all(a*V1 - b <= abs_tol),
        % points V1 are strictly on the positive side of this hyperplane
        if all(-a*V2 + b <= abs_tol),
            % points V2 are strictly on the negative side of this hyperplane
            result = 1;
            return
        end
    end
end        


%----------------------------------------------------------------------
function [sephp, xopt] = sub_sephp(V1, V2, lpsolver)

% try to find a separating hyperplane between P and Pn
[nv1, nx] = size(V1);
nv2 = size(V2, 1);
f = -ones(1,nx+1);
A = [V1 -ones(nv1,1); -V2 ones(nv2, 1); zeros(1, nx) 1];
B = [zeros(nv1+nv2, 1); 1];
[xopt,fval,lambda,exitflag,how]=mpt_solveLPi(f,A,B,[],[],[],lpsolver,0);

sephp = ~all(abs(xopt<1e-8)) & ~any(xopt==1e9);
