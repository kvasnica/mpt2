function answer = dointersect(P1,P2)
%DOINTERSECT Checks if two polytopes / polyarrays intersect
%
% answer = dointersect(P1,P2)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Returns true (1) if input polytopes / polyarrays intersect.
%
% This function does not return the intersection!
%
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P1,P2   - Polytopes or polyarrays
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% answer  - 1 if polytopes / polyarrays intersect, 0 otherwise
%
% see also AND
%

% Copyright is with the following author(s):
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

global mptOptions;

if ~isa(P1, 'polytope') | ~isa(P2, 'polytope'),
    error('DOINTERSECT: both input arguments must be polytope objects!');
end

lenP1 = length(P1);
lenP2 = length(P2);

if dimension(P1) ~= dimension(P2)
    error('DOINTERSECT: polytopes must have the same dimension!');
end

if lenP1 == 1 & lenP2 == 1
    % special case, both inputs are single polytopes
    
    if ~isempty(P1.bbox) & ~isempty(P2.bbox)
        if all(P1.bbox(:,2) < P2.bbox(:,1)) | all(P1.bbox(:,1) > P2.bbox(:,2))
            % even bounding boxes of the two polytopes do not intersect, abort
            % quickly
            answer = 0;
            return
        end
    end

    Hint = [P1.H; P2.H];
    Kint = [P1.K; P2.K];
    [xcheb, rcheb] = chebyball_f(Hint, Kint, mptOptions);
    answer =  (rcheb > mptOptions.abs_tol);

else
    [H1, K1] = double(P1);
    [H2, K2] = double(P2);
    if ~iscell(H1),
        H1 = {H1};
        K1 = {K1};
    end
    if ~iscell(H2),
        H2 = {H2};
        K2 = {K2};
    end
    
    answer = 0;
    for i1 = 1:lenP1,
        for i2 = 1:lenP2,
            Hint = [H1{i1}; H2{i2}];
            Kint = [K1{i1}; K2{i2}];
            [xcheb, rcheb] = chebyball_f(Hint, Kint, mptOptions);
            answer =  (rcheb > mptOptions.abs_tol);
            if answer == 1,
                return
            end
        end
    end
end
