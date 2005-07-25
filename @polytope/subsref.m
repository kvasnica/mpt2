function Q = subsref(P, X)
%SUBSREF Indexed referencing for polytope objects
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% P(I) is an array formed from the elements of A specifed by the
%      subscript vector I.
%
% If P is a polytope, P(1) returns that polytope
%
% If P is an array of polytopes (e.g. P=[P1 P2 P3]), P(2) returns P2, etc.
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
%
% see also SUBSASGN, END
%

% Copyright is with the following author(s):
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


if numel(X)>1,
    error('??? Attempt to reference field of non-structure array.');
else
    if (~strcmp(X.type,'()')),
        % only indexes in round brackets are allowed
        if X.type(1)=='.',
            error(['Indexing with ''' X.type ''' not supported! Use [H,K]=double(P) to access the H-representation; type ''help polytope'' for more details']);
        end
        error(['Indexing with ''' X.type ''' not supported!']);
    end
end

indices = X.subs{1};
if isempty(indices),
    Q = polytope;
    return
end

lenP = length(P.Array);
if (lenP==0)
    if indices(1)>1 | length(indices)>1,
        error('SUBSREF: ??? Index exceeds array dimension');
    end
    Q=P;
    return
else
    if any(indices<=0),
        error('POLYTOPE:SUBSREF: Index is negative or zero');
    end
    if any(indices>lenP),
        %if any(indices > length(P.Array))
        error('POLYTOPE:SUBSREF: Index exceeds matrix dimension');
    end
    lenInd = length(indices);
    if (lenInd==1),
        Q = P.Array{indices};
    else
        Q = P;
        Q.Array = {P.Array{indices}};
    end
end
