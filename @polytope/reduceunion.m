function [Pret,keep] = reduceunion(P,Options)
% REDUCEUNION Removes redundant elements from a polytope array
%
% [Pret,kept] = reduceunion(P,Options),
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% given a (possibly non-convex) union of polytopes P, removes
% all elements of P which are completely covered by the other regions
%
% i.e. assume we have 3 polytopes:
%      p1 = polytope([0 -1; -1 0; 0 1; 1 0],[1;0;1;1])
%      p2 = polytope([0 -1; -1 0; 0 1; 1 0],[1;1;1;0])
%      p3 = polytope([0 -1; -1 0; 0 1; 1 0],[1;1;1;1])
%
% then if Pu=[p1 p2 p3], this function removes polytopes p1 and p2, since they are completely
% covered by a larger polytope (p3 in this case)
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
%   P           -   polytope array
%   Options     -   will be used when calling regiondiff
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
%
%  Pret    -  reduced polytope array
%  kept    -  0/1 vector of length(P) which stores a 0 at index i if polytope
%             P(i) is redundant and 1 at index i if P(i) is not redundant.
%

% (C) 2004 Pascal Grieder, Automatic Control Laboratory, ETH Zurich,
%          grieder@control.ee.ethz.ch
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%          kvasnica@control.ee.ethz.ch

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

lenP=length(P.Array);
if(lenP==0)
    Pret=P;
    keep=1;
    return 
end

%initialize
Options.reduce=0;         % for fast subset check
Options.constructset=0;   % for fast subset check
Options.reduce_output=0;  % for fast subset check

%sort polyarray according to chebychev radius
index=[];
for i=1:length(P.Array)
    index = [index; P.Array{i}.RCheb i];
end
index = sort(index,1);   %sort in ascending order

keep=ones(1,lenP);     % initialize the vector, 1 means keep the region, 0 kick it out
for ii=1:lenP,   
    sel=index(end-ii+1);          %access region number
    ind1=[1:sel-1,sel+1:lenP];    % all indices except of the current one
    ind2=find(keep==0);           % indices of regions which were kicked out
    ind=setdiff(ind1,ind2);       % remove indices of regions, which were already kicked out
    
    if isempty(ind),              % if the set of indices is empty, that means that we kicked out all polytopes except of the last one
        Pret=P.Array{sel};          % so return it
        return
    end
    Pum = [P.Array{ind}];
    R=regiondiff(P.Array{sel},Pum,Options);    % solve the coverage problem
    if ~isfulldim(R),
        keep(end-ii+1)=0;                            % if the solution is not empty, means that Pu(ii) is fully covered by the remaining polytopes, we kick it out
    end
end

%write ouput
ind=find(keep==1);
Pret=polytope;
for kk=1:length(ind)
    Pret=[Pret P.Array{ind(kk)}];
end

return
