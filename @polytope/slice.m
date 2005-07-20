function Pcut = slice(PA,cut_dim,cut_value)
%SLICE cuts a polytope (polytope array) at prespecified values
%
%
% Pcut = slice(PA,cut_dim,cut_value,Options)
% 
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Orthogonal cut through a polytope (polytope array) at prespecified values.
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% PA             - polytope array               
% cut_dim        - vector of x-dimensions at which to cut
% cut_value      - vector of corresponding values at which to cut
%                  if not given, cut is performed at zero values of respective
%                  coordinates
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% Pcut            - polytope array
%

% $Id: slice.m,v 1.1 2005/07/20 13:27:40 kvasnica Exp $
%
% (C) 2005 Frank J. Christophersen, Automatic Control Laboratory, ETH Zurich,
%          fjc@control.ee.ethz.ch

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


error(nargchk(2,3,nargin));

global mptOptions
if ~isstruct(mptOptions),
    mpt_error;
end

if ~isa(PA,'polytope') 
    error('SLICE: PA must be a polytope object!');
end

if length(unique(cut_dim)) ~= length(cut_dim)
    error('SLICE: same coordinate to cut is multiply defined!')
end


Pcut = mptOptions.emptypoly;
nx   = dimension(PA);
if length(cut_dim)>=nx
    error('SLICE: the cut is over-defined!')
end

if nargin < 3,
    cut_value = zeros(length(cut_dim), 1);
end
if any(cut_dim > dimension(PA)) | any(cut_dim < 1),
    error('SLICE: cut dimension exceeds polytope dimension.');
end
if length(cut_dim(:)) ~= length(cut_value(:)),
    error('SLICE: cut dimensions and cut values must of the same length.');
end

tokeep = [setdiff([1:nx]',cut_dim(:))]';
tokick = sort([cut_dim(:)';cut_value(:)'],2);

lenPA = length(PA.Array);
if lenPA==0,
    % input is a single polytope
    H = PA.H;
    K = PA.K;
    HH = H(:,tokeep);
    KK = K-H(:,tokick(1,:))*tokick(2,:)';
    Pcut = polytope(HH, KK);
else    
    % input is a polytope array
    for ii = 1:lenPA,
        H=[];K=[]; 
        HH=[];KK=[];
        
        H = PA.Array{ii}.H;
        K = PA.Array{ii}.K;
        
        HH = H(:,tokeep);
        KK = K-H(:,tokick(1,:))*tokick(2,:)';
        Pcut = [Pcut polytope(HH,KK)];
    end
end
