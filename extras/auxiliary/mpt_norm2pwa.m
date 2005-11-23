function pwafcn = mpt_norm2pwa(P,l,Pn)
% MPT_NORM2PWA  transformes a linear norm into an equivalent PWA fcn representation
%
% pwafcn = mpt_norm2pwa(P,l)
% pwafcn = mpt_norm2pwa(P,l,Pn)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Transformes a linear norm ||P*x||_l into an equivalent PWA function,
%   ||P*x||_l =  pwafcn.Bi{i}*x + pwafcn.Ci{i}  for x \in pwafcn.Pn(i)
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% P            - scaling matrix in ||P*x||_l
% l            - = 1 or Inf, standard vector norm (l=Inf will be assumed if not
%                set)
% Pn           - one polytope over which the norm should be defined (optional)
%
% ---------------------------------------------------------------------------
% OUTPUT
% ---------------------------------------------------------------------------
% pwafcn.Bi    - descriptopn of the PWA function 
%       .Ci
%       .Pn         
% pwafcn.epi   - implicit epigraph description of ||P*x||_l using sdvar.
%                (more efficient than the PWA description for computation)
%
% Note: for simple plotting use plot(pwafcn.epi)

% Copyright is with the following author(s):
%
%(c) 2005 Frank J. Christophersen, Automatic Control Laboratory, ETH Zurich,
%         fjc@control.ee.ethz.ch
%(c) 2005 Johan Loefberg, Automatic Control Laboratory, ETH Zurich,
%         loefberg@control.ee.ethz.ch

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


error(nargchk(1,3,nargin));

global mptOptions

if ~isstruct(mptOptions)
    mpt_error;
end

if nargin < 2
    l = Inf;
end

if isempty(l) | (~isinf(l) & l~=1)
    error('only linear norms are supported, i.e. l = 1 or Inf');
end



[mP,nx] = size(P);  

if nargin < 3
    Pn = unitbox(nx,mptOptions.infbox);
end
if length(Pn)>1
    error('it can only be defined over one polytope')
end
[H, K] = double(Pn);


% use a more efficient epi graph way to describe the norm.
% (more efficient than enumerating the possibilities)
x = sdpvar(nx,1);
[pwafcn.epi,pwafcn.Bi,pwafcn.Ci,pwafcn.Pn] = pwa(norm(P*x,l),set(H*x<=K));


% % alternatively: 
% % using 'enumeration' of the parts [ONLY for 1-norm, yet]
% T=[];
% for ii=0:2^mP-1,
%   a=dec2bin(ii, log2(mP)+1);
%   t=[];
%   for jj=1:length(a),
%     t=[t str2num(a(jj))];
%   end,
%   T=[T; t];
% end
% Tind = find(T(:)==0);
% T(Tind) = -1;
% 
% cnt = 0;
% for cc= 1:2^mP
%     pp = polytope([-repmat(T(cc,:)',1,nx).*P;H], [zeros(mP,1);K]);
%     if isfulldim(pp)
%         cnt = cnt+1;
%         pwafcn.Bi{cnt} = T(cc,:)*P;
%         pwafcn.Ci{cnt} = 0;
%         pwafcn.Pn(cnt) = pp;
%     end
% end%(FOR) cc
% pwafcn.Pfinal = Pn;

return