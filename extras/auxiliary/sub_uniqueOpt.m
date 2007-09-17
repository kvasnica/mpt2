function table = sub_uniqueOpt(Fi, Gi, nu)
% SUB_UNIQUEOPT identifies unique optimizers
%
% table = sub_uniqueOpt(Fi, Gi, nu)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Identifies unique optimizers
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% Fi, Gi   - cell arrays defining the optimizer as J = Fi{r}*x + Gi{r}
% nu       - number of system inputs (optional)
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% table    - linking table

% Copyright is with the following author(s):
%
% (C) 2007 Michal Kvasnica, Slovak University of Technology in Bratislava
%          michal.kvasnica@stuba.sk

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

if ~iscell(Fi)
    Fi = { Fi };
    Gi = { Gi };
end
if length(Fi) ~= length(Gi)
    error('Length of Fi and Gi must be identical.');
end
if nargin < 3
    nu = size(Fi{1}, 1);
end

nr = length(Fi);
table.Reg = NaN*ones(1,nr);
table.Table = {}; 
table.Fi = {};
table.Gi = {};
reg_set = 1:nr;
d=0;

while length(reg_set) > 0
    
    % add the first region in the set to the new entry in the table 
    % and remove it from the set
    r = reg_set(1);
    d = d+1;
    table.Table{d} = r;
    table.Reg(r) = d;
    reg_set(1) = [];
    
    % check all the regions in the set if they have the same PWA dynamics.
    % if so, add them
    tolEq = 10e-10;
    for k = reg_set
        % fast implementation
        if all(all(abs(Fi{r}(1:nu,:)-Fi{k}(1:nu,:))<=tolEq)) & ... 
                all(all(abs(Gi{r}(1:nu,:)-Gi{k}(1:nu,:))<=tolEq))
            table.Table{d}(end+1) = k;
            table.Reg(k) = d;
            reg_set(find(reg_set==k)) = [];
        end;
    end;
    
end; 

for ii = 1:length(table.Table),
    table.Fi{ii} = Fi{table.Table{ii}(1)}(1:nu,:);
    table.Gi{ii} = Gi{table.Table{ii}(1)}(1:nu,:);
end
