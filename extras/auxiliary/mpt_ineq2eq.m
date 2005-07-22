function [A, B, Aeq, Beq] = mpt_ineq2eq(A, B)
%MPT_INEQ2EQ Detects inequality constraints whihc form equalities
%
% [Ain, Bin, Aeq, Beq] = mpt_ineq2eq(A, B)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% For a system of inequalities A*x<=B, this function detects and returns those
% inequality constraints which form equalities. For instance:
%
% A = [1; -1; 1]; B = [1; -1; 2];
%
% The output will lead:
% Ain = [-1]; Bin = [2]; Aeq = [1]; Beq = 1;
%
% such that the original problem can be rewritten as:
%   Ain*x <= Bin
%   Aeq*x  = Beq
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% A, B        - system of inequalities
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% Ain,Bin     - new system of inequalities
% Aeq,Beq     - system of equalities
%

% $Id: mpt_ineq2eq.m,v 1.1 2005/07/09 20:51:08 kvasnica Exp $
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

[ne, nx] = size(A);
Aeq = [];
Beq = [];
ind_eq = [];
I = ones(nx+1, 1);
for ii = 1:ne-1,
    a1 = A(ii, :);
    b1 = B(ii);
    s = sum(a1) + b1;
    
    % get matrix which contains all rows starting from ii+1 
    M = [-A(ii+1:end, :) -B(ii+1:end)];
    
    % quickly compute sum of each row
    sumM = M*I;
    
    % possible candidates are those rows whose sum is equal to the sum of the
    % original row
    possible_eq = find(sumM == s) + ii;
    for jj = possible_eq',
        % now compare if the two inequalities (the second one with opposite
        % sign) are really equal (hence they form an equality constraint)
        a2 = -A(jj, :);
        b2 = -B(jj);
        if b1==b2,
            % first compare the B part, this is very cheap
            if sum(a1)==sum(a2),
                % now compare sum of the A part (this is still cheaper then
                % doing all(a1==a2) here
                if all(a1==a2),
                    % finaly compare every element of the two inequalities
                    
                    % jj-th inequality together with ii-th inequality forms an equality
                    % constraint
                    ind_eq = [ind_eq; ii jj];
                    break
                end
            end
        end
    end
end

if isempty(ind_eq),
    % no equality constraints
    return
else
    % indices of remaining constraints which are inequalities
    ind_ineq = setdiff(1:ne, ind_eq(:));
    Aeq = A(ind_eq(:,1), :);
    Beq = B(ind_eq(:,1));
    A = A(ind_ineq, :);
    B = B(ind_ineq);
end