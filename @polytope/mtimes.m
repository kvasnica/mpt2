function R=mtimes(P,Q,Options)
%MTIMES Polytope multiplication
%
% MTIMES Polytop multiplication
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
%  Three cases could happen:
%   1. Multiplication of two polytopes P1*P2 leads to a higher dimensional
%      polytope [H1 0; 0 H2] <= [K1; K2]
%   2. Multiplication of a polytope by a matrix -- this leads an affine
%      transformation performed by range.m
%   3. Multiplication of a polytope by a scalar -- if the origin is included
%      in the polytope, the polytope will be shrinked
%
% USAGE:
%   R = P1*P2
%   R = P1*[1 0; 1 1]
%   R = P1*alfa
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% R     - resulting polytope
%

% $Id: mtimes.m,v 1.4 2005/04/21 10:52:10 kvasnica Exp $
%
% (C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
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

if ~(isa(P,'polytope') | isa(Q, 'polytope'))
  error('MTIMES: Argument MUST be a polytope object');
end

if isa(P,'polytope') & isa(Q,'polytope')
    if length(P.Array)>0 | length(Q.Array)>0,
        error('MTIMES: arrays of polytope are not supported by this function');
    end
    [Pnc, Pnx] = size(P.H);
    [Qnc, Qnx] = size(Q.H);
    % form polytope in higher dimension
    H = [P.H zeros(Pnc,Qnx); zeros(Qnc,Pnx) Q.H];
    K = [P.K; Q.K];
    if P.minrep & Q.minrep,
        R = polytope(H,K,0,1);
    else
        R = polytope(H,K);
    end
    return
end

% P is a double, Q is a polytope
if isa(P,'double') & isa(Q,'polytope'),
    if all(size(P)==[1 1]),
        % scaling with a scalar
        
        if P==1,
            % special case - scaling with 1 = no scaling [issue112]
            R = Q;
            return
        end
        
        % handle polyarrays:
        if length(Q.Array)>0,
            R = polytope;
            for ii=1:length(Q.Array),
                [nc, nx] = size(Q.Array{ii}.H);
                if ~isinside(Q.Array{ii}, zeros(nx,1)),
                    % origin is not included --> error
                    error('MTIMES: when scaling with a scalar, origin has to be included in the polytope');
                else
                    % origin is included, do the scaling
                    R = [R polytope(Q.H/P, Q.K)];
                end
            end
            return
        end
                
        [nc, nx] = size(Q.H);
        if ~isinside(Q, zeros(nx,1)),
            % origin is not included --> error
            error('MTIMES: when scaling with a scalar, origin has to be included in the polytope');
        else
            % origin is included, do the scaling
            R = polytope(Q.H/P, Q.K);
            return
        end
    else
        % P is a matrix, perform affine transformation
        R = range(Q,P);
        return
    end
end

% P is a polytope, Q is a double
if isa(Q,'double') & isa(P,'polytope'),
    if all(size(Q)==[1 1]),
        % scaling with a scalar

        if Q==1,
            % special case - scaling with 1 = no scaling [issue112]
            R = P;
            return
        end

        % handle polyarrays:
        if length(P.Array)>0,
            R = polytope;
            for ii=1:length(P.Array),
                [nc, nx] = size(P.Array{ii}.H);
                if ~isinside(P.Array{ii}, zeros(nx,1)),
                    % origin is not included --> error
                    error('MTIMES: when scaling with a scalar, origin has to be included in the polytope');
                else
                    % origin is included, do the scaling
                    R = [R polytope(P.H/Q, P.K)];
                end
            end
            return
        end
                
        [nc, nx] = size(P.H);
        if ~isinside(P, zeros(nx,1)),
            % origin is not included --> error
            error('MTIMES: when scaling with a scalar, origin has to be included in the polytope');
        else
            % origin is included, do the scaling
            R = polytope(P.H/Q, P.K);
            return
        end
    else
        % P is a matrix, perform affine transformation
        R = range(P,Q);
        return
    end
end

error('MTIMES: unhandled case! see help polytope/mtimes for details...');
