function C = mpt_mpsol2ctrl(Pfinal, Pn, Fi, Gi, Ai, Bi, Ci) 
%MPT_MPSOL2CTRL Convert solution of multi-parametric program into a controller structure
%
% C = mpt_mpsol2ctrl(Pfinal, Pn, Fi, Gi, Ai, Bi, Ci) 
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Converts a solution of a multi-parametric program into a dummt controller
% structure which is then suitable for mpt_removeOverlaps.
%
% Internal function
% 
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
%

% Copyright is with the following author(s):
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

dummystruct.dummyfield = [];

C.sysStruct = dummystruct;
C.probStruct = dummystruct;
C.details.origSysStruct = dummystruct;
C.details.origProbStruct = dummystruct;

nr = length(Pn);

C.Pfinal = Pfinal;
C.Pn = Pn;
C.Fi = Fi;
C.Gi = Gi;
if isempty(Ai),
    Ai = cell(1, nr);
end
C.Ai = Ai;
C.Bi = Bi;
C.Ci = Ci;
C.dynamics = repmat(1, 1, nr);
C.overlaps = 0;
