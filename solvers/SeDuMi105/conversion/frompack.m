% FROMPACK Converts a cone problem in SDPPACK format to SEDUMI format.
%
% [At,c] = frompack(A,b,C,blk)  Given a problem (A,b,C,blk) in the
%   SDPPACK-0.9-beta format, this produces At and c for use with
%   SeDuMi. This lets you execute
%
% [x,y,info] = SEDUMI(At,b,c,blk);
% 
% IMPORTANT: this function assumes that the SDPPACK function `smat'
%    exists in your search path.
%
% SEE ALSO sedumi.

function [At,c] = frompack(A,b,C,blk)
 %  
 %   This file is part of SeDuMi 1.02   (03AUG1998)
 %   Copyright (C) 1998 Jos F. Sturm
 %   CRL, McMaster University, Canada.
 %   Supported by the Netherlands Organization for Scientific Research (NWO).
 % 
 %   This program is free software; you can redistribute it and/or modify
 %   it under the terms of the GNU General Public License as published by
 %   the Free Software Foundation; either version 2 of the License, or
 %   (at your option) any later version.
 % 
 %   This program is distributed in the hope that it will be useful,
 %   but WITHOUT ANY WARRANTY; without even the implied warranty of
 %   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 %   GNU General Public License for more details.
 % 
 %   You should have received a copy of the GNU General Public License
 %   along with this program; if not, write to the Free Software
 %   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 %

m = length(b);
% ----------------------------------------
% In SDPPACK, 0s are sometimes used  as emptyset and vice versa.
% ----------------------------------------
if blk.l == 0
  A.l = []; C.l = [];
end
if isempty(blk.q)
  A.q = []; C.q = [];
end
if isempty(blk.s)
  A.s = []; C.s = [];
end
% ----------------------------------------
% SDP:
% ----------------------------------------
Asdp = [];
if sum(blk.s) == 0
  csdp = [];
else
  for i=1:m
    Asdp = [Asdp blk2vec( smat(sparse(A.s(i,:)),blk.s), blk.s ) ];
  end
  csdp = blk2vec(C.s,blk.s);
end
% ----------------------------------------
% Assemble LP, LORENTZ and SDP.
%----------------------------------------
At = [A.l'; A.q'; Asdp];
c = [C.l; C.q; csdp];