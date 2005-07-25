function fval = subsref(ctrl, X)
%SUBSREF Indexed referencing for MPTCTRL object
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Allows to access each field of the MPTCTRL object directly using the standard
% structure-like indexing, e.g.
%
%   ctrl.Pn
%   ctrl.details
%
% and so on. Type 'struct(ctrl)' to see all accessible fields.
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

if numel(X)>1,
    XX = X(1);
else
    XX = X;
end
if ~strcmp(XX.type,'.'),
    error(['Indexing with ''' X.type ''' not supported!']);
end

ctrl_str = struct(ctrl);
if length(X)==1,
    fname = X.subs;
else
    fname = X(1).subs;
end
if isfield(ctrl_str, fname)
    fval = getfield(ctrl_str, fname);
else
    error(['??? Reference to non-existent field ''' fname '''.']);
end
if length(X)>1,
    try
        fval = subsref(fval, X(2:end));
    catch
        rethrow(lasterror);
    end
end