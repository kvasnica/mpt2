function [X,U,Y,mode]=mpt_simSys(sysStruct,x0,inU,Options)
%MPT_SIMSYS Simulates evolution of a given system
%
% [X,U,Y]=mpt_simSys(sysStruct,x0,inU)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Simulates evolution of system defined by sysStruct from a given state x0 using
% inputs defined in 'inU'
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% sysStruct   - system structure in sysStruct format
% x0          - initial state
% inU         - inputs to apply to the system (one row per time step)
% Options.abs_tol - absolute tolerance
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% X    - evolution of states
% U    - evolution of control moves
% Y    - evolution of outputs
% mode - PWA dynamics active at step k
%
% see also MPT_COMPUTETRAJECTORY

% $Id: mpt_simSys.m,v 1.5 2005/05/03 13:07:46 kvasnica Exp $
%
%(C) 2003 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
%         kvasnica@control.ee.ethz.ch

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

global mptOptions
if ~isstruct(mptOptions)
    mpt_error;
end

error(nargchk(3,4,nargin));

if ~isfield(sysStruct,'verified'),
    sysStruct=mpt_verifySysStruct(sysStruct);
end

if nargin<4
    Options = [];
end

if ~isfield(Options, 'abs_tol')
    Options.rel_tol = mptOptions.rel_tol;
end
if ~isfield(Options, 'dynamics')
    Options.dynamics = 0;
end

Options.manualU = inU;
Options.noCScheck = 1;

ispwa = iscell(sysStruct.A);
ismld = 0;
if isfield(sysStruct, 'data'),
    if isfield(sysStruct.data, 'onlymld');
        % this flag will exist only when mpt_sys was called with the 'mld' flag,
        % i.e.:
        %   sysStruct = mpt_sys('hysdelfile', 'mld')
        % in which case no equivalent PWA representation is being created due to
        % complexity issues
        ismld = 1;
    end
end
if ispwa,
    nPWA = length(sysStruct.A);
    nu = size(sysStruct.B{1},2);
else
    nu = size(sysStruct.B,2);
end

if size(inU,1)==nu & size(inU,1)~=size(inU,2),
    inU = inU';
end

X = [];
Y = [];
mode = [];
for ii=1:size(inU,1),
    U = inU(ii,:)';
    if ismld,
        % run the simulator obtained by HYSDEL and stored in sysStruct for
        % systems for which no equivalent PWA representation was created
        simcode = sysStruct.data.SIM.code;
        [xnext, y] = sub_getMLDupdate(simcode, x0, U);
        if isempty(xnext)
            error(['mpt_simSys: no dynamics associated to state ' mat2str(x0') ' and input ' mat2str(U') ' !']);
        end
        % simulator does not give index of active mode, we therefore set it to 0
        dyn = 0;
        
    elseif ispwa,
        dyn = Options.dynamics;
        if dyn==0
            for jj=1:nPWA,
                if all(sysStruct.guardX{jj}*x0 + sysStruct.guardU{jj}*U - sysStruct.guardC{jj} <= Options.rel_tol),
                    dyn = jj;
                    break
                end
            end
        end
        if dyn==0,
            error(['mpt_simSys: no dynamics associated to state ' mat2str(x0') ' and input ' mat2str(U') ' !']);
        end
        A = sysStruct.A{dyn};
        B = sysStruct.B{dyn};
        C = sysStruct.C{dyn};
        D = sysStruct.D{dyn};
        f = sysStruct.f{dyn};
        g = sysStruct.g{dyn};
        xnext = A*x0 + B*U + f;
        y = C*x0 + D*U + g;
        
    else
        xnext = sysStruct.A*x0 + sysStruct.B*U;
        y = sysStruct.C*x0 + sysStruct.D*U;
        if isfield(sysStruct,'f'),
            xnext = xnext + sysStruct.f;
        end
        if isfield(sysStruct,'g')
            y = y + sysStruct.g;
        end
        dyn = 1;
    end
    x0 = xnext;
    X = [X; xnext'];
    Y = [Y; y'];
    mode = [mode; dyn];
end
U = inU;

%-----------------------------------------------------
function [xn, y] = sub_getMLDupdate(simcode, x, u)

try
    eval(simcode);
catch
    xn = [];
    y = [];
end