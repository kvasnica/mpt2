function sysStruct = mpt_pwa2sys(PWA, MLD)
%MPT_PWA2SYS Converts structure generated by hys2pwa.m to a sysStruct structure
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Converts structure generated by hys2pwa.m to a sysStruct structure
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

% $Id: mpt_pwa2sys.m,v 1.7 2005/03/13 16:57:27 kvasnica Exp $
%
%(C) 2005 Michal Kvasnica, Automatic Control Laboratory, ETH Zurich,
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

% each "binary" constraint x==1 is rewritten to x >= threshold
% each "binary" constraint x==0 is rewritten to x <= threshold
threshold = 0.5;

if ~iscell(PWA)
    % easier case, no binary states/inputs

    if iscell(PWA.fx)
        sysStruct.A = PWA.fx;
        sysStruct.B = PWA.fu;
        sysStruct.f = PWA.f0;
        sysStruct.C = PWA.gx;
        sysStruct.D = PWA.gu;
        sysStruct.g = PWA.g0;

        ndyn = length(PWA.fx);
        nx = size(sysStruct.A{1},2);
        sysStruct.guardX = cell(1, ndyn);
        sysStruct.guardU = cell(1, ndyn);
        sysStruct.guardC = PWA.Ki;
        for idyn = 1:ndyn,
            sysStruct.guardX{idyn} = PWA.Hi{idyn}(:,1:nx);
            sysStruct.guardU{idyn} = PWA.Hi{idyn}(:,nx+1:size(PWA.Hi{idyn},2));
        end
    else
        sysStruct.A = { PWA.fx };
        sysStruct.B = { PWA.fu };
        sysStruct.f = { PWA.f0 };
        sysStruct.C = { PWA.gx };
        sysStruct.D = { PWA.gu };
        sysStruct.g = { PWA.g0 };

        nx = size(sysStruct.A,2);
        if iscell(PWA.Hi),
            Hi = PWA.Hi{1};
        else
            Hi = PWA.Hi;
        end
        sysStruct.guardX = { Hi(:,1:nx) };
        sysStruct.guardU = { Hi(:,nx+1:size(Hi,2)) };
        sysStruct.guardC = { Ki };
    end
    
   
else
    % binary inputs/states
    
    nP = length(PWA);
    ndyn = 0;
    for ip = 1:nP,
        ndyn = ndyn + length(PWA{ip}.fx);
    end
    
    sysStruct.A = {};
    sysStruct.B = {};
    sysStruct.f = {};
    sysStruct.C = {};
    sysStruct.D = {};
    sysStruct.g = {};
    sysStruct.guardX = {};
    sysStruct.guardU = {};
    sysStruct.guardC = {};
    
    nxr = size(PWA{1}.fx{1},2);
    nxb = length(PWA{1}.xb);
    nx = size(PWA{1}.fx{1},1);
    
    nur = size(PWA{1}.fu{1},2);
    nub = length(PWA{1}.ub);
    nu = nur + nub;
    
    ny = size(PWA{1}.gx{1},1);
    
    for ip = 1:nP,
        ub = PWA{ip}.ub;
        xb = PWA{ip}.xb;
        for idyn = 1:length(PWA{ip}.Hi)
            
            if isempty(PWA{ip}.Hi{idyn}),
                PWA{ip}.Hi{idyn} = zeros(ub, nxr+nur);
                PWA{ip}.Ki{idyn} = zeros(ub,1);
            end
            
            sysStruct.A{end+1} = [PWA{ip}.fx{idyn} zeros(nx, nxb)];
            if isempty(PWA{ip}.fu{idyn}),
                sysStruct.B{end+1} = zeros(nx, nu);
            else
                sysStruct.B{end+1} = [PWA{ip}.fu{idyn} zeros(nx, nub)];
            end
            if isempty(PWA{ip}.f0{idyn}),
                sysStruct.f{end+1} = zeros(nx, 1);
            else
                sysStruct.f{end+1} = PWA{ip}.f0{idyn};
            end
            
            if isempty(PWA{ip}.gx{idyn}),
                sysStruct.C{end+1} = eye(nx);
            else
                sysStruct.C{end+1} = [PWA{ip}.gx{idyn} zeros(ny, nxb)];
            end
            if isempty(PWA{ip}.gu{idyn}),
                sysStruct.D{end+1} = zeros(nx, nu);
            else
                sysStruct.D{end+1} = [PWA{ip}.gu{idyn} zeros(ny, nub)];
            end
            if isempty(PWA{ip}.g0{idyn}),
                sysStruct.g{end+1} = zeros(nx, 1);
            else
                sysStruct.g{end+1} = PWA{ip}.g0{idyn};
            end
            
            % first extract guards for real states and inputs
            Hi = PWA{ip}.Hi{idyn};
            nc = size(Hi,1);
            Guard_x = [Hi(:,1:nxr) zeros(nc, nxb)];
            Guard_u = [Hi(:,nxr+1:size(Hi,2)) zeros(nc, nub)];
            Guard_c = PWA{ip}.Ki{idyn};
            
            % include guards for binary states
            for ib = 1:nxb,
                xb = PWA{ip}.xb(ib);
                if xb==1,
                    Guard_x = [Guard_x; zeros(1, nxr) zeros(1, ib-1) -1 zeros(1, nxb-ib)];
                    Guard_u = [Guard_u; zeros(1, nu)];
                    Guard_c = [Guard_c; -threshold];
                else
                    Guard_x = [Guard_x; zeros(1, nxr) zeros(1, ib-1) 1 zeros(1, nxb-ib)];
                    Guard_u = [Guard_u; zeros(1, nu)];
                    Guard_c = [Guard_c; threshold];
                end
            end
            
            % include guards for binary inputs
            for ib = 1:nub,
                ub = PWA{ip}.ub(ib);
                if ub==1,
                    Guard_x = [Guard_x; zeros(1, nx)];
                    Guard_u = [Guard_u; zeros(1, nur) zeros(1, ib-1) -1 zeros(1, nub-ib)];
                    Guard_c = [Guard_c; -threshold];
                else
                    Guard_x = [Guard_x; zeros(1, nx)];
                    Guard_u = [Guard_u; zeros(1, nur) zeros(1, ib-1) 1 zeros(1, nub-ib)];
                    Guard_c = [Guard_c; threshold];
                end
            end
            
            sysStruct.guardX{end+1} = Guard_x;
            sysStruct.guardU{end+1} = Guard_u;
            sysStruct.guardC{end+1} = Guard_c;
            
        end
    end
    sysStruct.Uset = cell(1,nu);
    for iu = 1:nur,
        % denote 'nur' inputs as real
        sysStruct.Uset{iu} = [-Inf Inf];
    end
    for iu = 1:nub,
        % set 'nub' inputs as boolean
        sysStruct.Uset{nur+iu} = [0 1];
    end
end

if isfield(MLD, 'nxb')
    if MLD.nxb~=0,
        sysStruct.xbool = MLD.nxr+1:MLD.nx;
    end
end

if isfield(MLD, 'uu') & isfield(MLD, 'ul'),
    if all(~isinf(MLD.uu)) & all(~isinf(MLD.ul)) & nu>0,
        sysStruct.umax = MLD.uu;
        sysStruct.umin = MLD.ul;
    end
end

if isfield(MLD, 'xu') & isfield(MLD, 'xl'),
    if all(~isinf(MLD.xu)) & all(~isinf(MLD.xl)) & nx>0,
        sysStruct.xmin = MLD.xl;
        sysStruct.xmax = MLD.xu;
        sysStruct.Pbnd = polytope([eye(nx); -eye(nx)], [MLD.xu; -MLD.xl], 1);
    elseif nx>0
        sysStruct.Pbnd = unitbox(nx, mptOptions.infbox);
    end
end

sysStruct.data.MLD = MLD;
