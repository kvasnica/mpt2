function [nx,nu,ny,ndyn,nbool,ubool,intInfo] = mpt_sysStructInfo(sysStruct)
%MPT_SYSSTRUCTINFO Returns information about system structure
%
% [nx,nu,ny,ndyn,nbool,ubool,intInfo] = mpt_sysStructInfo(sysStruct)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Returns number of states, inputs, outputs and number of dynamics contained in
% a given system structure
%
% internal function
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% sysStruct  - system structure describing an LTI system
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% nx       - number of states
% nu       - number of control inputs
% ny       - number of outputs
% ndyn     - number of dynamics
% nbool    - number of boolean inputs
% ubool    - indexes of integer (or boolean) inputs
% intInfo  - structure with information about overlapping dynamics
%

% $Id: mpt_sysStructInfo.m,v 1.1 2005/02/23 14:01:27 kvasnica Exp $
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

error(nargchk(1,1,nargin));

if ~isstruct(sysStruct),
    error('mpt_sysStructInfo: Input argument must be a sysStruct structure!');
end

if ~isfield(sysStruct,'verified')
    verOpt.verbose=0;
    sysStruct=mpt_verifySysStruct(sysStruct,verOpt);
end

if iscell(sysStruct.A),
    % PWA system
    ndyn = length(sysStruct.A);
    nx = size(sysStruct.A{end},2);
    nu = size(sysStruct.B{end},2);
    ny = size(sysStruct.C{end},1);
else
    % PWA system
    ndyn = 1;
    nx = size(sysStruct.A,2);
    nu = size(sysStruct.B,2);
    ny = size(sysStruct.C,1);
end
nbool = 0;
ubool = [];

if nargout <= 4,
    return
end

if isfield(sysStruct,'Uset')
    % boolean inputs present
    if iscell(sysStruct.Uset),
        for ii=1:length(sysStruct.Uset),
            if any(isinf(sysStruct.Uset{ii})) | any(isinf(-sysStruct.Uset{ii})),
                % this input is continuous
            else
                nbool = nbool+1;
                ubool = [ubool; ii];
            end
        end
    else
        if any(isinf(sysStruct.Uset)) | any(isinf(-sysStruct.Uset)),
            nbool = 0;
        else
            nbool = 1;
            ubool = 1;
        end
    end
else
    nbool = 0;
end

if nargout <= 6
    return
end

Pdyn = {};
if iscell(sysStruct.A),
    Xintersect = {};
    for ii=1:ndyn,
        Xintersect{ii} = [];
        gX = sysStruct.guardX{ii};
        gC = sysStruct.guardC{ii};
        gU = sysStruct.guardU{ii};
        zerorows = [];
        for jj=1:size(gX,1)
            if all(gX(jj,:)==0),
                zerorows = [zerorows; jj];
            end
        end
        nonzerorows = setdiff(1:size(gX,1),zerorows);
        gXnz = gX(nonzerorows,:);
        gCnz = gC(nonzerorows,:);
        gUnz = gU(nonzerorows,:);
        pureX_gX{ii} = gXnz;
        pureX_gU{ii} = gUnz;
        pureX_gC{ii} = gCnz;
        
        if isempty(gXnz),
            P = polytope;
        else
            P = polytope([gXnz gUnz],gCnz);
        end
        
        % project it down to X space
        if ~isfulldim(P) | isempty(gXnz)
            PX = sysStruct.Pbnd;
        else
            try
                TT = evalc('PX = projection(P,1:nx) & sysStruct.Pbnd;');
            catch
                PX = sysStruct.Pbnd;
            end
        end
        Pdyn{ii} = PX;
        
    end
    for ii=1:ndyn,
        for jj=1:ndyn
            if ii==jj,
                Xintersect{ii} = [Xintersect{ii} ii];
                continue
            end
            H = [pureX_gX{ii}; pureX_gX{jj}];
            K = [pureX_gC{ii}; pureX_gC{jj}];
            if isempty(H),
                P = sysStruct.Pbnd;
            else
                P = polytope(H,K,0,2);
            end
            if isfulldim(P),
                Xintersect{ii} = [Xintersect{ii} jj];
            end
        end
    end
else
    Xintersect = 1;
end


if ~iscell(sysStruct.A),
    intInfo = [];
    return
end

Dyn = {};
for ii=1:ndyn,
    dyns = Xintersect{ii};
    D = ii;
    for jj=1:length(dyns),
        if dyns(jj)~=ii,
            D = [D Xintersect{dyns(jj)}];
        end
    end
    Dyn{ii} = unique(D);
end
Davail = ones(length(Dyn),1);
for ii=1:length(Dyn),
    for jj=1:length(Dyn),
        if ii==jj | Davail(jj)==0, continue, end
        if isempty(setdiff(Dyn{ii},Dyn{jj})),
            % Dyn{ii} is a subset of Dyn{jj}
            Davail(ii)=0;
            break
        end
    end
end
ovl_dyns = find(Davail==1);
Dyns = {Dyn{ovl_dyns}};
dynamics_links = [];
for dyn=1:ndyn,
    for jj=1:length(Dyns),
        if any(Dyns{jj} == dyn)
            dynamics_links = [dynamics_links; dyn jj];
            break
        end
    end
end

intInfo.Xintersect = Xintersect;
intInfo.dyns_stack = Dyns;
intInfo.dyns_links = dynamics_links;
intInfo.stacks = length(Dyns);
intInfo.Pdyn = Pdyn;
