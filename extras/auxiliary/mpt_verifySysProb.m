function [sysStruct,probStruct]=mpt_verifySysProb(sysStruct,probStruct,Options)
%MPT_VERIFYSYSPROB Verifies system and problem structures
%
% [sysStruct,probStruct]=mpt_verifySysProb(sysStruct,probStruct,Options)
%
% ---------------------------------------------------------------------------
% DESCRIPTION
% ---------------------------------------------------------------------------
% Checks sysStruct and probStruct structures for validity
%
% Internal function.
%
% ---------------------------------------------------------------------------
% INPUT
% ---------------------------------------------------------------------------
% sysStruct        - System structure in the sysStruct format
% probStruct       - Problem structure in the probStruct format
% Options.verbose  - level of verbosity
%
% Note: If Options is missing or some of the fields are not defined, the default
%       values from mptOptions will be used
%
% ---------------------------------------------------------------------------
% OUTPUT                                                                                                    
% ---------------------------------------------------------------------------
% sysStruct        - Verified system structure in the sysStruct format
% probStruct       - Verified problem structure in the probStruct format
%
% see also MPT_VERIFYSYSSTRUCT, MPT_VERIFYPROBSTRUCT
%

% $Id: mpt_verifySysProb.m,v 1.4 2005/06/06 15:10:34 kvasnica Exp $
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

error(nargchk(2,3,nargin));

global mptOptions;

if ~isstruct(mptOptions),
    mpt_error;
end

if nargin<3
    Options=[];
end
if ~isfield(Options,'verbose'),
    Options.verbose=mptOptions.verbose;
end
if ~isfield(Options, 'sysstructname')
    Options.sysstructname = inputname(1);
end
if ~isfield(Options, 'probstructname')
    Options.probstructname = inputname(2);
end
if ~isfield(Options, 'guierrors'),
    Options.guierrors = 0;
end
if ~isfield(Options, 'forceverify'),
    Options.forceverify = 0;
end

ssn = Options.sysstructname;
psn = Options.probstructname;

if ~isfield(sysStruct,'verified') | Options.forceverify,
    sysStruct = mpt_verifySysStruct(sysStruct, Options);
end
if ~isfield(probStruct,'verified') | Options.forceverify,
    probStruct = mpt_verifyProbStruct(probStruct, Options);
end

[nx,nu,ny,ndyn,nbool,ubool] = mpt_sysStructInfo(sysStruct);

if iscell(sysStruct.A) & isfield(probStruct, 'Nc')
    if probStruct.Nc ~= probStruct.N,
        if Options.guierrors,
            error('Control horizon must be equal to the prediction horizon for PWA systems!');
        else
            error(['"' psn '.Nc" must be equal to "' psn '".N" for PWA systems!']);
        end
    end
end

Dnonzero = 0;
if iscell(sysStruct.D),
    for ii=1:ndyn,
        if any(sysStruct.D{ii}~=0),
            Dnonzero=1;
        end
    end
else
    Dnonzero = any(sysStruct.D~=0);
end

if Dnonzero & probStruct.y0bounds == 0,
    if Options.guierrors,
        error('y0bounds must be set to true for non-zero system "D" matrix!');
    else
        error(['"' psn '.y0bounds" must be set to 1 for non-zero ' ssn '.D matrix!']);
    end
end

if isfield(probStruct,'Qy'),
    if iscell(probStruct.Qy) & iscell(sysStruct.A),
        error(['time-varying "' psn '.Qy" not allowed for PWA systems!']);
    end
end

if iscell(probStruct.Q),
    for ii=1:length(probStruct.Q),
        if size(probStruct.Q{ii},2)~=nx,
            error(sprintf('all entries of "%s.Q" must be a %dx%d matrix!', psn,nx,nx));
        end
    end
elseif size(probStruct.Q,2)~=nx,
    if Options.guierrors,
        error(sprintf('Penalty on states must be a %dx%d matrix!',nx,nx));
    else
        error(sprintf('"%s.Q" must be a %dx%d matrix!',psn,nx,nx));
    end
end

if isfield(probStruct,'Qy'),
    if iscell(probStruct.Qy),
        for ii=1:length(probStruct.Qy),
            if size(probStruct.Qy{ii},2)~=ny,
                error(sprintf('all entries of "%s.Qy" must be a %dx%d matrix!',psn,ny,ny));
            end
        end
    elseif size(probStruct.Qy,2)~=ny & ~isfield(sysStruct,'Cy'),
        if Options.guierrors,
            error(sprintf('Penalty on outputs must be a %dx%d matrix!',ny,ny));
        else
            error(sprintf('"%s.Qy" must be a %dx%d matrix!',psn,ny,ny));
        end
    end
end

if iscell(probStruct.R),
    for ii=1:length(probStruct.R),
        if size(probStruct.R{ii},2)~=nu,
            error(sprintf('all entries of "%s.R" must be a %dx%d matrix!',psn,nu,nu));
        end
    end
elseif size(probStruct.R,2)~=nu,
    if Options.guierrors,
        error(sprintf('Penalty on inputs must be a %dx%d matrix!',nu,nu));
    else
        error(sprintf('"%s.R" must be a %dx%d matrix!',psn,nu,nu));
    end
end

if isfield(probStruct,'Qy'),
    if size(probStruct.Qy,2)~=ny & ~isfield(sysStruct,'Cy'),
        if Options.guierrors,
            error(sprintf('Penalty on outputs must be a %dx%d matrix!',ny,ny));
        else
            error(sprintf('"%s.Qy" must be a %dx%d matrix!',psn,ny,ny));
        end
    end
end
if isfield(probStruct,'Rdu'),
    if size(probStruct.Rdu,2)~=nu,
        if Options.guierrors,
            error(sprintf('Penalty on delta U must be a %dx%d matrix!',nu,nu));
        else
            error(sprintf('"%s.Rdu" must be a %dx%d matrix!',psn,nu,nu));
        end
    end
end

if isfield(sysStruct,'Uexcl'),
    if size(sysStruct.Uexcl,2)~=nbool,
        error(sprintf('"%s.Uexcl" has to be a matrix with %d columns!',ssn,nbool));
    end
end

if isfield(probStruct,'yref'),
    if isfield(sysStruct, 'Cy'),
        if isfield(sysStruct.Cy),
            ny = size(sysStruct.Cy{1},1);
        else
            ny = size(sysStruct.Cy,1);
        end
    end
    if length(probStruct.yref(:))~=ny
        if Options.guierrors,
            error(sprintf('Reference point must be a %dx1 vector!',ny));
        else
            error(['"' psn '.yref" has to be a ' num2str(ny) 'x1 vector!']);
        end
    end
end

if probStruct.Tconstraint==2 & dimension(probStruct.Tset)~=nx,
    if Options.guierrors,
        error(sprintf('Target set must be a polytope in %dD!', nx));
    else
        error(sprintf('%s.Tset must be a polytope in %dD!', psn, nx));
    end
end

if isfield(probStruct, 'Qy') & probStruct.Tconstraint==1,
    disp('WARNING: Closed-loop stability not guaranteed for penalties on outputs.');
end
