function mpt_setup
%MPT_SETUP Main setup function

% $Id: mpt_setup.m,v 1.3 2005/04/04 20:49:55 kvasnica Exp $
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

global mptOptions

if ~isstruct(mptOptions)
    mpt_error;
end

numrel = 0;
try
    release = version('-release');
    numrel = str2num(release);
end
if numrel<12
    fprintf('\nWARNING: The GUI was not tested with MATLAB 5.x and older, full functionality is not guaranteed.\n\n');
    fprintf('Press any key to continue...');
    pause
    fprintf('\n');
end
w = warning('off');
mpt_guiSetup_export;
warning(w);