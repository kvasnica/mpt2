%MPT_TEST_PWA Extensive test of MPT for a 2D/3D PWA systems

% ---------------------------------------------------------------------------
% Legal note:
%     This file is free software; you can redistribute it and/or modify it
%     under the terms of the GNU General Public License as published by
%     the Free Software Foundation; either version 2, or (at your option)
%     any later version.
%
%     This file is distributed in the hope that it will be useful, but WITHOUT
%     ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
%     or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
%     License for more details.
%
%     You should have received a copy of the GNU General Public License
%     along with this file; see the file COPYING. If not, write to the Free
%     Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
%     02111-1307, USA. 
%
% ---------------------------------------------------------------------------


% default test
mpt_runTests('pwa_def','all');

% y0bounds=1
mpt_runTests('pwa_def','all','probStruct.y0bounds=1;');

% target set
mpt_runTests('pwa_def','all','probStruct.Tset=unitbox(2,1); probStruct.Tconstraint=2;');

% target set & feedback pre-stabilization
mpt_runTests('pwa_def','all','probStruct.Tset=unitbox(2,1); probStruct.Tconstraint=2; probStruct.feedback=1;','T1');

% non-LQR end point penalty
mpt_runTests('pwa_def','all','probStruct.Tconstraint=0; probStruct.P_N=eye(2);');

% zero end point penalty
mpt_runTests('pwa_def','all','probStruct.Tconstraint=0; probStruct.P_N=zeros(2);');

% feedback pre-stabilization
mpt_runTests('pwa_def','all','probStruct.feedback=1;');

% output regulation
mpt_runTests('pwa_def','all','probStruct.yref=[2;0]; probStruct.Qy=1000*eye(2);');

% output regulation & feedback prestabilization
mpt_runTests('pwa_def','all','probStruct.yref=[2;0]; probStruct.Qy=1000*eye(2); probStruct.feedback=1;');

% non-symmetrical constraints on U
mpt_runTests('pwa_def','all','sysStruct.umax=2; sysStruct.umin=-1;');

% non-symmetrical constraints on U
mpt_runTests('pwa_def','all','sysStruct.umax=1; sysStruct.umin=-2;');

% non-symmetrical constraints on U & feedback pre-stabilization
mpt_runTests('pwa_def','all','sysStruct.umax=2; sysStruct.umin=-1; probStruct.feedback=1;');


%-----------------------------------------------------------------
% % non-symmetrical constraints on Y
% mpt_runTests('pwa_def','all','sysStruct.ymax=[5;4]; sysStruct.ymin=[-4.5;-3];');
% 
% % non-symmetrical constraints on Y & feedback pre-stabilization
% mpt_runTests('pwa_def','all','sysStruct.ymax=[5;4]; sysStruct.ymin=[-4.5;-3]; probStruct.feedback=1;');

% % free state tracking
% mpt_runTests('pwa_def','all','probStruct.tracking=1;');
% 
% % free state tracking & feedback pre-stabilization
% mpt_runTests('pwa_def','all','probStruct.tracking=1; probStruct.feedback=1;');