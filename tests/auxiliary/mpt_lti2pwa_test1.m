function mpt_lti2pwa_test1

% mpt_lti2pwa() dislikes when there ar no output constraints defined
%
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/94901b1914d5

Double_Integrator;
sysStruct.xmax = sysStruct.ymax;
sysStruct.xmin = sysStruct.ymin;
sysStruct = rmfield(sysStruct, 'ymax');
sysStruct = rmfield(sysStruct, 'ymin');

% unpatched mpt_lti2pwa() breaks here
sst = mpt_lti2pwa(sysStruct);
