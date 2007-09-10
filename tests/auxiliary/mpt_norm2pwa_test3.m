function mpt_norm2pwa_test3

% tests that we properly check wrong dimension of Pn
% http://control.ee.ethz.ch/~mpt/hg/mpt-26x/rev/9785b2294fd2

% here Pn has wrong dimension
Q = eye(3);
Options.Pn = unitbox(2);



%disp('Testing the YALMIP-based method (expecting nice error message)');
Options.method = 1;
lasterr('');
try
    P = mpt_norm2pwa(Q, 1, Options);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'the dimension of P and Pn don''t match');

%disp('Testing the enumeration method (expecting nice error message)');
Options.method = 2;
lasterr('');
try
    P = mpt_norm2pwa(Q, 1, Options);
    worked = 1;
catch
    worked = 0;
end
mbg_assertfalse(worked);
mbg_assertcontains(lasterr, 'the dimension of P and Pn don''t match');
