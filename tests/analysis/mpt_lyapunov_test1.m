function mpt_lyapunov_test1

% tests how we cope with lower-dimensional noise in V-representation
load ctrl_lowdimnoise_inv

% only a common quadratic lyap function can be computed when noise is present
a = mpt_lyapunov(ctrl, 'quad');
mbg_assertequal(a.details.lyapunov.feasible, 1);

fprintf('\nPWQ function cannot be computed due to noise\n');
lasterr('');
try
    a = mpt_lyapunov(ctrl, 'pwq');
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
    
fprintf('\nPWP function cannot be computed due to noise\n');
lasterr('');
try
    a = mpt_lyapunov(ctrl, 'pwp');
    worked = 1;
catch
    worked = 0;
end
disp(lasterr);
mbg_assertfalse(worked);
    